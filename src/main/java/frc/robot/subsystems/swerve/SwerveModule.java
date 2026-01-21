package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Pure-logic swerve module controller.
 *
 * <p>This class encapsulates the mathematical transformations for a single swerve module without
 * any hardware dependencies. It converts high-level setpoints (SwerveModuleState) into low-level
 * motor commands.
 *
 * <p><strong>Design Rationale</strong>: Separating logic from IO enables:
 *
 * <ul>
 *   <li>Unit testing without hardware
 *   <li>Consistent behavior across simulation and real robot
 *   <li>Clear separation of concerns
 * </ul>
 *
 * <p><strong>Key Optimizations</strong>:
 *
 * <ul>
 *   <li><strong>Wheel Direction Optimization</strong>: Minimizes rotation by potentially reversing
 *       drive direction (via WPILib's optimize()).
 *   <li><strong>Cosine Scaling</strong>: Reduces wheel scrub during steering by scaling speed
 *       proportional to alignment with target heading.
 * </ul>
 */
public class SwerveModule {

  /**
   * Output container for motor commands.
   *
   * <p>Populated by {@link #updateSetpoint} and read by the Drive subsystem to apply to hardware.
   */
  public static class ModuleIO {
    /** Drive motor command: velocity (rot/s) in closed-loop, or voltage in open-loop. */
    public double driveDemand;

    /** Steer motor command: target angle in rotations. */
    public double steerDemand;

    /** True if using open-loop (voltage) control for drive. */
    public boolean isOpenLoop = false;
  }

  private final int mModuleIndex;
  private final ModuleIO mModuleIO = new ModuleIO();

  /**
   * Constructs a swerve module controller.
   *
   * @param index Module index (0=FL, 1=FR, 2=BL, 3=BR).
   */
  public SwerveModule(int index) {
    mModuleIndex = index;
  }

  /**
   * Calculates motor commands for a desired module state.
   *
   * <p>This is the main logic step called each control cycle. It performs:
   *
   * <ol>
   *   <li>Wheel direction optimization (minimize rotation)
   *   <li>Cosine scaling for scrub reduction
   *   <li>Unit conversion to motor native units
   * </ol>
   *
   * <p><strong>Cosine Scaling Formula</strong>:
   *
   * <pre>
   * v_scaled = v_desired * |cos(θ_error)|
   * </pre>
   *
   * This reduces commanded velocity when the wheel is misaligned, preventing the robot from
   * drifting off-course during rapid direction changes.
   *
   * @param desiredState The target state from kinematics (speed in m/s, angle).
   * @param currentSteerAngleRotations Current wheel angle from encoder (rotations).
   * @param isOpenLoop True for voltage control, false for velocity closed-loop.
   * @return The computed motor commands (cached in internal ModuleIO).
   */
  public ModuleIO updateSetpoint(
      SwerveModuleState desiredState, double currentSteerAngleRotations, boolean isOpenLoop) {

    // NOTE: Wheel direction optimization (flip drive to minimize steer rotation) is
    // performed
    // in SwerveSetpointGenerator.generateSetpoint() using mPrevSetpoint as
    // reference.
    // We do NOT optimize here again, because using a different reference angle
    // (actual encoder)
    // could produce inconsistent results and cause the robot to drive at an angle.

    Rotation2d currentAngle = Rotation2d.fromRotations(currentSteerAngleRotations);

    // Cosine scaling: reduce commanded speed based on angular error
    // This prevents wheel scrub when the wheel is still turning to the target
    // angle.
    Rotation2d error = desiredState.angle.minus(currentAngle);
    double cosineScalar = error.getCos();
    double scaledSpeed = desiredState.speedMetersPerSecond * Math.abs(cosineScalar);

    // Convert to motor commands
    mModuleIO.isOpenLoop = isOpenLoop;
    if (isOpenLoop) {
      // Open loop: speed ratio → voltage (assuming 12V nominal)
      mModuleIO.driveDemand = (scaledSpeed / Constants.Swerve.kMaxDriveVelocity) * 12.0;
    } else {
      // Closed loop: convert m/s → rotations/s for TalonFX
      double velocityRotPerSec = scaledSpeed / Constants.Swerve.Control.kWheelCircumference;
      mModuleIO.driveDemand = velocityRotPerSec;
    }

    mModuleIO.steerDemand = desiredState.angle.getRotations();

    return mModuleIO;
  }

  /**
   * Returns the current module state (velocity + angle).
   *
   * @param driveVelocityRotPerSec Drive motor velocity in rotations per second.
   * @param steerPositionRotations Steer motor position in rotations.
   * @return The current SwerveModuleState.
   */
  public SwerveModuleState getState(double driveVelocityRotPerSec, double steerPositionRotations) {
    double speedMetersPerSecond =
        driveVelocityRotPerSec * Constants.Swerve.Control.kWheelCircumference;
    return new SwerveModuleState(
        speedMetersPerSecond, Rotation2d.fromRotations(steerPositionRotations));
  }

  /**
   * Returns the current module position (distance + angle).
   *
   * <p><strong>CRITICAL</strong>: This method MUST apply {@code kDrivePositionCoefficient}
   * consistently with {@link #updatePosition}. The WPILib {@code
   * SwerveDriveOdometry.resetPosition()} uses this method to record the baseline for differential
   * calculations. If the coefficient is not applied here but is applied in {@code
   * updatePosition()}, the odometry will exhibit a systematic offset proportional to the encoder's
   * accumulated rotations at reset time.
   *
   * @param drivePositionRotations Drive motor position in rotations.
   * @param steerPositionRotations Steer motor position in rotations.
   * @return The current SwerveModulePosition with corrected distance.
   */
  public SwerveModulePosition getPosition(
      double drivePositionRotations, double steerPositionRotations) {
    double distanceMeters =
        drivePositionRotations
            * Constants.Swerve.Control.kWheelCircumference
            * Constants.Swerve.Control.kDrivePositionCoefficient;
    return new SwerveModulePosition(
        distanceMeters, Rotation2d.fromRotations(steerPositionRotations));
  }

  /**
   * Updates an existing SwerveModulePosition in-place (zero-allocation).
   *
   * <p>Used to avoid garbage collection pressure in the hot path. The only unavoidable allocation
   * is the Rotation2d object, which is immutable.
   *
   * @param drivePositionRotations Drive motor position in rotations.
   * @param steerPositionRotations Steer motor position in rotations.
   * @param out The SwerveModulePosition to update.
   */
  public void updatePosition(
      double drivePositionRotations, double steerPositionRotations, SwerveModulePosition out) {
    double distanceMeters = drivePositionRotations * Constants.Swerve.Control.kWheelCircumference;
    out.distanceMeters = distanceMeters * Constants.Swerve.Control.kDrivePositionCoefficient;
    out.angle = Rotation2d.fromRotations(steerPositionRotations);
  }
}
