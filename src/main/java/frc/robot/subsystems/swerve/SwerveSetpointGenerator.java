package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Kinematic Feasibility Filter for swerve drive.
 *
 * <p>This class ensures that commanded module states are physically achievable given the robot's
 * traction limits. If the requested acceleration exceeds what friction can provide, all modules are
 * scaled down proportionally to prevent wheel slip.
 *
 * <p><strong>Inspiration</strong>: Based on Team 1690's kinematic feasibility approach, which
 * treats each wheel as an independent traction-limited actuator.
 *
 * <p><strong>Physics Basis</strong>:
 *
 * <pre>
 * Maximum acceleration without slip: a_max = μ * g
 *
 * where:
 *   μ = coefficient of friction (~1.1 for FRC carpet)
 *   g = gravitational acceleration (9.81 m/s²)
 * </pre>
 *
 * <p><strong>Algorithm</strong>:
 *
 * <ol>
 *   <li>Compute desired acceleration vector for each module.
 *   <li>Find the maximum acceleration magnitude across all modules.
 *   <li>If max > traction limit, compute scale factor = limit / max.
 *   <li>Apply scale factor to all modules uniformly.
 * </ol>
 *
 * @see SwerveSlipDetector
 */
public class SwerveSetpointGenerator {
  private final SwerveDriveKinematics mKinematics;

  /** Previous setpoint for acceleration calculation (dt derivative). */
  private SwerveModuleState[] mPrevSetpoint;

  /** Previous chassis speeds for global acceleration limiting. */
  private ChassisSpeeds mPrevChassisSpeeds = new ChassisSpeeds();

  /** Filtered voltage to prevent oscillation in the feasibility filter. */
  private final double[] mFilteredVoltages = new double[] {12.0, 12.0, 12.0, 12.0};

  /**
   * Constructs a setpoint generator.
   *
   * @param kinematics The robot's swerve kinematics instance.
   */
  public SwerveSetpointGenerator(SwerveDriveKinematics kinematics) {
    mKinematics = kinematics;
    mPrevSetpoint = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      mPrevSetpoint[i] = new SwerveModuleState(0.0, new Rotation2d());
    }
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   * @return Optimized swerve module state.
   * @deprecated Use the instance method instead.
   */
  @Deprecated
  private SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  /**
   * Generates feasible module states from desired chassis speeds.
   *
   * <p>The returned states are guaranteed to be achievable without exceeding the robot's traction
   * or motor torque limits.
   *
   * @param desiredSpeeds The commanded chassis speeds (field or robot relative).
   * @param driveVoltages Array of 4 supply voltages, one for each drive motor.
   * @return Array of 4 feasible SwerveModuleStates (FL, FR, BL, BR).
   */
  public SwerveModuleState[] generateSetpoint(ChassisSpeeds desiredSpeeds, double[] driveVoltages) {
    // Update Voltage Filter
    // Simple LPF: y[n] = y[n-1] * 0.85 + x[n] * 0.15
    // This dampens the "instant" voltage sag to prevent oscillation in the velocity
    // cap.
    for (int i = 0; i < 4; i++) {
      mFilteredVoltages[i] = mFilteredVoltages[i] * 0.85 + driveVoltages[i] * 0.15;
    }

    // Apply feasibility limiting at the ChassisSpeeds level.
    // This scales down the entire acceleration vector (including omega) if any
    // module
    // would exceed its physical friction/torque limit.
    ChassisSpeeds feasibleSpeeds =
        enforceFeasibility(desiredSpeeds, Constants.kLooperDt, mFilteredVoltages);

    // Convert feasible chassis speeds to module states
    SwerveModuleState[] feasibleStates = mKinematics.toSwerveModuleStates(feasibleSpeeds);

    // Speed lock threshold: If speed is microscopic, preserve the last module angle
    // to prevent twitching between directions due to noise vs absolute zero.
    boolean isStationary =
        Math.abs(feasibleSpeeds.vxMetersPerSecond) < 0.01
            && Math.abs(feasibleSpeeds.vyMetersPerSecond) < 0.01
            && Math.abs(feasibleSpeeds.omegaRadiansPerSecond) < 0.01;

    // Optimize wheel directions to minimize rotation.
    // Use previous setpoint angle as reference to ensure continuity.
    for (int i = 0; i < 4; i++) {
      if (isStationary) {
        feasibleStates[i].speedMetersPerSecond = 0.0;
        feasibleStates[i].angle = mPrevSetpoint[i].angle;
      }
      feasibleStates[i] = optimize(feasibleStates[i], mPrevSetpoint[i].angle);
    }

    // Update previous setpoints for next cycle
    mPrevSetpoint = feasibleStates;
    mPrevChassisSpeeds = feasibleSpeeds;
    return feasibleStates;
  }

  /**
   * Enforces physical feasibility by limiting acceleration at the ChassisSpeeds level.
   *
   * <p><strong>Physics Rationale - Global Acceleration Scaling</strong>: When the robot's commanded
   * acceleration exceeds physical limits (motor torque or tire friction), we must scale down the
   * ENTIRE acceleration vector (vx, vy, omega) proportionally. This preserves the intended path
   * curvature and heading rate ratio.
   *
   * <p><strong>Algorithm</strong>:
   *
   * <ol>
   *   <li>Convert desired ChassisSpeeds to module states.
   *   <li>For each module, calculate the required linear acceleration and the maximum allowed
   *       acceleration (min of friction and torque limits).
   *   <li>Find the most constrained module and compute global scale factor k.
   *   <li>Apply k to the entire ChassisSpeeds delta (not just individual module speeds).
   *   <li>This ensures rotation (omega) is also scaled, preventing path curvature distortion.
   * </ol>
   *
   * <p><strong>Why scale omega too?</strong>: In the original implementation, only module speeds
   * were scaled but omega was passed through unchanged. This causes path curvature distortion: when
   * linear speed is reduced but omega stays the same, the robot turns tighter than intended. By
   * scaling the entire ChassisSpeeds delta, we maintain the commanded curvature ratio.
   *
   * @param desiredSpeeds The desired chassis speeds.
   * @param dt Time step for acceleration calculation.
   * @param driveVoltages Array of 4 supply voltages.
   * @return Feasible ChassisSpeeds scaled to respect physical limits.
   */
  private ChassisSpeeds enforceFeasibility(
      ChassisSpeeds desiredSpeeds, double dt, double[] driveVoltages) {

    // Convert desired speeds to module states to analyze per-module requirements
    SwerveModuleState[] idealStates = mKinematics.toSwerveModuleStates(desiredSpeeds);

    // Optimize wheel directions using previous angles (for consistent acceleration
    // calculation)
    for (int i = 0; i < 4; i++) {
      idealStates[i] = optimize(idealStates[i], mPrevSetpoint[i].angle);
    }

    // Physical Friction Limit
    double limitFriction = Constants.Swerve.kMaxDriveAcceleration; // μ * g

    // Calculate required and allowed accelerations for each module
    double[] linearAccelerations = new double[4];
    double[] maxAllowedLinearAccelerations = new double[4];

    for (int i = 0; i < 4; i++) {
      double vCurrent = mPrevSetpoint[i].speedMetersPerSecond;
      double vTarget = idealStates[i].speedMetersPerSecond;

      // --- Torque Limit Calculation ---
      // 1. Motor Speed (rad/s)
      double motorSpeedRadPerSec =
          (Math.abs(vCurrent) / Constants.Swerve.Control.kWheelCircumference)
              * Constants.Swerve.kDriveGearRatio
              * (2 * Math.PI);

      // 2. Available Torque (Nm) at this speed, limited by current
      double torqueCurrentLimit =
          Constants.Swerve.Control.kDriveStatorCurrentLimit
              * Constants.Swerve.kDriveGearbox.KtNMPerAmp;

      // Calculate torque at voltage limit (Dynamic Voltage!)
      double validVoltage = Math.max(0.0, driveVoltages[i]);
      double currentAtVoltage =
          Constants.Swerve.kDriveGearbox.getCurrent(motorSpeedRadPerSec, validVoltage);
      double torqueVoltageLimit = currentAtVoltage * Constants.Swerve.kDriveGearbox.KtNMPerAmp;

      // The feasible torque is the intersection of the two limits
      double maxTorqueBot = Math.min(torqueCurrentLimit, Math.max(0.0, torqueVoltageLimit));

      // 3. Force at Wheel (N) -> Acceleration (m/s^2)
      double maxWheelTorque = maxTorqueBot * Constants.Swerve.kDriveGearRatio;
      double maxForceOneWheel = maxWheelTorque / Constants.Swerve.kWheelRadius;
      double maxMotorAccel = (4.0 * maxForceOneWheel) / Constants.Swerve.kRobotMass;

      // Determine the governing limit for this specific module
      double aMax = Math.min(limitFriction, maxMotorAccel);

      // Linear Acceleration Demand
      double aLin = (vTarget - vCurrent) / dt;
      linearAccelerations[i] = aLin;
      maxAllowedLinearAccelerations[i] = aMax;
    }

    // Determine Global Scaling Factor
    // Find the most constrained module
    double minK = 1.0;
    for (int i = 0; i < 4; i++) {
      double req = Math.abs(linearAccelerations[i]);
      double allowed = maxAllowedLinearAccelerations[i];

      if (req > allowed + 1e-6) {
        double k = allowed / req;
        if (k < minK) {
          minK = k;
        }
      }
    }

    // Apply the global scale factor to the ENTIRE ChassisSpeeds delta
    // This ensures omega is also scaled proportionally, preserving path curvature.
    //
    // delta_v = desiredSpeeds - mPrevChassisSpeeds
    // feasibleSpeeds = mPrevChassisSpeeds + delta_v * minK

    double deltaVx = desiredSpeeds.vxMetersPerSecond - mPrevChassisSpeeds.vxMetersPerSecond;
    double deltaVy = desiredSpeeds.vyMetersPerSecond - mPrevChassisSpeeds.vyMetersPerSecond;
    double deltaOmega =
        desiredSpeeds.omegaRadiansPerSecond - mPrevChassisSpeeds.omegaRadiansPerSecond;

    double feasibleVx = mPrevChassisSpeeds.vxMetersPerSecond + deltaVx * minK;
    double feasibleVy = mPrevChassisSpeeds.vyMetersPerSecond + deltaVy * minK;
    double feasibleOmega = mPrevChassisSpeeds.omegaRadiansPerSecond + deltaOmega * minK;

    return new ChassisSpeeds(feasibleVx, feasibleVy, feasibleOmega);
  }

  /**
   * Resets the previous setpoint state.
   *
   * <p>Call this when the robot transitions from disabled to enabled to prevent large acceleration
   * commands from the stale previous state.
   */
  public void reset() {
    for (int i = 0; i < 4; i++) {
      mPrevSetpoint[i] = new SwerveModuleState(0.0, new Rotation2d());
      mFilteredVoltages[i] = 12.0;
    }
    mPrevChassisSpeeds = new ChassisSpeeds();
  }
}
