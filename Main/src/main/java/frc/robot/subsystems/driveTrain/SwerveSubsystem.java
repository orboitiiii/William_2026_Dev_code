package frc.robot.subsystems.driveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.core.RobotIntentState;
import frc.robot.lib.core.RobotOperatingContext;
import frc.robot.lib.core.StateDrivenSubsystem;
import frc.robot.lib.swerve.Gyro;
import frc.robot.lib.swerve.SwerveController;
import java.util.Optional;

/**
 * The SwerveSubsystem manages the SwerveController. It translates the global RobotIntentState into
 * local Swerve states (e.g., OPEN_LOOP, X_LOCK, DISABLED).
 */
public final class SwerveSubsystem extends StateDrivenSubsystem<SwerveSubsystem.State> {

  /** Local states for the Swerve subsystem. */
  public enum State {
    OPEN_LOOP_TELEOP, // Standard driver control
    X_LOCK, // Lock wheels in X pattern
    DISABLED // Motors off
  }

  private final SwerveController swerveController;
  private final Gyro gyro;

  public SwerveSubsystem(SwerveController swerveController, Gyro gyro) {
    this.swerveController = swerveController;
    this.gyro = gyro;
  }

  /**
   * Gets the current estimated pose from the SwerveController.
   *
   * @return The current Pose2d.
   */
  public Pose2d getPose() {
    return swerveController.getEstimatedRobotPose();
  }

  /**
   * Resets the odometry to the robot's current yaw.
   *
   * @param gyroYaw The current yaw from the gyro.
   */
  public void resetOdometry(Rotation2d gyroYaw) {
    swerveController.reset(new Pose2d(getPose().getTranslation(), gyroYaw));
  }

  /**
   * Updates the pose estimation. This is called from SubsystemManager.
   *
   * @param visionPose Optional vision pose.
   * @param visionFom Optional vision Figure of Merit.
   * @param avgTagArea Optional vision tag area.
   */
  public void updateOdometry(
      final Optional<Pose2d> visionPose,
      final Optional<Double> visionFom,
      final double avgTagAreaPercentage) {

    // Get gyro data
    Rotation2d yaw = gyro.getYaw();
    double yawRate = gyro.getYawRateRadPerSec();

    // Call the controller's update method
    swerveController.updatePoseEstimation(
        yaw.getRadians(), yawRate, visionPose, visionFom, avgTagAreaPercentage);
  }

  /**
   * This is the local state machine for the Swerve subsystem. It translates the *global*
   * RobotIntentState into a *local* SwerveSubsystem.State.
   */
  @Override
  protected State transition(RobotIntentState desired, RobotOperatingContext ctx) {
    switch (desired) {
      case DISABLED:
        return State.DISABLED;
      case TRAVEL:
        return State.OPEN_LOOP_TELEOP;
      default:
        // Default safety state
        return State.DISABLED;
    }
  }

  /** This handles the *local* state. */
  @Override
  protected void handle(State state, RobotOperatingContext ctx) {
    switch (state) {
      case DISABLED:
        swerveController.stopAllModules();
        break;
      case X_LOCK:
        swerveController.engageXLock();
        break;
      case OPEN_LOOP_TELEOP:
      default:
        {
          // Get desired speeds from RobotOperatingContext
          ChassisSpeeds cmd = ctx.driverChassisSpeeds;

          // Get current pose rotation for field-relative control
          // We use the supplier here to get the *latest* pose
          Rotation2d robotHeading = ctx.robotPose.get().getRotation();

          swerveController.executeFieldRelativeCommand(cmd, robotHeading);
          break;
        }
    }
  }
}
