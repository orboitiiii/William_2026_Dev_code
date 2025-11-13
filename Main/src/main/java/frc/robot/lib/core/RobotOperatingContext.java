package frc.robot.lib.core;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

/**
 * Represents the operating context (the "state of the world") for all subsystems. This object is
 * built by the SubsystemManager and passed to all StateDrivenSubsystems during their 'operate'
 * call.
 *
 * <p>It contains processed inputs (like ChassisSpeeds) and sensed data (like Pose).
 */
public final class RobotOperatingContext {
  public final ChassisSpeeds driverChassisSpeeds;
  public final Supplier<Pose2d> robotPose;

  /**
   * Creates a new RobotOperatingContext.
   *
   * @param driverChassisSpeeds The processed, field-relative chassis speeds requested by the
   *     driver.
   * @param robotPose A supplier for the robot's current estimated pose.
   */
  public RobotOperatingContext(ChassisSpeeds driverChassisSpeeds, Supplier<Pose2d> robotPose) {
    this.driverChassisSpeeds = driverChassisSpeeds;
    this.robotPose = robotPose;
  }
}
