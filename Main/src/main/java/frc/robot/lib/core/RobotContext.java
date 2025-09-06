package frc.robot.lib.core;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public final class RobotContext {
  public Supplier<ChassisSpeeds> driverDesireSpeeds;
  public Supplier<Pose2d> robotPose;

  public RobotContext(Supplier<ChassisSpeeds> driverDesireSpeeds, Supplier<Pose2d> robotPose) {
    this.driverDesireSpeeds = driverDesireSpeeds;
    this.robotPose = robotPose;
  }
}
