package frc.robot.lib.core;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class RobotContext {
  public Supplier<ChassisSpeeds> driverDesireSpeeds;
  public Supplier<Pose2d> robotPose;

  public RobotContext(Supplier<ChassisSpeeds> driverDesireSpeeds, Supplier<Pose2d> robotPose) {
    this.driverDesireSpeeds = driverDesireSpeeds;
    this.robotPose = robotPose;
  }
}
