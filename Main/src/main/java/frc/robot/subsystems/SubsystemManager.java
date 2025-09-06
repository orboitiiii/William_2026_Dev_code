// ================= SubsystemManager =================
package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.lib.core.RobotContext;
import frc.robot.lib.core.SubsystemNode;
import frc.robot.lib.swerve.SwerveController;
import frc.robot.lib.swerve.SwerveModule;
import frc.robot.lib.swerve.SwerveModuleIo;
import frc.robot.subsystems.driveTrain.SparkMaxModuleIo;
import frc.robot.subsystems.driveTrain.SwerveConstants.DriveConstants;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import java.util.Arrays;

public class SubsystemManager {
  // FL, FR, RL, RR
  private final SwerveModuleIo[] moduleIo =
      new SwerveModuleIo[] {
        new SparkMaxModuleIo(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset),
        new SparkMaxModuleIo(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset),
        new SparkMaxModuleIo(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset),
        new SparkMaxModuleIo(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset)
      };

  private final SwerveModule[] modules =
      Arrays.stream(moduleIo).map(SwerveModule::new).toArray(SwerveModule[]::new);

  private final SwerveController swerveController =
      new SwerveController(DriveConstants.swerveConfig, modules);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(swerveController);

  private final PS5Controller driver = new PS5Controller(0);

  private final SubsystemNode<?>[] subsystems = new SubsystemNode<?>[] {swerveSubsystem};

  public RobotContext readContext() {
    double vx = driver.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond;
    double vy = driver.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond;
    double w = driver.getRightX() * DriveConstants.kMaxAngularSpeed;

    return new RobotContext(() -> new ChassisSpeeds(vx, vy, w), () -> swerveSubsystem.getPose());
  }

  public void operate(frc.robot.lib.core.RobotIntentState desired, RobotContext ctx) {
    for (SubsystemNode<?> node : subsystems) {
      node.operate(desired, ctx);
    }
  }
}
