// ================= SubsystemManager =================
package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.lib.core.RobotOperatingContext;
import frc.robot.lib.core.StateDrivenSubsystem;
import frc.robot.lib.swerve.SwerveController;
import frc.robot.lib.swerve.SwerveModule;
import frc.robot.subsystems.driveTrain.Pigeon2Gyro;
import frc.robot.subsystems.driveTrain.SparkMaxModuleIO;
import frc.robot.subsystems.driveTrain.SwerveConstants.DriveConstants;
import frc.robot.subsystems.driveTrain.SwerveConstants.OIConstants;
import frc.robot.subsystems.driveTrain.SwerveSubsystem;
import java.util.Arrays;

public class SubsystemManager {
  // FL, FR, RL, RR
  private final SparkMaxModuleIO[] moduleIo =
      new SparkMaxModuleIO[] {
        new SparkMaxModuleIO(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset),
        new SparkMaxModuleIO(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset),
        new SparkMaxModuleIO(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset),
        new SparkMaxModuleIO(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset)
      };

  private final SwerveModule[] modules =
      Arrays.stream(moduleIo).map(SwerveModule::new).toArray(SwerveModule[]::new);

  private final SwerveController swerveController =
      new SwerveController(DriveConstants.swerveConfig, modules);

  private final Pigeon2Gyro gyro = new Pigeon2Gyro(0, null, false);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(swerveController, gyro);

  private final PS5Controller driver = new PS5Controller(0);

  private final StateDrivenSubsystem<?>[] subsystems =
      new StateDrivenSubsystem<?>[] {swerveSubsystem};

  public RobotOperatingContext readContext() {
    double vx =
        applyDeadband(driver.getLeftY())
            * DriveConstants.kMaxSpeedMetersPerSecond; // PS5 forward is negative
    double vy = applyDeadband(driver.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
    double w = applyDeadband(driver.getRightX()) * DriveConstants.kMaxAngularSpeed;

    return new RobotOperatingContext(new ChassisSpeeds(vx, vy, w), () -> swerveSubsystem.getPose());
  }

  public void operate(frc.robot.lib.core.RobotIntentState desired, RobotOperatingContext ctx) {
    for (StateDrivenSubsystem<?> node : subsystems) {
      node.operate(desired, ctx);
    }
  }

  private static double applyDeadband(double value) {
    double magnitude = Math.abs(value);
    if (magnitude < OIConstants.kDriveDeadband) {
      return 0.0;
    }
    double scaled = (magnitude - OIConstants.kDriveDeadband) / (1.0 - OIConstants.kDriveDeadband);
    return Math.copySign(Math.min(1.0, scaled), value);
  }
}
