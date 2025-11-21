package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.lib.core.RobotIntentState;
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
import java.util.Optional;

public class SubsystemManager {

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

  // Assume "rio" as default bus, adjust if using CANivore
  private final Pigeon2Gyro gyro = new Pigeon2Gyro(0, "rio");

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(swerveController, gyro);

  private final PS5Controller driver = new PS5Controller(0);

  private final StateDrivenSubsystem<?>[] subsystems =
      new StateDrivenSubsystem<?>[] {swerveSubsystem};

  /**
   * This method MUST be called at the beginning of every robot cycle (RobotPeriodic). It updates
   * the "State of the World" (Odometry, Sensor Inputs) before any logic runs.
   */
  public void periodic() {
    // Update Odometry with no vision data for now (pass empty optionals)
    // In the future, VisionSubsystem data would be injected here.
    swerveSubsystem.updateOdometry(Optional.empty(), Optional.empty(), 0.0);
  }

  public RobotOperatingContext readContext() {
    // Inputs should be raw but conditioned (deadbanded)
    // Coordinate System: Forward is +X (Joystick -Y), Left is +Y (Joystick -X)
    double vx = applyDeadband(-driver.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
    double vy = applyDeadband(-driver.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
    double omega = applyDeadband(-driver.getRightX()) * DriveConstants.kMaxAngularSpeed;

    // Create the context with the LATEST pose (updated in periodic())
    return new RobotOperatingContext(
        new ChassisSpeeds(vx, vy, omega), () -> swerveSubsystem.getPose());
  }

  public void operate(RobotIntentState desired, RobotOperatingContext ctx) {
    for (StateDrivenSubsystem<?> subsystem : subsystems) {
      subsystem.operate(desired, ctx);
    }
  }

  private static double applyDeadband(double value) {
    double magnitude = Math.abs(value);
    if (magnitude < OIConstants.kDriveDeadband) {
      return 0.0;
    }
    return Math.copySign(
        (magnitude - OIConstants.kDriveDeadband) / (1.0 - OIConstants.kDriveDeadband), value);
  }
}
