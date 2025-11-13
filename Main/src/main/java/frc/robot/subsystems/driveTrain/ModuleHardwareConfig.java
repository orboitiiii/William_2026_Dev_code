package frc.robot.subsystems.driveTrain;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.driveTrain.SwerveConstants.ModuleConstants;

/**
 * Holds all vendor-specific (e.g., SparkMax) hardware configuration objects. This was renamed from
 * 'SwerveConfigs' to avoid naming conflicts.
 */
public final class ModuleHardwareConfig {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI; // Radians

      // --- Driving Motor Configuration ---
      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(40)
          .signals
          .primaryEncoderPositionPeriodMs(5); // 200 Hz

      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0) // m/s
          .quadratureMeasurementPeriod(20)
          .quadratureAverageDepth(64);

      drivingConfig
          .voltageCompensation(12.0)
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(
              SwerveConstants.DriveConstants.swerveConfig.kPSpeed,
              0.0,
              0.0) // Use kP from SwerveConfig
          .outputRange(-1.0, 1.0);

      // --- Turning Motor Configuration ---
      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .signals
          .primaryEncoderPositionPeriodMs(5);

      turningConfig
          .absoluteEncoder
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // rad/s

      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(
              SwerveConstants.DriveConstants.swerveConfig.kPAngle,
              0.0,
              0.0) // Use kP from SwerveConfig
          .outputRange(-1.0, 1.0)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0.0, turningFactor);
    }
  }
}
