package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.AbsoluteSensorRange;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.SwerveModuleType;
import frc.robot.util.Phoenix6Util;

/**
 * Phoenix 6 configuration factory for swerve modules.
 *
 * <p>This class centralizes all motor controller and encoder configuration to ensure consistency
 * across all four modules. Configuration is applied once during robot initialization.
 *
 * <p><strong>Configuration Details</strong>:
 *
 * <ul>
 *   <li><strong>Drive Motor</strong>: Velocity closed-loop with supply current limiting.
 *   <li><strong>Steer Motor</strong>: Position closed-loop using CANcoder as remote sensor.
 *   <li><strong>CANcoder</strong>: Configured with MagnetOffset for zero-point alignment.
 * </ul>
 *
 * <p><strong>Remote Sensor Fusion</strong>: The steer motor uses the CANcoder as its feedback
 * device (RemoteCANcoder mode). This provides absolute position tracking that persists across power
 * cycles, eliminating the need for homing routines.
 *
 * @see <a href="https://v6.docs.ctr-electronics.com/">CTRE Phoenix 6 Documentation</a>
 */
public class SwerveModuleConfigurator {

  /**
   * Configures a complete swerve module (drive motor, steer motor, CANcoder).
   *
   * <p>This method applies all PID gains, current limits, and sensor fusion settings from
   * Constants.
   *
   * @param driveMotor The drive TalonFX instance.
   * @param steerMotor The steer TalonFX instance.
   * @param encoder The CANcoder instance.
   * @param moduleType The type of swerve module (e.g., SDS MK4i).
   * @param steerOffsetRotations The MagnetOffset for this module (rotations).
   * @return true if all configurations were applied successfully, false otherwise.
   */
  public static boolean configure(
      TalonFX driveMotor,
      TalonFX steerMotor,
      CANcoder encoder,
      SwerveModuleType moduleType,
      double steerOffsetRotations) {
    boolean allOk = true;

    // --- CANcoder Configuration ---
    var coderConfig = new CANcoderConfiguration();
    // coderConfig.MagnetSensor.AbsoluteSensorRange =
    // AbsoluteSensorRange.Signed_PlusMinus_Half;
    coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    coderConfig.MagnetSensor.MagnetOffset = steerOffsetRotations;

    allOk &=
        Phoenix6Util.checkManeuver(
            () -> encoder.getConfigurator().apply(coderConfig), "Module Encoder Config");

    Timer.delay(0.1);

    // --- Drive Motor Configuration ---
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    driveConfig.Slot0.kP = Constants.Swerve.Control.kDrivekP;
    driveConfig.Slot0.kI = Constants.Swerve.Control.kDrivekI;
    driveConfig.Slot0.kD = Constants.Swerve.Control.kDrivekD;
    driveConfig.Slot0.kV = Constants.Swerve.Control.kDrivekV;

    driveConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.Swerve.Control.kDriveSupplyCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.Control.kDriveSupplyCurrentLimitEnable;
    driveConfig.CurrentLimits.StatorCurrentLimit =
        Constants.Swerve.Control.kDriveStatorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Swerve.Control.kDriveStatorCurrentLimitEnable;

    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;

    driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kDriveGearRatio;

    allOk &=
        Phoenix6Util.checkManeuver(
            () -> driveMotor.getConfigurator().apply(driveConfig), "Module Drive Motor Config");

    Timer.delay(0.1);

    // --- Steer Motor Configuration ---
    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    steerConfig.Slot0.kP = Constants.Swerve.Control.kSteerkP;
    steerConfig.Slot0.kI = Constants.Swerve.Control.kSteerkI;
    steerConfig.Slot0.kD = Constants.Swerve.Control.kSteerkD;

    steerConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.Swerve.Control.kSteerSupplyCurrentLimit;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.Control.kSteerSupplyCurrentLimitEnable;

    steerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;

    steerConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.RotorToSensorRatio = Constants.Swerve.kSteerGearRatio;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

    allOk &=
        Phoenix6Util.checkManeuver(
            () -> steerMotor.getConfigurator().apply(steerConfig), "Module Steer Motor Config");

    return allOk;
  }
}
