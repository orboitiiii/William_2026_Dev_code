package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class SwerveModuleConfigurator {

  public static void configure(
      TalonFX driveMotor, TalonFX steerMotor, CANcoder cancoder, double cancoderOffset) {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0.kP = 0.1;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = 0.0;
    driveConfig.Slot0.kV = 0.12;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kDriveGearRatio;

    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.Slot0.kP = 2.0;
    steerConfig.Slot0.kI = 0.0;
    steerConfig.Slot0.kD = 0.0;
    steerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kSteerGearRatio;

    // Magnet Offset
    var cancoderConfig = new com.ctre.phoenix6.configs.CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;
    cancoder.getConfigurator().apply(cancoderConfig);

    driveMotor.getConfigurator().apply(driveConfig);
    steerMotor.getConfigurator().apply(steerConfig);

    // Seed Steer Motor with CANCoder absolute position
    // Wait for a valid signal to ensure we don't seed 0.0 by accident
    var posSignal = cancoder.getAbsolutePosition();
    posSignal.waitForUpdate(0.1);
    steerMotor.setPosition(posSignal.getValueAsDouble());
  }
}
