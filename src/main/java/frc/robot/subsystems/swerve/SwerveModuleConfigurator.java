package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

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
   * @param cancoder The CANcoder instance.
   * @param cancoderOffset The MagnetOffset for this module (rotations).
   */
  public static void configure(
      TalonFX driveMotor, TalonFX steerMotor, CANcoder cancoder, double cancoderOffset) {

    // --- Drive Motor Configuration ---
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Velocity PID gains from SysId characterization
    driveConfig.Slot0.kP = Constants.Swerve.Control.kDrivekP;
    driveConfig.Slot0.kI = Constants.Swerve.Control.kDrivekI;
    driveConfig.Slot0.kD = Constants.Swerve.Control.kDrivekD;
    driveConfig.Slot0.kV = Constants.Swerve.Control.kDrivekV;

    // Current limiting to prevent brownouts during high-torque maneuvers
    driveConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.Swerve.Control.kDriveSupplyCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.Control.kDriveSupplyCurrentLimitEnable;

    // Stator current limit for slip protection (Team 254 uses 80A)
    driveConfig.CurrentLimits.StatorCurrentLimit =
        Constants.Swerve.Control.kDriveStatorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Swerve.Control.kDriveStatorCurrentLimitEnable;

    // Closed-loop ramps for soft-start (prevents initialization overshoot)
    // First Principles: Limiting dV/dt prevents instantaneous torque spikes
    // that cause wheel slip and odometry errors during startup.
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;
    driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;

    // Disable boot/config beeps (Team 254 standard)
    driveConfig.Audio.BeepOnBoot = false;
    driveConfig.Audio.BeepOnConfig = false;

    // Gear ratio for mechanism units (wheel rotations, not rotor rotations)
    driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kDriveGearRatio;

    // --- Steer Motor Configuration ---
    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // SDS modules typically require inverted steer direction
    steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Position PID gains
    steerConfig.Slot0.kP = Constants.Swerve.Control.kSteerkP;
    steerConfig.Slot0.kI = Constants.Swerve.Control.kSteerkI;
    steerConfig.Slot0.kD = Constants.Swerve.Control.kSteerkD;

    // Current limiting for steer motors
    steerConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.Swerve.Control.kSteerSupplyCurrentLimit;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Swerve.Control.kSteerSupplyCurrentLimitEnable;

    // Closed-loop ramps for smooth steering (prevents harsh snapping)
    steerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;
    steerConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;
    steerConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        Constants.Swerve.Control.kClosedLoopRampPeriod;

    // Disable boot/config beeps (Team 254 standard)
    steerConfig.Audio.BeepOnBoot = false;
    steerConfig.Audio.BeepOnConfig = false;

    // Enable continuous wrap for shortest-path steering optimization
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // Remote CANcoder fusion: motor uses CANcoder for position feedback
    steerConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    steerConfig.Feedback.SensorToMechanismRatio = 1.0;
    steerConfig.Feedback.RotorToSensorRatio = Constants.Swerve.kSteerGearRatio;

    // --- CANcoder Configuration ---
    var cancoderConfig = new com.ctre.phoenix6.configs.CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        com.ctre.phoenix6.signals.SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    // Apply configurations to motors
    driveMotor.getConfigurator().apply(driveConfig);
    steerMotor.getConfigurator().apply(steerConfig);
  }
}
