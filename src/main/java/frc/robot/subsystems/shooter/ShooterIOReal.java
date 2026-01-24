package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Real hardware implementation of ShooterIO using CTRE Phoenix 6.
 *
 * <p>This class manages CAN communication with the dual-motor shooter mechanism.
 *
 * <p><strong>Motor Configuration</strong>:
 *
 * <ul>
 *   <li>Right Motor (ID from Constants): Leader motor, inverted (Clockwise Positive).
 *   <li>Left Motor (ID from Constants): Follower motor, inverted via Opposed alignment.
 * </ul>
 *
 * <p><strong>Control Strategy</strong>: Uses open-loop voltage control for simple flywheel
 * spinning. Both motors are physically mirrored, so the left motor follows the right motor with
 * opposing voltage to produce synchronized counter-rotation.
 *
 * <p><strong>First Principles</strong>: In a dual-flywheel shooter where both flywheels spin inward
 * toward the ball, both motors must produce torque in opposite physical directions (mirrored).
 * Since they face each other, the "Opposed" follower mode ensures +V on Leader becomes -V on
 * Follower, achieving inward counter-rotation.
 *
 * @see ShooterIO
 */
public class ShooterIOReal implements ShooterIO {
  private final TalonFX mRightMotor; // Leader
  private final TalonFX mLeftMotor; // Follower

  // --- Cached Status Signals (Right Motor) ---
  private final StatusSignal<Angle> mRightPosition;
  private final StatusSignal<AngularVelocity> mRightVelocity;
  private final StatusSignal<Voltage> mRightAppliedVolts;
  private final StatusSignal<Current> mRightCurrent;

  // --- Cached Status Signals (Left Motor) ---
  private final StatusSignal<AngularVelocity> mLeftVelocity;
  private final StatusSignal<Voltage> mLeftAppliedVolts;
  private final StatusSignal<Current> mLeftCurrent;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Objects (reused to avoid allocation) ---
  private final VoltageOut mVoltageOut = new VoltageOut(0);
  private final Follower mFollower;

  /**
   * Initializes the shooter motors and configures leader-follower relationship.
   *
   * <p>The right motor acts as the leader. The left motor follows in opposing direction for
   * synchronized dual-flywheel operation where flywheels spin inward.
   */
  public ShooterIOReal() {
    mRightMotor = new TalonFX(Constants.Shooter.kRightMotorId);
    mLeftMotor = new TalonFX(Constants.Shooter.kLeftMotorId);

    // --- Right Motor (Leader) Configuration ---
    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Inverted: CounterClockwise_Positive (Standard) to match IntakePivot
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Current limits
    rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.kSupplyCurrentLimit;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Shooter.kSupplyCurrentLimitEnable;
    rightConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.kStatorCurrentLimit;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.Shooter.kStatorCurrentLimitEnable;

    mRightMotor.getConfigurator().apply(rightConfig);

    // --- Left Motor (Follower) Configuration ---
    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Left Motor: Same base config (CounterClockwise_Positive)
    // The "Opposed" follower mode will invert the voltage for mirrored rotation.
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Current limits (same as leader)
    leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.kSupplyCurrentLimit;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Shooter.kSupplyCurrentLimitEnable;
    leftConfig.CurrentLimits.StatorCurrentLimit = Constants.Shooter.kStatorCurrentLimit;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.Shooter.kStatorCurrentLimitEnable;

    mLeftMotor.getConfigurator().apply(leftConfig);

    // --- Follower Control: Left follows Right, Opposed = true ---
    // Opposed alignment ensures +V on Leader becomes -V on Follower.
    // Result: Both flywheels spin inward (mirrored rotation) for shooter firing
    // action.
    mFollower = new Follower(Constants.Shooter.kRightMotorId, MotorAlignmentValue.Opposed);
    mLeftMotor.setControl(mFollower);

    // --- Cache Status Signals ---
    mRightPosition = mRightMotor.getPosition();
    mRightVelocity = mRightMotor.getVelocity();
    mRightAppliedVolts = mRightMotor.getMotorVoltage();
    mRightCurrent = mRightMotor.getSupplyCurrent();
    mLeftVelocity = mLeftMotor.getVelocity();
    mLeftAppliedVolts = mLeftMotor.getMotorVoltage();
    mLeftCurrent = mLeftMotor.getSupplyCurrent();

    mAllSignals =
        new BaseStatusSignal[] {
          mRightPosition,
          mRightVelocity,
          mRightAppliedVolts,
          mRightCurrent,
          mLeftVelocity,
          mLeftAppliedVolts,
          mLeftCurrent
        };

    // Configure 50Hz update rate
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(mAllSignals);

    inputs.rightPositionRot = mRightPosition.getValueAsDouble();
    inputs.rightVelocityRotPerSec = mRightVelocity.getValueAsDouble();
    inputs.leftVelocityRotPerSec = mLeftVelocity.getValueAsDouble();
    inputs.rightAppliedVolts = mRightAppliedVolts.getValueAsDouble();
    inputs.leftAppliedVolts = mLeftAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = mRightCurrent.getValueAsDouble();
    inputs.leftCurrentAmps = mLeftCurrent.getValueAsDouble();

    // Connection status (Passive Check)
    inputs.rightMotorConnected = mRightVelocity.getStatus().isOK();
    inputs.leftMotorConnected = mLeftVelocity.getStatus().isOK();
  }

  @Override
  public void setVoltage(double volts) {
    // Command voltage to right motor (leader)
    // Left motor is configured as a Follower (Opposed), so it will automatically
    // apply the same voltage magnitude but in the opposite direction (mirrored
    // spin).
    mRightMotor.setControl(mVoltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    mRightMotor.setControl(mVoltageOut.withOutput(0));
    // Re-establish follower mode for consistency
    mLeftMotor.setControl(mFollower);
    mRightMotor.setControl(mVoltageOut.withOutput(0));
  }

  @Override
  public int getRightMotorFirmwareVersion() {
    var versionSignal = mRightMotor.getVersion();
    versionSignal.refresh();
    if (versionSignal.getStatus().isOK()) {
      return versionSignal.getValue();
    }
    return 0;
  }

  @Override
  public int getLeftMotorFirmwareVersion() {
    var versionSignal = mLeftMotor.getVersion();
    versionSignal.refresh();
    if (versionSignal.getStatus().isOK()) {
      return versionSignal.getValue();
    }
    return 0;
  }
}
