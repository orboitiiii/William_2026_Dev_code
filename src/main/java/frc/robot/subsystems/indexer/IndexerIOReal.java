package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Real hardware implementation of IndexerIO using CTRE Phoenix 6.
 *
 * <p>This class manages CAN communication with two Krakon X44 motors:
 *
 * <ul>
 *   <li><strong>Side Roller</strong>: Transfers game pieces laterally.
 *   <li><strong>Straight Roller</strong>: Transfers game pieces forward/backward.
 * </ul>
 *
 * <p><strong>Control Strategy</strong>: Uses VoltageOut control for consistent output regardless of
 * battery voltage fluctuations. Both motors receive the same voltage command to ensure synchronized
 * operation during indexing.
 *
 * <p><strong>Motor Specifications (Krakon X44)</strong>:
 *
 * <ul>
 *   <li>Stall Torque: 3.82 Nm
 *   <li>Stall Current: 236 A
 *   <li>Free Speed: 7200 RPM
 * </ul>
 *
 * @see IndexerIO
 */
public class IndexerIOReal implements IndexerIO {
  // --- Motor Hardware ---
  private final TalonFX mSideRollerMotor;
  private final TalonFX mStraightRollerMotor;

  // --- Cached Status Signals: Side Roller ---
  private final StatusSignal<AngularVelocity> mSideRollerVelocity;
  private final StatusSignal<Voltage> mSideRollerAppliedVolts;
  private final StatusSignal<Current> mSideRollerCurrent;

  // --- Cached Status Signals: Straight Roller ---
  private final StatusSignal<AngularVelocity> mStraightRollerVelocity;
  private final StatusSignal<Voltage> mStraightRollerAppliedVolts;
  private final StatusSignal<Current> mStraightRollerCurrent;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Objects (reused to avoid allocation) ---
  private final VoltageOut mSideRollerVoltageOut = new VoltageOut(0);
  private final VoltageOut mStraightRollerVoltageOut = new VoltageOut(0);

  /** Initializes both indexer motors and configures current limits. */
  public IndexerIOReal() {
    mSideRollerMotor = new TalonFX(Constants.Indexer.kSideRollerMotorId);
    mStraightRollerMotor = new TalonFX(Constants.Indexer.kStraightRollerMotorId);

    // --- Configure Side Roller Motor ---
    var sideConfig = new TalonFXConfiguration();
    sideConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    sideConfig.MotorOutput.Inverted =
        Constants.Indexer.kSideRollerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    sideConfig.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.kSupplyCurrentLimit;
    sideConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Indexer.kSupplyCurrentLimitEnable;
    mSideRollerMotor.getConfigurator().apply(sideConfig);

    // --- Configure Straight Roller Motor ---
    var straightConfig = new TalonFXConfiguration();
    straightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    straightConfig.MotorOutput.Inverted =
        Constants.Indexer.kStraightRollerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    straightConfig.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.kSupplyCurrentLimit;
    straightConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.Indexer.kSupplyCurrentLimitEnable;
    mStraightRollerMotor.getConfigurator().apply(straightConfig);

    // --- Cache Status Signals ---
    mSideRollerVelocity = mSideRollerMotor.getVelocity();
    mSideRollerAppliedVolts = mSideRollerMotor.getMotorVoltage();
    mSideRollerCurrent = mSideRollerMotor.getSupplyCurrent();

    mStraightRollerVelocity = mStraightRollerMotor.getVelocity();
    mStraightRollerAppliedVolts = mStraightRollerMotor.getMotorVoltage();
    mStraightRollerCurrent = mStraightRollerMotor.getSupplyCurrent();

    mAllSignals =
        new BaseStatusSignal[] {
          mSideRollerVelocity,
          mSideRollerAppliedVolts,
          mSideRollerCurrent,
          mStraightRollerVelocity,
          mStraightRollerAppliedVolts,
          mStraightRollerCurrent
        };

    // Configure 50Hz update rate (sufficient for indexer status monitoring)
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(mAllSignals);

    // Side Roller
    inputs.sideRollerVelocityRotPerSec = mSideRollerVelocity.getValueAsDouble();
    inputs.sideRollerAppliedVolts = mSideRollerAppliedVolts.getValueAsDouble();
    inputs.sideRollerCurrentAmps = mSideRollerCurrent.getValueAsDouble();
    inputs.sideRollerConnected = mSideRollerVelocity.getStatus().isOK();

    // Straight Roller
    inputs.straightRollerVelocityRotPerSec = mStraightRollerVelocity.getValueAsDouble();
    inputs.straightRollerAppliedVolts = mStraightRollerAppliedVolts.getValueAsDouble();
    inputs.straightRollerCurrentAmps = mStraightRollerCurrent.getValueAsDouble();
    inputs.straightRollerConnected = mStraightRollerVelocity.getStatus().isOK();
  }

  @Override
  public void setVoltage(double volts) {
    // Determine if we should run or stop based on the requested voltage (from
    // Indexer.java)
    if (volts == 0.0) {
      stop();
    } else {
      // Use individual constants for side and straight rollers
      mSideRollerMotor.setControl(
          mSideRollerVoltageOut.withOutput(Constants.Indexer.kSideRollerVoltage));
      mStraightRollerMotor.setControl(
          mStraightRollerVoltageOut.withOutput(Constants.Indexer.kStraightRollerVoltage));
    }
  }

  @Override
  public void stop() {
    mSideRollerMotor.setControl(mSideRollerVoltageOut.withOutput(0));
    mStraightRollerMotor.setControl(mStraightRollerVoltageOut.withOutput(0));
  }

  @Override
  public int getSideRollerFirmwareVersion() {
    var versionSignal = mSideRollerMotor.getVersion();
    versionSignal.refresh();
    if (versionSignal.getStatus().isOK()) {
      return versionSignal.getValue();
    }
    return 0;
  }

  @Override
  public int getStraightRollerFirmwareVersion() {
    var versionSignal = mStraightRollerMotor.getVersion();
    versionSignal.refresh();
    if (versionSignal.getStatus().isOK()) {
      return versionSignal.getValue();
    }
    return 0;
  }
}
