package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
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
  private final StatusSignal<Angle> mSideRollerPosition;
  private final StatusSignal<AngularVelocity> mSideRollerVelocity;
  private final StatusSignal<Voltage> mSideRollerAppliedVolts;
  private final StatusSignal<Current> mSideRollerCurrent;

  // --- Cached Status Signals: Straight Roller ---
  private final StatusSignal<Angle> mStraightRollerPosition;
  private final StatusSignal<AngularVelocity> mStraightRollerVelocity;
  private final StatusSignal<Voltage> mStraightRollerAppliedVolts;
  private final StatusSignal<Current> mStraightRollerCurrent;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Objects (reused to avoid allocation) ---
  private final VoltageOut mSideRollerVoltageOut = new VoltageOut(0);
  private final VoltageOut mStraightRollerVoltageOut = new VoltageOut(0);
  private final VelocityVoltage mSideRollerVelocityOut = new VelocityVoltage(0);
  private final VelocityVoltage mStraightRollerVelocityOut = new VelocityVoltage(0);

  /** Initializes both indexer motors and configures current limits. */
  public IndexerIOReal() {
    mSideRollerMotor = new TalonFX(Constants.Indexer.kSideRollerMotorId, Constants.kCANBusName);
    mStraightRollerMotor =
        new TalonFX(Constants.Indexer.kStraightRollerMotorId, Constants.kCANBusName);

    // --- Configure Side Roller Motor ---
    var sideConfig = new TalonFXConfiguration();
    sideConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    sideConfig.MotorOutput.Inverted =
        Constants.Indexer.kSideRollerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    sideConfig.CurrentLimits.SupplyCurrentLimit = Constants.Indexer.kSupplyCurrentLimit;
    sideConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Indexer.kSupplyCurrentLimitEnable;
    // Slot 0 Gains
    sideConfig.Slot0.kP = Constants.Indexer.kSidekP;
    sideConfig.Slot0.kI = 0.0;
    sideConfig.Slot0.kD = 0.0;
    sideConfig.Slot0.kS = Constants.Indexer.kSidekS;
    sideConfig.Slot0.kV = Constants.Indexer.kSidekV;
    sideConfig.Slot0.kA = Constants.Indexer.kSidekA;

    boolean sideOk =
        frc.robot.util.Phoenix6Util.checkManeuver(
            () -> mSideRollerMotor.getConfigurator().apply(sideConfig),
            "Indexer Side Roller Config");
    Timer.delay(0.1);

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
    // Slot 0 Gains
    straightConfig.Slot0.kP = Constants.Indexer.kStraightkP;
    straightConfig.Slot0.kI = 0.0;
    straightConfig.Slot0.kD = 0.0;
    straightConfig.Slot0.kS = Constants.Indexer.kStraightkS;
    straightConfig.Slot0.kV = Constants.Indexer.kStraightkV;
    straightConfig.Slot0.kA = Constants.Indexer.kStraightkA;

    boolean straightOk =
        frc.robot.util.Phoenix6Util.checkManeuver(
            () -> mStraightRollerMotor.getConfigurator().apply(straightConfig),
            "Indexer Straight Roller Config");
    Timer.delay(0.1);

    // --- Cache Status Signals ---
    mSideRollerPosition = mSideRollerMotor.getPosition();
    mSideRollerVelocity = mSideRollerMotor.getVelocity();
    mSideRollerAppliedVolts = mSideRollerMotor.getMotorVoltage();
    mSideRollerCurrent = mSideRollerMotor.getSupplyCurrent();

    mStraightRollerPosition = mStraightRollerMotor.getPosition();
    mStraightRollerVelocity = mStraightRollerMotor.getVelocity();
    mStraightRollerAppliedVolts = mStraightRollerMotor.getMotorVoltage();
    mStraightRollerCurrent = mStraightRollerMotor.getSupplyCurrent();

    // FAIL-SAFE: Only register signals if configuration succeeded
    // If config failed, the device is likely dead/disconnected, so we shouldn't
    // block on it.
    if (sideOk && straightOk) {
      mAllSignals =
          new BaseStatusSignal[] {
            mSideRollerPosition,
            mSideRollerVelocity,
            mSideRollerAppliedVolts,
            mSideRollerCurrent,
            mStraightRollerPosition,
            mStraightRollerVelocity,
            mStraightRollerAppliedVolts,
            mStraightRollerCurrent
          };
      // Configure 50Hz update rate
      BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);

      // Minimize CAN bus usage by disabling unused status frames
      mSideRollerMotor.optimizeBusUtilization();
      mStraightRollerMotor.optimizeBusUtilization();
    } else {
      System.err.println(
          "CRITICAL: Indexer FAILED config - Excluding from synchronous updates to prevent Loop Overrun.");
      mAllSignals = new BaseStatusSignal[0];
    }

    // PAUSE: Allow CAN buffer to drain
    Timer.delay(0.05);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(mAllSignals);

    // Side Roller
    inputs.sideRollerPositionRot = mSideRollerPosition.getValueAsDouble();
    inputs.sideRollerVelocityRotPerSec = mSideRollerVelocity.getValueAsDouble();
    inputs.sideRollerAppliedVolts = mSideRollerAppliedVolts.getValueAsDouble();
    inputs.sideRollerCurrentAmps = mSideRollerCurrent.getValueAsDouble();
    inputs.sideRollerConnected = mSideRollerVelocity.getStatus().isOK();

    // Straight Roller
    inputs.straightRollerPositionRot = mStraightRollerPosition.getValueAsDouble();
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
    }
  }

  @Override
  public void setSideRollerVoltage(double volts) {
    if (volts == 0.0) {
      mSideRollerMotor.setControl(mSideRollerVoltageOut.withOutput(0));
    } else {
      mSideRollerMotor.setControl(mSideRollerVoltageOut.withOutput(volts));
    }
  }

  @Override
  public void setStraightRollerVoltage(double volts) {
    if (volts == 0.0) {
      mStraightRollerMotor.setControl(mStraightRollerVoltageOut.withOutput(0));
    } else {
      mStraightRollerMotor.setControl(mStraightRollerVoltageOut.withOutput(volts));
    }
  }

  @Override
  public void setSideRollerTargetVelocity(double velocityRotPerSec) {
    mSideRollerMotor.setControl(mSideRollerVelocityOut.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setStraightRollerTargetVelocity(double velocityRotPerSec) {
    mStraightRollerMotor.setControl(mStraightRollerVelocityOut.withVelocity(velocityRotPerSec));
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
