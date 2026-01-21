package frc.robot.subsystems.intake;

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
 * Real hardware implementation of IntakeWheelIO using CTRE Phoenix 6.
 *
 * <p>This class manages CAN communication with the intake wheel TalonFX motor.
 *
 * <p><strong>Control Strategy</strong>: Uses VoltageOut control for consistent output regardless of
 * battery voltage fluctuations. This is preferred over DutyCycleOut for mechanisms where consistent
 * force/torque is more important than consistent speed.
 *
 * @see IntakeWheelIO
 */
public class IntakeWheelIOReal implements IntakeWheelIO {
  private final TalonFX mMotor;

  // --- Cached Status Signals ---
  private final StatusSignal<AngularVelocity> mVelocity;
  private final StatusSignal<Voltage> mAppliedVolts;
  private final StatusSignal<Current> mCurrent;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Object (reused to avoid allocation) ---
  private final VoltageOut mVoltageOut = new VoltageOut(0);

  /** Initializes the intake wheel motor and configures current limits. */
  public IntakeWheelIOReal() {
    mMotor = new TalonFX(Constants.Intake.kIntakeMotorId);

    // --- Motor Configuration ---
    var config = new TalonFXConfiguration();

    // Brake mode holds position when stopped, preventing game piece rollback
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Inversion state based on physical mounting
    config.MotorOutput.Inverted =
        Constants.Intake.kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Supply current limiting to prevent stall damage and brownouts
    config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Intake.kSupplyCurrentLimitEnable;

    mMotor.getConfigurator().apply(config);

    // --- Cache Status Signals ---
    mVelocity = mMotor.getVelocity();
    mAppliedVolts = mMotor.getMotorVoltage();
    mCurrent = mMotor.getSupplyCurrent();

    mAllSignals = new BaseStatusSignal[] {mVelocity, mAppliedVolts, mCurrent};

    // Configure 50Hz update rate (sufficient for intake status monitoring)
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(mAllSignals);

    inputs.velocityRotationsPerSec = mVelocity.getValueAsDouble();
    inputs.appliedVolts = mAppliedVolts.getValueAsDouble();
    inputs.currentAmps = mCurrent.getValueAsDouble();

    // Populate connection status (Passive Check)
    inputs.motorConnected = mVelocity.getStatus().isOK();
  }

  @Override
  public void setVoltage(double volts) {
    mMotor.setControl(mVoltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    mMotor.setControl(mVoltageOut.withOutput(0));
  }

  @Override
  public int getMotorFirmwareVersion() {
    var versionSignal = mMotor.getVersion();
    versionSignal.refresh();
    if (versionSignal.getStatus().isOK()) {
      return versionSignal.getValue();
    }
    return 0;
  }
}
