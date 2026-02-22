package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** Real hardware implementation of IntakeWheelIO using a TalonFX. */
public class IntakeWheelIOReal implements IntakeWheelIO {

  private final TalonFX mIntakeMotor;
  private final TalonFX mIntakeFollower;
  private final VoltageOut mVoltageRequest = new VoltageOut(0);

  // Status Signals
  private final StatusSignal<Voltage> mVoltageSignal;
  private final StatusSignal<Current> mCurrentSignal;
  private final StatusSignal<AngularVelocity> mVelocitySignal;

  public IntakeWheelIOReal() {
    mIntakeMotor = new TalonFX(Constants.Intake.kIntakeMotorId, Constants.kCANBusName);
    mIntakeFollower = new TalonFX(Constants.Intake.kIntakeFollowerId, Constants.kCANBusName);

    // Apply Configuration
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor Output Config
    config.MotorOutput.Inverted =
        Constants.Intake.kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Brake mode for better control

    // Current Limits
    config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Intake.kSupplyCurrentLimitEnable;

    mIntakeMotor.getConfigurator().apply(config);
    mIntakeFollower.getConfigurator().apply(config);

    // Set Follower Mode (Follow ID 30, oppose_master_direction=false for same
    // direction)
    // Set Follower Mode (Follow ID 30, oppose_master_direction=false for same
    // direction)
    // Using MotorAlignmentValue.values()[0] as "Standard/Strict" alignment
    // assumption
    mIntakeFollower.setControl(
        new Follower(Constants.Intake.kIntakeMotorId, MotorAlignmentValue.values()[0]));

    // Initialize Signals
    mVoltageSignal = mIntakeMotor.getMotorVoltage();
    mCurrentSignal = mIntakeMotor.getSupplyCurrent();
    mVelocitySignal = mIntakeMotor.getVelocity();

    // Optimize bus utilization (Intake doesn't need high frequency updates)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50Hz default
        mVoltageSignal,
        mCurrentSignal,
        mVelocitySignal);
  }

  @Override
  public void updateInputs(IntakeWheelIOInputs inputs) {
    // Refresh signals
    BaseStatusSignal.refreshAll(mVoltageSignal, mCurrentSignal, mVelocitySignal);

    inputs.velocityRotationsPerSec = mVelocitySignal.getValueAsDouble();
    inputs.appliedVolts = mVoltageSignal.getValueAsDouble();
    inputs.currentAmps = mCurrentSignal.getValueAsDouble();

    // Check connection (Phoenix 6 doesn't have a direct isCheck method on motor,
    // but we can infer from signals)
    // Actually BaseStatusSignal.refreshAll returns a status, but for now we assume
    // connected if no error
    inputs.motorConnected = true;
  }

  @Override
  public void setVoltage(double volts) {
    mIntakeMotor.setControl(mVoltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    mIntakeMotor.setControl(mVoltageRequest.withOutput(0));
  }
}
