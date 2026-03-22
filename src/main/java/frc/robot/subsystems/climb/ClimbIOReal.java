package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
 * Real hardware implementation of ClimbIO using CTRE Phoenix 6.
 *
 * <p>
 * This class manages CAN communication with the climb motor (Kraken X44).
 */
public class ClimbIOReal implements ClimbIO {
  private final TalonFX mMotor;

  // --- Cached Status Signals ---
  private final StatusSignal<Angle> mPosition;
  private final StatusSignal<AngularVelocity> mVelocity;
  private final StatusSignal<Voltage> mAppliedVolts;
  private final StatusSignal<Current> mCurrent;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Objects (reused to avoid allocation) ---
  // Configured to 100Hz update frequency to eliminate control latency when
  // buttons are pressed
  private final PositionVoltage mPositionControl = new PositionVoltage(0).withUpdateFreqHz(100);
  private final VoltageOut mVoltageOut = new VoltageOut(0).withUpdateFreqHz(100);

  /** Initializes the climb motor and configures closed-loop control. */
  public ClimbIOReal() {
    mMotor = new TalonFX(Constants.Climb.kMotorId, Constants.kCANBusName);

    // --- Motor Configuration ---
    var config = new TalonFXConfiguration();

    // Default to Brake mode, Inverted assumption (CW Positive = pulling up)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Position PID Gains
    config.Slot0.kP = Constants.Climb.kP;
    config.Slot0.kI = Constants.Climb.kI;
    config.Slot0.kD = Constants.Climb.kD;
    config.Slot0.kS = Constants.Climb.kS;
    config.Slot0.kV = Constants.Climb.kV;
    config.Slot0.kA = Constants.Climb.kA;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = Constants.Climb.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Climb.kSupplyCurrentLimitEnable;
    config.CurrentLimits.StatorCurrentLimit = Constants.Climb.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = Constants.Climb.kStatorCurrentLimitEnable;

    // Gear ratio (motor rotations -> mechanism rotations)
    // 125.0 motor rotations = 1 spool rotation
    config.Feedback.SensorToMechanismRatio = Constants.Climb.kGearRatio;

    boolean isConfigValid = frc.robot.util.Phoenix6Util.checkManeuver(
        () -> mMotor.getConfigurator().apply(config), "Climb Motor Config");

    // Zero the sensor on boot
    mMotor.setPosition(0.0);

    // --- Cache Status Signals ---
    mPosition = mMotor.getPosition();
    mVelocity = mMotor.getVelocity();
    mAppliedVolts = mMotor.getMotorVoltage();
    mCurrent = mMotor.getSupplyCurrent();

    mAllSignals = new BaseStatusSignal[] { mPosition, mVelocity, mAppliedVolts, mCurrent };

    // Configure 50Hz update rate and optimize bus
    mMotor.optimizeBusUtilization();
    Timer.delay(0.05);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);

    if (!isConfigValid) {
      System.err.println(
          "CRITICAL: Climb FAILED config. Will still poll its status but it may be unresponsive.");
    }
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    BaseStatusSignal.refreshAll(mAllSignals);

    // Position & Velocity in mechanism rotations
    inputs.positionRotations = mPosition.getValueAsDouble();
    inputs.velocityRotSec = mVelocity.getValueAsDouble();

    inputs.appliedVolts = mAppliedVolts.getValueAsDouble();
    inputs.currentAmps = mCurrent.getValueAsDouble();
    inputs.timestamp = Timer.getFPGATimestamp();

    // Connection status
    inputs.motorConnected = mPosition.getStatus().isOK();
  }

  @Override
  public void setPositionSetpoint(double positionRotations, double feedforwardVolts) {
    mMotor.setControl(
        mPositionControl.withPosition(positionRotations).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setVoltage(double volts) {
    mMotor.setControl(mVoltageOut.withOutput(volts));
  }

  @Override
  public void stop() {
    mMotor.setControl(mVoltageOut.withOutput(0.0));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    mMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
