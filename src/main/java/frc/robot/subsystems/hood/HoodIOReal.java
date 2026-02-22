package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
 * Real hardware implementation of HoodIO using CTRE Phoenix 6.
 *
 * <p>This class manages CAN communication with the hood motor.
 *
 * <p><strong>Motor Configuration</strong>: Single motor (e.g., Falcon500 or KrakenX60) driving a
 * linear actuator or worm gear mechanism for hood angle adjustment.
 *
 * <p><strong>Control Strategy</strong>: Uses position control with velocity feedforward for smooth
 * tracking.
 *
 * @see HoodIO
 */
public class HoodIOReal implements HoodIO {
  private final TalonFX mMotor;

  // --- Cached Status Signals ---
  private final StatusSignal<Angle> mPosition;
  private final StatusSignal<AngularVelocity> mVelocity;
  private final StatusSignal<Voltage> mAppliedVolts;
  private final StatusSignal<Current> mCurrent;

  private HoodOutputMode mLastMode = null;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Objects (reused to avoid allocation) ---
  private final PositionVoltage mPositionControl = new PositionVoltage(0);
  private final MotionMagicVoltage mMotionMagicControl = new MotionMagicVoltage(0);
  private final VoltageOut mVoltageOut = new VoltageOut(0);

  /** Initializes the hood motor and configures closed-loop control. */
  public HoodIOReal() {
    Timer.delay(0.1);
    mMotor = new TalonFX(Constants.Hood.kMotorId, Constants.kCANBusName);

    // --- Motor Configuration ---
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Position PID Gains
    config.Slot0.kP = Constants.Hood.kP;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = Constants.Hood.kD;
    config.Slot0.kS = 0.0; // Applied manually in Hood.java arbFF
    config.Slot0.kV = Constants.Hood.kV;
    config.Slot0.kA = Constants.Hood.kA;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Hood.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Gear ratio (motor rotations -> hood mechanism rotations)
    // Example: 42.5 motor rotations = 1 mechanism rotation (360 deg)
    config.Feedback.SensorToMechanismRatio = Constants.Hood.kGearRatio;

    // Soft limits (in Mechanism Rotations)
    // 1 Mechanism Rotation = 360 Degrees
    double minRotations = Constants.Hood.kMinAngleRads / (2 * Math.PI);
    double maxRotations = Constants.Hood.kMaxAngleRads / (2 * Math.PI);

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRotations;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    // MotionMagic configuration (Mechanism units: rotations, rotations/s,
    // rotations/s²)
    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.kMotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = Constants.Hood.kMotionMagicAcceleration;

    boolean isConfigValid =
        frc.robot.util.Phoenix6Util.checkManeuver(
            () -> mMotor.getConfigurator().apply(config), "Hood Motor Config");

    // --- Set Initial Position ---
    // Physical hardware starts at kInitialAngleDegrees (65.0)
    // setPosition() uses MECHANISM units when SensorToMechanismRatio is configured.
    // Units: Mechanism Rotations = Degrees / 360.0
    mMotor.setPosition(Constants.Hood.kInitialAngleDegrees / 360.0);

    // --- Cache Status Signals ---
    mPosition = mMotor.getPosition();
    mVelocity = mMotor.getVelocity();
    mAppliedVolts = mMotor.getMotorVoltage();
    mCurrent = mMotor.getSupplyCurrent();

    if (isConfigValid) {
      mAllSignals = new BaseStatusSignal[] {mPosition, mVelocity, mAppliedVolts, mCurrent};

      // Configure 50Hz update rate
      // Minimize CAN bus usage by disabling unused status frames
      mMotor.optimizeBusUtilization();

      // PAUSE: Allow CAN buffer to drain
      Timer.delay(0.05);

      // Configure 50Hz update rate
      BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);
    } else {
      mAllSignals = new BaseStatusSignal[0];
      System.err.println(
          "CRITICAL: Hood FAILED config - Excluding from synchronous updates to prevent Loop Overrun.");
    }
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(mAllSignals);

    // Position in rotations -> radians
    inputs.positionRads = mPosition.getValueAsDouble() * 2 * Math.PI;
    inputs.velocityRadsPerSec = mVelocity.getValueAsDouble() * 2 * Math.PI;
    inputs.appliedVolts = mAppliedVolts.getValueAsDouble();
    inputs.currentAmps = mCurrent.getValueAsDouble();
    inputs.timestamp = Timer.getFPGATimestamp();

    // Connection status (Passive Check)
    inputs.motorConnected = mPosition.getStatus().isOK();
  }

  @Override
  public void setPositionSetpoint(
      double positionRads, double velocityRadsPerSec, double feedforwardVolts) {
    // Convert radians to rotations for Phoenix 6
    double positionRotations = positionRads / (2 * Math.PI);
    double velocityRotPerSec = velocityRadsPerSec / (2 * Math.PI);

    mMotor.setControl(
        mPositionControl
            .withPosition(positionRotations)
            .withVelocity(velocityRotPerSec)
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setVoltage(double volts) {
    mMotor.setControl(mVoltageOut.withOutput(volts));
  }

  @Override
  public void setMotionMagicSetpoint(double positionRads) {
    // Convert radians to rotations for Phoenix 6
    double positionRotations = positionRads / (2 * Math.PI);
    mMotor.setControl(mMotionMagicControl.withPosition(positionRotations));
  }

  @Override
  public void stop() {
    mMotor.setControl(mVoltageOut.withOutput(0));
  }

  @Override
  public void setOutputMode(HoodOutputMode mode) {
    if (mode == mLastMode) {
      return;
    }
    mLastMode = mode;

    // Use setNeutralMode directly instead of applying full config
    switch (mode) {
      case BRAKE:
        mMotor.setNeutralMode(NeutralModeValue.Brake);
        break;
      case COAST:
        mMotor.setNeutralMode(NeutralModeValue.Coast);
        break;
      case CLOSED_LOOP:
        // Neutral mode doesn't affect closed-loop operation
        // But we might want to default to brake?
        // For now, do nothing as intended
        break;
    }
  }

  @Override
  public void setPIDGains(double kP, double kD) {
    // Only update the Slot0 config, not the entire config
    var slotConfig = new com.ctre.phoenix6.configs.Slot0Configs();
    slotConfig.kP = kP;
    slotConfig.kD = kD;
    mMotor.getConfigurator().apply(slotConfig);
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    // Only update soft limit enable flags
    var softLimitConfig = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
    softLimitConfig.ForwardSoftLimitEnable = enabled;
    softLimitConfig.ReverseSoftLimitEnable = enabled;
    mMotor.getConfigurator().apply(softLimitConfig);
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
