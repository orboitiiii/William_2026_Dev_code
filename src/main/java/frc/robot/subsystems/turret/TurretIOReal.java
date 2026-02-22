package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Real hardware implementation of TurretIO using CTRE Phoenix 6.
 *
 * <p>This class manages CAN communication with the turret motor and encoder.
 *
 * <p><strong>Motor Configuration</strong>: Single KrakenX60 motor with CANcoder for absolute
 * position feedback.
 *
 * <p><strong>Control Strategy</strong>: Uses position control with velocity feedforward for smooth
 * tracking during shoot-on-move operations.
 *
 * @see TurretIO
 */
public class TurretIOReal implements TurretIO {
  private final TalonFX mMotor;

  // --- Cached Status Signals ---
  private final StatusSignal<Angle> mPosition;
  private final StatusSignal<AngularVelocity> mVelocity;
  private final StatusSignal<Voltage> mAppliedVolts;
  private final StatusSignal<Current> mCurrent;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Objects (reused to avoid allocation) ---
  private final PositionVoltage mPositionControl = new PositionVoltage(0);
  private final com.ctre.phoenix6.controls.MotionMagicVoltage mMotionMagicControl =
      new com.ctre.phoenix6.controls.MotionMagicVoltage(0);
  private final VoltageOut mVoltageOut = new VoltageOut(0);

  private boolean mIsUsingMotionMagic = false;

  // --- CRT Absolute Positioning CANcoders ---
  private final CANcoder mMotorEncoder;
  private final CANcoder mAuxEncoder;

  /** Initializes the turret motor and configures closed-loop control. */
  public TurretIOReal() {
    Timer.delay(0.1);
    mMotor = new TalonFX(Constants.Turret.kMotorId, Constants.kCANBusName);

    // --- Motor Configuration ---
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // CCW positive when viewed from above
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Position PID Gains
    config.Slot0.kP = Constants.Turret.kP;
    config.Slot0.kI = Constants.Turret.kI;
    config.Slot0.kD = Constants.Turret.kD;
    config.Slot0.kS = Constants.Turret.kS;
    config.Slot0.kV = Constants.Turret.kV;
    config.Slot0.kA = Constants.Turret.kA;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Turret.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Motion Magic Configuration
    config.MotionMagic.MotionMagicCruiseVelocity =
        Constants.Turret.kMaxVelocityRadPerSec / (2.0 * Math.PI);
    config.MotionMagic.MotionMagicAcceleration =
        Constants.Turret.kMaxAccelerationRadPerSecSq / (2.0 * Math.PI);

    // Gear ratio for accurate position reporting (motor rotations -> turret
    // radians)
    // SensorToMechanismRatio: motor rotations / mechanism rotations
    config.Feedback.SensorToMechanismRatio = Constants.Turret.kGearRatio;

    // Soft limits (in rotations)
    double minRotations = Constants.Turret.kMinAngleRads / (2 * Math.PI);
    double maxRotations = Constants.Turret.kMaxAngleRads / (2 * Math.PI);
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRotations;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    boolean isConfigValid =
        frc.robot.util.Phoenix6Util.checkManeuver(
            () -> mMotor.getConfigurator().apply(config), "Turret Motor Config");

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
          "CRITICAL: Turret FAILED config - Excluding from synchronous updates to prevent Loop Overrun.");
    }

    // --- CANcoder Init (for CRT absolute positioning) ---
    mMotorEncoder = new CANcoder(Constants.Turret.kMotorEncoderId, Constants.kCANBusName);
    mAuxEncoder = new CANcoder(Constants.Turret.kAuxEncoderId, Constants.kCANBusName);

    // Motor Shaft CANcoder Configuration
    var motorEncoderConfig = new CANcoderConfiguration();
    motorEncoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.kMotorEncoderOffsetRotations;
    // Positive direction matches motor (Counter-Clockwise Positive from motor side)
    motorEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    frc.robot.util.Phoenix6Util.checkManeuver(
        () -> mMotorEncoder.getConfigurator().apply(motorEncoderConfig), "Motor CANcoder Config");

    // Auxiliary Shaft CANcoder Configuration
    var auxEncoderConfig = new CANcoderConfiguration();
    auxEncoderConfig.MagnetSensor.MagnetOffset = Constants.Turret.kAuxEncoderOffsetRotations;
    auxEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    frc.robot.util.Phoenix6Util.checkManeuver(
        () -> mAuxEncoder.getConfigurator().apply(auxEncoderConfig), "Aux CANcoder Config");

    Timer.delay(0.05); // Wait for CANcoder configs
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(mAllSignals);

    // Position in rotations -> radians
    inputs.positionRads = mPosition.getValueAsDouble() * 2 * Math.PI;
    inputs.velocityRadsPerSec = mVelocity.getValueAsDouble() * 2 * Math.PI;
    inputs.appliedVolts = mAppliedVolts.getValueAsDouble();
    inputs.currentAmps = mCurrent.getValueAsDouble();
    inputs.timestamp = Timer.getFPGATimestamp();

    // Connection status (Passive Check)
    inputs.motorConnected = mPosition.getStatus().isOK();
    inputs.encoder1Connected = mMotorEncoder.getAbsolutePosition().getStatus().isOK();
    inputs.encoder2Connected = mAuxEncoder.getAbsolutePosition().getStatus().isOK();

    inputs.motorEncoderAbsPosRotations = mMotorEncoder.getAbsolutePosition().getValueAsDouble();
    inputs.auxEncoderAbsPosRotations = mAuxEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(
      double positionRads, double velocityRadsPerSec, double feedforwardVolts) {
    // Convert radians to rotations for Phoenix 6
    double targetRotations = positionRads / (2.0 * Math.PI);
    double currentRotations = mPosition.getValueAsDouble();
    double currentVelocityRotPerSec = mVelocity.getValueAsDouble();

    // Calculate kS Feedforward (Static friction compensation)
    double kS_FF = 0.0;
    if (Math.abs(currentVelocityRotPerSec) > 1e-3) {
      kS_FF = Constants.Turret.kS * Math.signum(currentVelocityRotPerSec);
    }
    double totalFeedforward = kS_FF + feedforwardVolts;

    // Calculate absolute position error in rotations, properly handling phase
    // wraparound
    // Convert to radians to use WPILib angleModulus for [-pi, pi] shortest path,
    // then back to rotations
    double errorRads =
        edu.wpi.first.math.MathUtil.angleModulus(
            (targetRotations - currentRotations) * 2.0 * Math.PI);
    double errorRotations = Math.abs(errorRads / (2.0 * Math.PI));

    // Hysteresis Band (10.0 degrees upper, 5.0 degrees lower)
    double upperThresholdRot = 10.0 / 360.0;
    double lowerThresholdRot = 5.0 / 360.0;

    if (errorRotations > upperThresholdRot && !mIsUsingMotionMagic) {
      mIsUsingMotionMagic = true;
    } else if (errorRotations < lowerThresholdRot && mIsUsingMotionMagic) {
      mIsUsingMotionMagic = false;
    }

    if (mIsUsingMotionMagic) {
      // State 1: Macro Movement (Smooth Trapezoidal Trajectory)
      mMotor.setControl(
          mMotionMagicControl.withPosition(targetRotations).withFeedForward(totalFeedforward));
    } else {
      // State 2: Micro Tracking (Responsive PID + Raw Voltage Feedforward for
      // shoot-on-move)
      mMotor.setControl(
          mPositionControl
              .withPosition(targetRotations)
              .withVelocity(0.0)
              .withFeedForward(totalFeedforward));
    }
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
  public void setBrakeMode(boolean brake) {
    var config = new com.ctre.phoenix6.configs.MotorOutputConfigs();
    config.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    mMotor.getConfigurator().apply(config);
  }

  @Override
  public void setPIDGains(double kP, double kD) {
    var config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    mMotor.getConfigurator().apply(config);
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    // CRITICAL: Do NOT create a new TalonFXConfiguration() and apply it,
    // as that overwrites ALL configs (PID, CurrentLimits, etc.) with defaults.

    // Instead, modify only the SoftwareLimitSwitchConfigs
    var limitConfig = new com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs();
    mMotor.getConfigurator().refresh(limitConfig); // Read current values

    limitConfig.ReverseSoftLimitEnable = enabled;
    limitConfig.ForwardSoftLimitEnable = enabled;

    // If enabling, ensure thresholds are set correctly (though they should persist
    // if we refreshed)
    if (enabled) {
      double minRotations = Constants.Turret.kMinAngleRads / (2 * Math.PI);
      double maxRotations = Constants.Turret.kMaxAngleRads / (2 * Math.PI);
      limitConfig.ReverseSoftLimitThreshold = minRotations;
      limitConfig.ForwardSoftLimitThreshold = maxRotations;
    }

    mMotor.getConfigurator().apply(limitConfig);
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

  @Override
  public double initializeAbsolutePosition() {
    // Read absolute positions from both CANcoders
    var motorAbsPos = mMotorEncoder.getAbsolutePosition();
    var auxAbsPos = mAuxEncoder.getAbsolutePosition();

    // Wait up to 250ms for fresh, valid signals, guaranteeing config offset has
    // applied
    BaseStatusSignal.waitForAll(0.25, motorAbsPos, auxAbsPos);

    if (!motorAbsPos.getStatus().isOK()) {
      System.err.println(
          "CRT INIT FAIL: Motor CANcoder (ID "
              + Constants.Turret.kMotorEncoderId
              + ") not responding.");
      return Double.NaN;
    }
    if (!auxAbsPos.getStatus().isOK()) {
      System.err.println(
          "CRT INIT FAIL: Aux CANcoder (ID "
              + Constants.Turret.kAuxEncoderId
              + ") not responding.");
      return Double.NaN;
    }

    // CANcoder returns rotations, range determined by
    // AbsoluteSensorDiscontinuityPoint.
    // Default is [-0.5, 0.5), we need [0, 1).
    double enc1Raw = motorAbsPos.getValueAsDouble();
    double enc2Raw = auxAbsPos.getValueAsDouble();

    // Normalize to [0, 1)
    double enc1Normalized = enc1Raw - Math.floor(enc1Raw);
    double enc2Normalized = enc2Raw - Math.floor(enc2Raw);

    System.out.println(
        "CRT: motorEncoder="
            + enc1Raw
            + " (norm="
            + enc1Normalized
            + "), auxEncoder="
            + enc2Raw
            + " (norm="
            + enc2Normalized
            + ")");

    // CRT Calculation
    double angleRads =
        TurretCRTResolver.resolveAngleRads(
            enc1Normalized,
            enc2Normalized,
            Constants.Turret.kMotorGearTeeth,
            Constants.Turret.kAuxGearTeeth,
            Constants.Turret.kTurretRingTeeth);

    if (Double.isNaN(angleRads)) {
      System.err.println("CRT INIT FAIL: CRT resolver returned NaN.");
      return Double.NaN;
    }

    // Write angle to motor encoder position.
    // Phoenix 6 setPosition uses mechanism rotations.
    // SensorToMechanismRatio is already set, so we pass turret rotations.
    double turretRotations = angleRads / (2.0 * Math.PI);
    mMotor.setPosition(turretRotations);

    System.out.println(
        "CRT INIT OK: turret angle = "
            + Math.toDegrees(angleRads)
            + "° ("
            + turretRotations
            + " rot)");

    return angleRads;
  }
}
