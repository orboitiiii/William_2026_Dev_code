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
  private final VoltageOut mVoltageControl = new VoltageOut(0);

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
    config.Slot0.kS = Constants.Turret.kS; // 恢復硬體級 kS，馬達內部處理更平滑
    config.Slot0.kV = Constants.Turret.kV;
    config.Slot0.kA = Constants.Turret.kA;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.kSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = Constants.Turret.kStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // Removed Motion Magic config: We utilize RIO-side TrapezoidProfile with
    // PositionVoltage,
    // so internal profile generator bounds are dead code and may cause confusion.

    // Gear ratio for accurate position reporting (motor rotations -> turret
    // radians)
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

    mAllSignals = new BaseStatusSignal[] {mPosition, mVelocity, mAppliedVolts, mCurrent};

    // Configure 50Hz update rate
    // Minimize CAN bus usage by disabling unused status frames
    mMotor.optimizeBusUtilization();

    // PAUSE: Allow CAN buffer to drain
    Timer.delay(0.05);

    // 降低砲塔回報頻率至 50Hz (20ms)
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);

    if (!isConfigValid) {
      System.err.println(
          "CRITICAL: Turret FAILED config. Will still poll its status but it may be unresponsive.");
    }

    // --- CANcoder Init (for CRT absolute positioning) ---
    mMotorEncoder = new CANcoder(Constants.Turret.kMotorEncoderId, Constants.kCANBusName);
    mAuxEncoder = new CANcoder(Constants.Turret.kAuxEncoderId, Constants.kCANBusName);

    // Motor Shaft CANcoder Configuration
    var motorEncoderConfig = new CANcoderConfiguration();
    // 移除硬體的 MagnetOffset，讓 CRT 得以使用最純粹的原始分數進行同步，避免在 0度附近獨自溢位
    motorEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
    // Positive direction matches motor (Counter-Clockwise Positive from motor side)
    motorEncoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    frc.robot.util.Phoenix6Util.checkManeuver(
        () -> mMotorEncoder.getConfigurator().apply(motorEncoderConfig), "Motor CANcoder Config");

    // Auxiliary Shaft CANcoder Configuration
    var auxEncoderConfig = new CANcoderConfiguration();
    // 移除硬體的 MagnetOffset
    auxEncoderConfig.MagnetSensor.MagnetOffset = 0.0;
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

    inputs.motorEncoderRawRotations = mMotorEncoder.getPosition().getValueAsDouble();
    inputs.auxEncoderRawRotations = mAuxEncoder.getPosition().getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(
      double positionRads, double velocityRadsPerSec, double feedforwardVolts) {

    double targetRotations = positionRads / (2.0 * Math.PI);
    double targetVelocityRotPerSec = velocityRadsPerSec / (2.0 * Math.PI);

    mMotor.setControl(
        mPositionControl
            .withPosition(targetRotations)
            .withVelocity(targetVelocityRotPerSec)
            .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setVoltage(double volts) {
    mMotor.setControl(mVoltageControl.withOutput(volts));
  }

  @Override
  public void stop() {
    mMotor.setControl(mVoltageControl.withOutput(0));
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

    double enc1Raw = motorAbsPos.getValueAsDouble();
    double enc2Raw = auxAbsPos.getValueAsDouble();

    System.out.println("CRT: motorEncoderRaw=" + enc1Raw + ", auxEncoderRaw=" + enc2Raw);

    // CRT (Vernier) Calculation
    double crtAngleRads =
        TurretCRTResolver.resolveTurretAngleRads(
            enc1Raw,
            enc2Raw,
            Constants.Turret.kMotorGearTeeth,
            Constants.Turret.kAuxGearTeeth,
            Constants.Turret.kTurretRingTeeth,
            Constants.Turret.kMotorEncoderOffsetRotations,
            Constants.Turret.kAuxEncoderOffsetRotations);

    if (Double.isNaN(crtAngleRads)) {
      System.err.println("CRT INIT FAIL: CRT resolver returned NaN.");
      return Double.NaN;
    }

    // ── 物理範圍映射 ──
    // CRT 輸出可能在任意一個等效圈數，我們將其平移到最接近砲塔物理中心範圍的角度
    double angleRads = crtAngleRads;
    double rangeCenter = (Constants.Turret.kMinAngleRads + Constants.Turret.kMaxAngleRads) / 2.0;

    // 將 CRT 角度平移到最接近 rangeCenter 的等價角 (-PI 到 +PI)
    angleRads += Math.round((rangeCenter - angleRads) / (2.0 * Math.PI)) * 2.0 * Math.PI;

    // 安全鉗位：如果是機械限制以外微小的浮點誤差
    angleRads =
        Math.max(
            Constants.Turret.kMinAngleRads, Math.min(angleRads, Constants.Turret.kMaxAngleRads));

    // Write angle to motor encoder position.
    // Phoenix 6 setPosition uses mechanism rotations.
    // SensorToMechanismRatio is already set (4 * 87/19), so we pass mechanism
    // (turret) rotations.
    double turretRotations = angleRads / (2.0 * Math.PI);
    mMotor.setPosition(turretRotations);

    System.out.println(
        "CRT INIT OK: crtRaw="
            + Math.toDegrees(crtAngleRads)
            + "° → mappedAngle="
            + Math.toDegrees(angleRads)
            + "° ("
            + turretRotations
            + " rot)");

    return angleRads;
  }
}
