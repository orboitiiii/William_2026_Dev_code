package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Real hardware implementation of IntakePivotIO using CTRE Phoenix 6.
 *
 * <p>This class manages CAN communication with the dual-motor intake pivot mechanism for the
 * four-bar linkage.
 *
 * <p><strong>Motor Configuration</strong>:
 *
 * <ul>
 *   <li>Right Motor (ID 30): Leader motor, provides position reference.
 *   <li>Left Motor (ID 31): Follower motor, inverted to produce opposing torque.
 * </ul>
 *
 * <p><strong>Control Strategy</strong>: Uses MotionMagic position control for smooth, trapezoid
 * velocity profiled motion. Both motors are mechanically coupled via the four-bar linkage, so the
 * left motor follows the right motor with opposing inversion.
 *
 * <p><strong>First Principles</strong>: In a four-bar linkage driven by two motors on opposite
 * sides, both motors must apply torque in the same "effective" direction relative to the linkage.
 * Since they are mechanically mirrored, the left motor must spin in the opposite direction to the
 * right motor for synchronized motion.
 *
 * @see IntakePivotIO
 */
public class IntakePivotIOReal implements IntakePivotIO {
  private final TalonFX mRightMotor; // Leader
  private final TalonFX mLeftMotor; // Follower

  // --- Cached Status Signals (Right Motor) ---
  private final StatusSignal<Angle> mPosition;
  private final StatusSignal<AngularVelocity> mVelocity;
  private final StatusSignal<Voltage> mRightAppliedVolts;
  private final StatusSignal<Current> mRightCurrent;

  // --- Cached Status Signals (Left Motor) ---
  private final StatusSignal<Voltage> mLeftAppliedVolts;
  private final StatusSignal<Current> mLeftCurrent;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Objects (reused to avoid allocation) ---
  private final MotionMagicVoltage mMotionMagic = new MotionMagicVoltage(0);
  private final VoltageOut mVoltageOut = new VoltageOut(0);
  private final Follower mFollower;

  /**
   * Initializes the intake pivot motors and configures leader-follower relationship.
   *
   * <p>The right motor (ID 30) acts as the leader with closed-loop position control. The left motor
   * (ID 31) follows in opposing direction for synchronized four-bar linkage operation.
   */
  public IntakePivotIOReal() {
    mRightMotor = new TalonFX(Constants.IntakePivot.kRightMotorId);
    mLeftMotor = new TalonFX(Constants.IntakePivot.kLeftMotorId);

    // --- Right Motor (Leader) Configuration ---
    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Right motor: CounterClockwise_Positive (standard convention)
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Position PID Gains
    rightConfig.Slot0.kP = Constants.IntakePivot.kP;
    rightConfig.Slot0.kI = Constants.IntakePivot.kI;
    rightConfig.Slot0.kD = Constants.IntakePivot.kD;
    rightConfig.Slot0.kS = Constants.IntakePivot.kS;
    rightConfig.Slot0.kV = Constants.IntakePivot.kV;
    rightConfig.Slot0.kG = Constants.IntakePivot.kG;

    // Motion Magic configuration for smooth profiled motion
    rightConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakePivot.kCruiseVelocity;
    rightConfig.MotionMagic.MotionMagicAcceleration = Constants.IntakePivot.kAcceleration;

    // Current limits
    rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivot.kSupplyCurrentLimit;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.IntakePivot.kSupplyCurrentLimitEnable;
    rightConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakePivot.kStatorCurrentLimit;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.IntakePivot.kStatorCurrentLimitEnable;

    // Gear ratio for accurate position reporting
    rightConfig.Feedback.SensorToMechanismRatio = Constants.IntakePivot.kGearRatio;

    mRightMotor.getConfigurator().apply(rightConfig);

    // --- Left Motor (Follower) Configuration ---
    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Invert left motor?
    // 254/1678 STANDARD: Keep config "Standard" where possible.
    // Left Motor = Standard (CCW+).
    // The "Opposed" Follower update will handle the voltage inversion.
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Current limits (same as leader)
    leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivot.kSupplyCurrentLimit;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.IntakePivot.kSupplyCurrentLimitEnable;
    leftConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakePivot.kStatorCurrentLimit;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.IntakePivot.kStatorCurrentLimitEnable;

    mLeftMotor.getConfigurator().apply(leftConfig);

    // --- Follower Control: Left follows Right, Opposed = true ---
    // 254/1678 STANDARD: Left motor is configured as Standard (CCW+).
    // Opposed alignment ensures +V on Leader becomes -V on Follower.
    // Standard Motor with -V -> Spins CW.
    // Result: Mirrored Motion (CCW/CW) with Opposed Voltage (+V/-V).
    mFollower = new Follower(Constants.IntakePivot.kRightMotorId, MotorAlignmentValue.Opposed);
    mLeftMotor.setControl(mFollower);

    // --- Cache Status Signals ---
    mPosition = mRightMotor.getPosition();
    mVelocity = mRightMotor.getVelocity();
    mRightAppliedVolts = mRightMotor.getMotorVoltage();
    mRightCurrent = mRightMotor.getSupplyCurrent();
    mLeftAppliedVolts = mLeftMotor.getMotorVoltage();
    mLeftCurrent = mLeftMotor.getSupplyCurrent();

    mAllSignals =
        new BaseStatusSignal[] {
          mPosition, mVelocity, mRightAppliedVolts, mRightCurrent, mLeftAppliedVolts, mLeftCurrent
        };

    // Configure 50Hz update rate
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(mAllSignals);

    // Position in rotations -> Rotation2d
    inputs.position = Rotation2d.fromRotations(mPosition.getValueAsDouble());
    inputs.velocityRotationsPerSec = mVelocity.getValueAsDouble();
    inputs.rightAppliedVolts = mRightAppliedVolts.getValueAsDouble();
    inputs.leftAppliedVolts = mLeftAppliedVolts.getValueAsDouble();
    inputs.rightCurrentAmps = mRightCurrent.getValueAsDouble();
    inputs.leftCurrentAmps = mLeftCurrent.getValueAsDouble();

    // Connection status (Passive Check)
    inputs.rightMotorConnected = mPosition.getStatus().isOK();
    inputs.leftMotorConnected = mLeftAppliedVolts.getStatus().isOK();
  }

  @Override
  public void setPosition(Rotation2d angle) {
    // Command position to right motor (leader)
    // Left motor follows automatically via Follower control mode
    mRightMotor.setControl(mMotionMagic.withPosition(angle.getRotations()));
  }

  @Override
  public void setVoltage(double volts) {
    // Command voltage to right motor (leader)
    // Left motor is configured as a Follower (Opposed), so it will automatically
    // apply the same voltage magnitude but in the opposite direction.
    // CRITICAL: Do NOT manually command mLeftMotor here, as that would exit
    // Follower mode!
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
  public void resetPosition(Rotation2d angle) {
    mRightMotor.setPosition(angle.getRotations());
    // Since Left is Standard (CCW+) but moving CW (physically "Up"),
    // its native position accumulates negatively. We must set negative
    // to match the physical absolute position.
    mLeftMotor.setPosition(-angle.getRotations());
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
