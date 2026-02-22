package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class IntakePivotIOReal implements IntakePivotIO {
  private final TalonFX mRightMotor; // Leader (Right)
  private final TalonFX mLeftMotor; // Independent (Left)

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
  private final MotionMagicVoltage mMotionMagic = new MotionMagicVoltage(0).withEnableFOC(true);
  private final VoltageOut mVoltageOut = new VoltageOut(0).withEnableFOC(true);

  // mCurrentCruiseVel and mCurrentAccel removed as dynamic config is disabled

  /**
   * Initializes the intake pivot motors in independent control mode.
   *
   * <p>Both motors are configured with identical parameters (PID, MotionMagic, etc.) but inverted
   * physically (Right=CW, Left=CCW) to move the mechanism together.
   */
  public IntakePivotIOReal() {
    mRightMotor = new TalonFX(Constants.IntakePivot.kRightMotorId, Constants.kCANBusName);
    mLeftMotor = new TalonFX(Constants.IntakePivot.kLeftMotorId, Constants.kCANBusName);

    // --- Right Motor Configuration ---
    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Right motor: Clockwise_Positive (Down = Positive Angle)
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // --- Slot0 (UP / Fast) ---
    rightConfig.Slot0.kP = Constants.IntakePivot.kP;
    rightConfig.Slot0.kI = Constants.IntakePivot.kI;
    rightConfig.Slot0.kD = Constants.IntakePivot.kD;
    rightConfig.Slot0.kS = Constants.IntakePivot.kS;
    rightConfig.Slot0.kV = Constants.IntakePivot.kV;
    rightConfig.Slot0.kA = Constants.IntakePivot.kA;
    rightConfig.Slot0.kG = 0.0; // Disabled: Using Software Lookup Table
    rightConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // --- Slot1 (DOWN / Slow) ---
    rightConfig.Slot1.kP = Constants.IntakePivot.kP; // Same PID gains for now
    rightConfig.Slot1.kI = Constants.IntakePivot.kI;
    rightConfig.Slot1.kD = Constants.IntakePivot.kD;
    rightConfig.Slot1.kS = Constants.IntakePivot.kS;
    rightConfig.Slot1.kV = Constants.IntakePivot.kV;
    rightConfig.Slot1.kA = Constants.IntakePivot.kA;
    rightConfig.Slot1.kG = 0.0; // Using Lookup Table
    rightConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    // Motion Magic configuration (Global - optimized for Safety/Down)
    rightConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakePivot.kCruiseVelocityDown;
    rightConfig.MotionMagic.MotionMagicAcceleration = Constants.IntakePivot.kAccelerationDown;

    // Current limits
    rightConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivot.kSupplyCurrentLimit;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.IntakePivot.kSupplyCurrentLimitEnable;
    rightConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakePivot.kStatorCurrentLimit;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.IntakePivot.kStatorCurrentLimitEnable;

    // Gear ratio
    rightConfig.Feedback.SensorToMechanismRatio = Constants.IntakePivot.kGearRatio;

    // Soft Limits (Reverse limit offset by -4.0 to prevent trapped MotionMagic
    // states)
    rightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Rotation2d.fromDegrees(Constants.IntakePivot.kMinAngle - 4.0).getRotations();
    rightConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Rotation2d.fromDegrees(Constants.IntakePivot.kMaxAngle).getRotations();
    rightConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    // Audio Configuration (Silent Operation)
    rightConfig.Audio.BeepOnBoot = false;
    rightConfig.Audio.BeepOnConfig = false;
    rightConfig.Audio.AllowMusicDurDisable = true;

    // Apply Right Config
    boolean isRightValid =
        frc.robot.util.Phoenix6Util.checkManeuver(
            () -> mRightMotor.getConfigurator().apply(rightConfig),
            "IntakePivot Right Motor Config");

    // --- Initial Position (Right) ---
    mRightMotor.setPosition(Constants.IntakePivot.kMinAngle);
    Timer.delay(0.1);

    // --- Left Motor Configuration ---
    // Clone right config to ensure identical parameters
    var leftConfig = new TalonFXConfiguration();
    // Manually copy relevant fields or just configure fresh to be safe and explicit
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // INVERSION: Left is physically opposed, so it must be
    // CounterClockwise_Positive
    // to match Right's Clockwise_Positive movement.
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Copy Gains for Slot0
    leftConfig.Slot0.kP = Constants.IntakePivot.kP;
    leftConfig.Slot0.kI = Constants.IntakePivot.kI;
    leftConfig.Slot0.kD = Constants.IntakePivot.kD;
    leftConfig.Slot0.kS = Constants.IntakePivot.kS;
    leftConfig.Slot0.kV = Constants.IntakePivot.kV;
    leftConfig.Slot0.kA = Constants.IntakePivot.kA;
    leftConfig.Slot0.kG = 0.0;
    leftConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // Copy Gains for Slot1
    leftConfig.Slot1.kP = Constants.IntakePivot.kP;
    leftConfig.Slot1.kI = Constants.IntakePivot.kI;
    leftConfig.Slot1.kD = Constants.IntakePivot.kD;
    leftConfig.Slot1.kS = Constants.IntakePivot.kS;
    leftConfig.Slot1.kV = Constants.IntakePivot.kV;
    leftConfig.Slot1.kA = Constants.IntakePivot.kA;
    leftConfig.Slot1.kG = 0.0;
    leftConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;

    // Copy Motion Magic (initial, will be updated dynamically)
    leftConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakePivot.kCruiseVelocityDown;
    leftConfig.MotionMagic.MotionMagicAcceleration = Constants.IntakePivot.kAccelerationDown;

    // Copy Current Limits
    leftConfig.CurrentLimits.SupplyCurrentLimit = Constants.IntakePivot.kSupplyCurrentLimit;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.IntakePivot.kSupplyCurrentLimitEnable;
    leftConfig.CurrentLimits.StatorCurrentLimit = Constants.IntakePivot.kStatorCurrentLimit;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable =
        Constants.IntakePivot.kStatorCurrentLimitEnable;

    // Copy Gear Ratio
    leftConfig.Feedback.SensorToMechanismRatio = Constants.IntakePivot.kGearRatio;

    // Copy Soft Limits
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        rightConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
    leftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        rightConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold;
    leftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    // Audio Configuration
    leftConfig.Audio.BeepOnBoot = false;
    leftConfig.Audio.BeepOnConfig = false;
    leftConfig.Audio.AllowMusicDurDisable = true;

    // Apply Left Config
    boolean isLeftValid =
        frc.robot.util.Phoenix6Util.checkManeuver(
            () -> mLeftMotor.getConfigurator().apply(leftConfig), "IntakePivot Left Motor Config");

    // --- Initial Position (Left) ---
    mLeftMotor.setPosition(Constants.IntakePivot.kMinAngle);
    Timer.delay(0.1);

    // --- Configure Updates ---
    // Cache signals
    mPosition = mRightMotor.getPosition();
    mVelocity = mRightMotor.getVelocity();
    mRightAppliedVolts = mRightMotor.getMotorVoltage();
    mRightCurrent = mRightMotor.getSupplyCurrent();
    mLeftAppliedVolts = mLeftMotor.getMotorVoltage();
    mLeftCurrent = mLeftMotor.getSupplyCurrent();

    if (isRightValid && isLeftValid) {
      mAllSignals =
          new BaseStatusSignal[] {
            mPosition, mVelocity, mRightAppliedVolts, mRightCurrent, mLeftAppliedVolts, mLeftCurrent
          };

      // Optimize bus
      mRightMotor.optimizeBusUtilization();
      mLeftMotor.optimizeBusUtilization();
      Timer.delay(0.05);

      // Set Update Frequency
      BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAllSignals);
    } else {
      mAllSignals = new BaseStatusSignal[0];
      System.err.println(
          "CRITICAL: IntakePivot FAILED config - Excluding from synchronous updates.");
    }
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
  public void setPosition(
      Rotation2d angle, double feedforwardVolts, double velocity, double acceleration) {
    // Independent Control: Send same command to both motors
    // Both motors are configured to move in the physical "positive" direction

    // Dynamic configuration removed (Option A).
    // Velocity and Acceleration arguments are ignored in favor of global static
    // config.

    mMotionMagic.Position = angle.getRotations();
    mMotionMagic.FeedForward = feedforwardVolts;
    // Slot 0 is used by default

    mRightMotor.setControl(mMotionMagic);
    mLeftMotor.setControl(mMotionMagic);
  }

  @Override
  public void setVoltage(double volts) {
    // Independent Control: Send same voltage to both motors
    var request = mVoltageOut.withOutput(volts);
    mRightMotor.setControl(request);
    mLeftMotor.setControl(request);
  }

  @Override
  public void stop() {
    var request = mVoltageOut.withOutput(0);
    mRightMotor.setControl(request);
    mLeftMotor.setControl(request);
  }

  @Override
  public void resetPosition(Rotation2d angle) {
    // Reset both encoders to ensure they stay synced
    double rotations = angle.getRotations();
    mRightMotor.setPosition(rotations);
    mLeftMotor.setPosition(rotations);
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
