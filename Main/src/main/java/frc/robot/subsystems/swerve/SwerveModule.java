package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
  private final TalonFX mDriveMotor;
  private final TalonFX mSteerMotor;
  private final CANcoder mCANCoder;

  // Controls
  private final DutyCycleOut mDutyCycleOut = new DutyCycleOut(0);
  private final VelocityVoltage mVelocityOut = new VelocityVoltage(0);
  private final PositionVoltage mPositionOut = new PositionVoltage(0);

  private final String mName;
  private final Translation2d mLocation;
  private final double mCANCoderOffset;

  public SwerveModule(
      String name,
      int driveId,
      int steerId,
      int cancoderId,
      double offset,
      Translation2d location) {
    mName = name;
    mCANCoderOffset = offset;
    mLocation = location;

    mDriveMotor = new TalonFX(driveId, Constants.Swerve.kCanivoreBusName);
    mSteerMotor = new TalonFX(steerId, Constants.Swerve.kCanivoreBusName);
    mCANCoder = new CANcoder(cancoderId, Constants.Swerve.kCanivoreBusName);

    configureDevices();
  }

  private void configureDevices() {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0.kP = 0.1; // TODO: Tune
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = 0.0;
    driveConfig.Slot0.kV = 0.12; // TODO: Tune
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kDriveGearRatio;

    var steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.Slot0.kP = 2.0; // TODO: Tune
    steerConfig.Slot0.kI = 0.0;
    steerConfig.Slot0.kD = 0.0;
    steerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.kSteerGearRatio;

    mDriveMotor.getConfigurator().apply(driveConfig);
    mSteerMotor.getConfigurator().apply(steerConfig);
  }

  public synchronized void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
    // Optimize first
    state.optimize(getSteerAngle());

    // 1690 Cosine Scaling: Speed_out = Speed_req * cos(Error)
    Rotation2d currentAngle = getSteerAngle();
    Rotation2d error = state.angle.minus(currentAngle);
    double cosineScalar = error.getCos();

    // Scale speed. If cosine is negative (unlikely after optimize, but possible during transient),
    // cap at 0?
    // Optimize ensures error is [-90, 90], so cos is [0, 1].
    double scaledSpeed = state.speedMetersPerSecond * Math.abs(cosineScalar);

    // Apply scaled speed
    if (isOpenLoop) {
      double percentOutput = scaledSpeed / Constants.Swerve.kMaxDriveVelocity;
      mDriveMotor.setControl(mDutyCycleOut.withOutput(percentOutput));
    } else {
      // Speed (m/s) -> Device Rotations/s
      // Rotations/s = (m/s) / (2 * Pi * r)
      double velocityRotPerSec = scaledSpeed / (2.0 * Math.PI * Constants.Swerve.kWheelRadius);
      mDriveMotor.setControl(mVelocityOut.withVelocity(velocityRotPerSec));
    }

    // Set Steer
    mSteerMotor.setControl(mPositionOut.withPosition(state.angle.getRotations()));
  }

  public Rotation2d getSteerAngle() {
    return Rotation2d.fromRotations(mSteerMotor.getPosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    double velRotPerSec = mDriveMotor.getVelocity().getValueAsDouble();
    double speedMetersPerSecond = velRotPerSec * (2.0 * Math.PI * Constants.Swerve.kWheelRadius);
    return new SwerveModuleState(speedMetersPerSecond, getSteerAngle());
  }

  public SwerveModulePosition getPosition() {
    double driveRotations = mDriveMotor.getPosition().getValueAsDouble();
    double distanceMeters = driveRotations * (2.0 * Math.PI * Constants.Swerve.kWheelRadius);
    return new SwerveModulePosition(distanceMeters, getSteerAngle());
  }

  public void stop() {
    mDriveMotor.stopMotor();
    mSteerMotor.stopMotor();
  }
}
