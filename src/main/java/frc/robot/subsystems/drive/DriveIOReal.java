package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModuleConfigurator;
import java.util.ArrayList;
import java.util.List;

/**
 * Real hardware implementation of DriveIO using CTRE Phoenix 6.
 *
 * <p>This class manages all CAN communication with the swerve drive hardware:
 *
 * <ul>
 *   <li>4x TalonFX drive motors
 *   <li>4x TalonFX steer motors
 *   <li>4x CANcoders for absolute position
 *   <li>1x Pigeon 2.0 IMU
 * </ul>
 *
 * <p><strong>CAN Bus Optimization</strong>: All StatusSignals are batched into a single array and
 * refreshed together using {@code BaseStatusSignal.refreshAll()}. This reduces CAN utilization
 * compared to individual signal reads.
 *
 * <p><strong>Signal Update Rate</strong>: All signals are configured to update at 100Hz for
 * low-latency control feedback.
 *
 * @see DriveIO
 * @see SwerveModuleConfigurator
 */
public class DriveIOReal implements DriveIO {
  private final TalonFX[] mDriveMotors = new TalonFX[4];
  private final TalonFX[] mSteerMotors = new TalonFX[4];
  private final CANcoder[] mCANCoders = new CANcoder[4];
  private final Pigeon2 mPigeon;

  // --- Cached Status Signals ---
  private final StatusSignal<Angle>[] mDrivePositions;
  private final StatusSignal<AngularVelocity>[] mDriveVelocities;
  private final StatusSignal<Voltage>[] mDriveAppliedVolts;
  private final StatusSignal<Current>[] mDriveCurrents;
  private final StatusSignal<Voltage>[] mDriveSupplyVoltages;

  private final StatusSignal<Angle>[] mSteerPositions;
  private final StatusSignal<AngularVelocity>[] mSteerVelocities;
  private final StatusSignal<Voltage>[] mSteerAppliedVolts;
  private final StatusSignal<Current>[] mSteerCurrents;
  private final StatusSignal<Angle>[] mSteerAbsolutePositions;

  private final StatusSignal<Angle> mPigeonYaw;
  private final StatusSignal<AngularVelocity> mPigeonYawRate;
  private final StatusSignal<LinearAcceleration> mPigeonAccelX;
  private final StatusSignal<LinearAcceleration> mPigeonAccelY;
  private final StatusSignal<LinearAcceleration> mPigeonAccelZ;

  /** Batched signal array for efficient CAN refresh. */
  private final BaseStatusSignal[] mAllSignals;

  // --- Control Request Objects (reused to avoid allocation) ---
  private final DutyCycleOut mDutyCycleOut = new DutyCycleOut(0);
  private final VoltageOut mVoltageOut = new VoltageOut(0);
  private final PositionVoltage mPositionOut = new PositionVoltage(0);
  private final VelocityVoltage mVelocityOut = new VelocityVoltage(0);

  /** Initializes all drive hardware and configures motor controllers. */
  @SuppressWarnings("unchecked")
  public DriveIOReal() {
    mDrivePositions = new StatusSignal[4];
    mDriveVelocities = new StatusSignal[4];
    mDriveAppliedVolts = new StatusSignal[4];
    mDriveCurrents = new StatusSignal[4];
    mDriveSupplyVoltages = new StatusSignal[4];
    mSteerPositions = new StatusSignal[4];
    mSteerVelocities = new StatusSignal[4];
    mSteerAppliedVolts = new StatusSignal[4];
    mSteerCurrents = new StatusSignal[4];
    mSteerAbsolutePositions = new StatusSignal[4];

    mPigeon = new Pigeon2(Constants.Swerve.kPigeonId);

    // IMPORTANT: MountPose configuration is intentionally NOT set in code.
    // Following Team 254's approach: Use Phoenix Tuner X to calibrate MountPose,
    // which persists the values to the device's flash memory. This avoids:
    // 1. Overwriting Tuner X calibration on every robot startup
    // 2. Hardcoding potentially inaccurate values
    // To calibrate: Phoenix Tuner X -> Config -> Mount Pose Calibration

    int[] driveIds = {
      Constants.Swerve.kFLDriveId,
      Constants.Swerve.kFRDriveId,
      Constants.Swerve.kBLDriveId,
      Constants.Swerve.kBRDriveId
    };
    int[] steerIds = {
      Constants.Swerve.kFLSteerId,
      Constants.Swerve.kFRSteerId,
      Constants.Swerve.kBLSteerId,
      Constants.Swerve.kBRSteerId
    };
    int[] cancoderIds = {
      Constants.Swerve.kFLEncoderId,
      Constants.Swerve.kFREncoderId,
      Constants.Swerve.kBLEncoderId,
      Constants.Swerve.kBREncoderId
    };
    double[] offsets = {
      Constants.Swerve.kFLOffset,
      Constants.Swerve.kFROffset,
      Constants.Swerve.kBLOffset,
      Constants.Swerve.kBROffset
    };

    List<BaseStatusSignal> signalsList = new ArrayList<>();

    for (int i = 0; i < 4; i++) {
      mDriveMotors[i] = new TalonFX(driveIds[i]);
      mSteerMotors[i] = new TalonFX(steerIds[i]);
      mCANCoders[i] = new CANcoder(cancoderIds[i]);

      SwerveModuleConfigurator.configure(
          mDriveMotors[i], mSteerMotors[i], mCANCoders[i], offsets[i]);

      // Cache signal references
      mDrivePositions[i] = mDriveMotors[i].getPosition();
      mDriveVelocities[i] = mDriveMotors[i].getVelocity();
      mDriveAppliedVolts[i] = mDriveMotors[i].getMotorVoltage();
      mDriveCurrents[i] = mDriveMotors[i].getSupplyCurrent();
      mDriveSupplyVoltages[i] = mDriveMotors[i].getSupplyVoltage();

      mSteerPositions[i] = mSteerMotors[i].getPosition();
      mSteerVelocities[i] = mSteerMotors[i].getVelocity();
      mSteerAppliedVolts[i] = mSteerMotors[i].getMotorVoltage();
      mSteerCurrents[i] = mSteerMotors[i].getSupplyCurrent();
      mSteerAbsolutePositions[i] = mCANCoders[i].getAbsolutePosition();

      signalsList.add(mDrivePositions[i]);
      signalsList.add(mDriveVelocities[i]);
      signalsList.add(mDriveAppliedVolts[i]);
      signalsList.add(mDriveCurrents[i]);
      signalsList.add(mDriveSupplyVoltages[i]);
      signalsList.add(mSteerPositions[i]);
      signalsList.add(mSteerVelocities[i]);
      signalsList.add(mSteerAppliedVolts[i]);
      signalsList.add(mSteerCurrents[i]);
      signalsList.add(mSteerAbsolutePositions[i]);
    }

    mPigeonYaw = mPigeon.getYaw();
    mPigeonYawRate = mPigeon.getAngularVelocityZWorld();
    mPigeonAccelX = mPigeon.getAccelerationX();
    mPigeonAccelY = mPigeon.getAccelerationY();
    mPigeonAccelZ = mPigeon.getAccelerationZ();

    signalsList.add(mPigeonYaw);
    signalsList.add(mPigeonYawRate);
    signalsList.add(mPigeonAccelX);
    signalsList.add(mPigeonAccelY);
    signalsList.add(mPigeonAccelZ);

    mAllSignals = signalsList.toArray(new BaseStatusSignal[0]);

    // Configure 100Hz update rate for motor signals
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, mAllSignals);

    // CRITICAL: Set Pigeon yaw signals to 250Hz (Team 254 approach).
    // Higher frequency reduces integration error and matches high-frequency
    // odometry loops.
    // The Pigeon's internal EKF runs at 800Hz, so 250Hz captures its fused output
    // accurately.
    mPigeonYaw.setUpdateFrequency(250.0);
    mPigeonYawRate.setUpdateFrequency(250.0);

    // CAN Bus Optimization (Team 254 approach):
    // Disable or reduce frequency of unused StatusSignals to prevent CAN bus
    // saturation.
    // This creates an "opt-in" model where only explicitly configured signals are
    // sent.
    // Without this, default signals at high rates can consume 60%+ of CAN
    // bandwidth.
    for (TalonFX motor : mDriveMotors) {
      motor.optimizeBusUtilization();
    }
    for (TalonFX motor : mSteerMotors) {
      motor.optimizeBusUtilization();
    }
    for (CANcoder encoder : mCANCoders) {
      encoder.optimizeBusUtilization();
    }
    mPigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.timestamp = Timer.getFPGATimestamp();

    // Synchronous batch refresh of all CAN signals
    BaseStatusSignal.refreshAll(mAllSignals);

    // IMU data
    inputs.gyroYaw = Rotation2d.fromDegrees(mPigeonYaw.getValueAsDouble());
    inputs.gyroYawVelocityRadPerSec = Math.toRadians(mPigeonYawRate.getValueAsDouble());

    // Convert accelerometer from G to m/s²
    inputs.accelMetersPerSec2[0] = mPigeonAccelX.getValueAsDouble() * 9.81;
    inputs.accelMetersPerSec2[1] = mPigeonAccelY.getValueAsDouble() * 9.81;
    inputs.accelMetersPerSec2[2] = mPigeonAccelZ.getValueAsDouble() * 9.81;

    // Module data
    for (int i = 0; i < 4; i++) {
      inputs.drivePositionRotations[i] = mDrivePositions[i].getValueAsDouble();
      inputs.driveVelocityRotationsPerSec[i] = mDriveVelocities[i].getValueAsDouble();
      inputs.driveAppliedVolts[i] = mDriveAppliedVolts[i].getValueAsDouble();
      inputs.driveCurrentAmps[i] = mDriveCurrents[i].getValueAsDouble();
      inputs.driveSupplyVoltage[i] = mDriveSupplyVoltages[i].getValueAsDouble();

      inputs.steerPositionRotations[i] = mSteerPositions[i].getValueAsDouble();
      inputs.steerVelocityRotationsPerSec[i] = mSteerVelocities[i].getValueAsDouble();
      inputs.steerAppliedVolts[i] = mSteerAppliedVolts[i].getValueAsDouble();
      inputs.steerCurrentAmps[i] = mSteerCurrents[i].getValueAsDouble();
      inputs.steerAbsolutePositionRotations[i] = mSteerAbsolutePositions[i].getValueAsDouble();

      // Populate connection status (Passive Check)
      inputs.driveMotorConnected[i] = mDrivePositions[i].getStatus().isOK();
      inputs.steerMotorConnected[i] = mSteerPositions[i].getStatus().isOK();
      inputs.cancoderConnected[i] = mSteerAbsolutePositions[i].getStatus().isOK();
    }

    // Pigeon connection status
    inputs.pigeonConnected = mPigeonYaw.getStatus().isOK();
  }

  @Override
  public void setDriveVoltage(int moduleIndex, double volts) {
    mDriveMotors[moduleIndex].setControl(mVoltageOut.withOutput(volts));
  }

  @Override
  public void setSteerVoltage(int moduleIndex, double volts) {
    mSteerMotors[moduleIndex].setControl(mVoltageOut.withOutput(volts));
  }

  @Override
  public void setDriveVelocity(int moduleIndex, double velocityRotPerSec) {
    // TalonFX VelocityVoltage takes rotations/second
    mDriveMotors[moduleIndex].setControl(mVelocityOut.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setSteerPosition(int moduleIndex, double rotations) {
    mSteerMotors[moduleIndex].setControl(mPositionOut.withPosition(rotations));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var mode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    for (var m : mDriveMotors) m.setNeutralMode(mode);
  }

  @Override
  public void setGyroYaw(double degrees) {
    mPigeon.setYaw(degrees);
  }

  // --- Calibration Methods ---

  @Override
  public double[] getAbsolutePositionsRotations() {
    double[] positions = new double[4];
    for (int i = 0; i < 4; i++) {
      mSteerAbsolutePositions[i].refresh();
      positions[i] = mSteerAbsolutePositions[i].getValueAsDouble();
    }
    return positions;
  }

  @Override
  public double[] getCurrentMagnetOffsets() {
    double[] offsets = new double[4];
    MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
    for (int i = 0; i < 4; i++) {
      mCANCoders[i].getConfigurator().refresh(magnetConfigs);
      offsets[i] = magnetConfigs.MagnetOffset;
    }
    return offsets;
  }

  @Override
  public void setMagnetOffset(int moduleIndex, double offset) {
    if (moduleIndex < 0 || moduleIndex >= 4) {
      System.err.println("[DriveIOReal] Invalid module index: " + moduleIndex);
      return;
    }

    CANcoderConfiguration config = new CANcoderConfiguration();
    mCANCoders[moduleIndex].getConfigurator().refresh(config);
    config.MagnetSensor.MagnetOffset = offset;
    mCANCoders[moduleIndex].getConfigurator().apply(config);

    System.out.println("[DriveIOReal] Module " + moduleIndex + " MagnetOffset set to: " + offset);
  }

  @Override
  public void playTone(double frequencyHz) {
    MusicTone tone = new MusicTone(frequencyHz);
    for (int i = 0; i < 4; i++) {
      mDriveMotors[i].setControl(tone);
    }
  }

  @Override
  public int[] getDriveMotorFirmwareVersions() {
    int[] versions = new int[4];
    for (int i = 0; i < 4; i++) {
      var versionSignal = mDriveMotors[i].getVersion();
      versionSignal.refresh();
      if (versionSignal.getStatus().isOK()) {
        versions[i] = versionSignal.getValue();
      } else {
        versions[i] = 0; // Read failed
      }
    }
    return versions;
  }

  @Override
  public int[] getSteerMotorFirmwareVersions() {
    int[] versions = new int[4];
    for (int i = 0; i < 4; i++) {
      var versionSignal = mSteerMotors[i].getVersion();
      versionSignal.refresh();
      if (versionSignal.getStatus().isOK()) {
        versions[i] = versionSignal.getValue();
      } else {
        versions[i] = 0;
      }
    }
    return versions;
  }

  @Override
  public int getPigeonFirmwareVersion() {
    var versionSignal = mPigeon.getVersion();
    versionSignal.refresh();
    if (versionSignal.getStatus().isOK()) {
      return versionSignal.getValue();
    }
    return 0;
  }
}
