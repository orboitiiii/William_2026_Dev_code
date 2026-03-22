package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Threads;
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

  private Boolean mLastBrakeMode = null;

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
  private final StatusSignal<AngularVelocity> mPigeonPitchRate;
  private final StatusSignal<AngularVelocity> mPigeonRollRate;
  private final StatusSignal<Angle> mPigeonPitch;
  private final StatusSignal<Angle> mPigeonRoll;
  private final StatusSignal<LinearAcceleration> mPigeonAccelX;
  private final StatusSignal<LinearAcceleration> mPigeonAccelY;
  private final StatusSignal<LinearAcceleration> mPigeonAccelZ;

  /** Batched signal array for efficient CAN refresh. */
  private BaseStatusSignal[] mHighFreqSignals;

  private BaseStatusSignal[] mLowFreqSignals;

  // --- Control Request Objects (reused to avoid allocation) ---

  private final VoltageOut mVoltageOut = new VoltageOut(0);
  private final PositionVoltage mPositionOut = new PositionVoltage(0);
  private final VelocityVoltage mVelocityOut = new VelocityVoltage(0);

  /** Initializes all drive hardware and configures motor controllers. */
  @SuppressWarnings("unchecked")
  public DriveIOReal() {
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop(); // Disable logging to save bandwidth and disk I/O

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

    // 提升 Pigeon2 更新率至 250Hz 以利砲塔追蹤 (與高 kP 同步)
    mPigeon.getYaw().setUpdateFrequency(250.0);
    mPigeon.getAngularVelocityZWorld().setUpdateFrequency(250.0);

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

    // Split signals into High Frequency (Control) and Low Frequency (Telemetry)
    List<BaseStatusSignal> highFreqList = new ArrayList<>();
    List<BaseStatusSignal> lowFreqList = new ArrayList<>();

    for (int i = 0; i < 4; i++) {
      mDriveMotors[i] = new TalonFX(driveIds[i]);
      mSteerMotors[i] = new TalonFX(steerIds[i]);
      mCANCoders[i] = new CANcoder(cancoderIds[i]);

      boolean isModuleValid =
          SwerveModuleConfigurator.configure(
              mDriveMotors[i],
              mSteerMotors[i],
              mCANCoders[i],
              Constants.Swerve.kModuleType,
              offsets[i]);

      Timer.delay(0.1);

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

      // 無論 isModuleValid 是否為 true，都要將 Signal 加入更新清單。
      // 否則 updateInputs() 裡面迴圈存取 mDrivePositions[i] 等，如果遇到從未 refresh 的空物件會導致
      // NullPointerException
      // 或引發 CTRE 底層的 Invalid Argument。
      // High Frequency: Position signals are absolutely critical for 250Hz Odometry
      highFreqList.add(mDrivePositions[i]);
      highFreqList.add(mSteerPositions[i]);

      // Low Frequency (50Hz loop): Velocities, Absolute Positions (for seeding),
      // Voltages, Currents
      lowFreqList.add(mDriveVelocities[i]);
      lowFreqList.add(mSteerAbsolutePositions[i]);
      lowFreqList.add(mDriveAppliedVolts[i]);
      lowFreqList.add(mDriveCurrents[i]);
      lowFreqList.add(mDriveSupplyVoltages[i]);
      lowFreqList.add(mSteerVelocities[i]); // We don't control steer velocity directly
      lowFreqList.add(mSteerAppliedVolts[i]);
      lowFreqList.add(mSteerCurrents[i]);

      if (!isModuleValid) {
        System.err.println(
            "CRITICAL: Module "
                + i
                + " FAILED config. Will still poll its status but it may be unresponsive.");
      }
    }

    mPigeonYaw = mPigeon.getYaw();
    mPigeonYawRate = mPigeon.getAngularVelocityZWorld();
    mPigeonPitchRate = mPigeon.getAngularVelocityYWorld();
    mPigeonRollRate = mPigeon.getAngularVelocityXWorld();
    mPigeonPitch = mPigeon.getPitch();
    mPigeonRoll = mPigeon.getRoll();
    mPigeonAccelX = mPigeon.getAccelerationX();
    mPigeonAccelY = mPigeon.getAccelerationY();
    mPigeonAccelZ = mPigeon.getAccelerationZ();

    // Pigeon High Frequency
    highFreqList.add(mPigeonYaw);

    // Pigeon Low Frequency
    lowFreqList.add(mPigeonYawRate);
    lowFreqList.add(mPigeonPitch);
    lowFreqList.add(mPigeonRoll);
    lowFreqList.add(mPigeonPitchRate);
    lowFreqList.add(mPigeonRollRate);
    lowFreqList.add(mPigeonAccelX);
    lowFreqList.add(mPigeonAccelY);
    lowFreqList.add(mPigeonAccelZ);

    mHighFreqSignals = highFreqList.toArray(new BaseStatusSignal[0]);
    mLowFreqSignals = lowFreqList.toArray(new BaseStatusSignal[0]);

    // CAN Bus Optimization (Team 254 approach):
    // Disable or reduce frequency of unused StatusSignals to prevent CAN bus
    // saturation.
    // This creates an "opt-in" model where only explicitly configured signals are
    // sent.
    // Without this, default signals at high rates can consume 60%+ of CAN
    // bandwidth.
    // CRITICAL: We must call optimizeBusUtilization() BEFORE scheduling the user
    // signals.
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

    // PAUSE: Allow the CAN transmit buffer to drain "Disable" commands before
    // sending "Enable" commands.
    // This prevents "CAN Output Buffer Full" and "Could not transmit" errors during
    // startup.
    Timer.delay(0.25);

    // Configure Update Frequencies
    // High Frequency: 250Hz (Matches Odometry Loop)
    BaseStatusSignal.setUpdateFrequencyForAll(250.0, mHighFreqSignals);

    // Low Frequency: 4Hz (Telemetry ~250ms)
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, mLowFreqSignals);

    // CRITICAL FIX: Steer motors use RemoteCANcoder for feedback!
    // The CANcoder must broadcast its absolute position at 100Hz so the TalonFX
    // can run its 1000Hz onboard steer PID smoothly. 4Hz is far too slow!
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, mSteerAbsolutePositions);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.timestamp = Timer.getFPGATimestamp();

    // Synchronous batch refresh of High Frequency (Control) signals only
    // This blocks for up to 20ms waiting for signals.
    BaseStatusSignal.refreshAll(mHighFreqSignals);

    // Low Frequency signals are NOT refreshed here to avoid blocking.
    // They are automatically updated in background or we read the last cached
    // value.
    // We call waitForUpdate(0) individually which updates the signal object from
    // the JNI buffer
    // without waiting for a new frame (since timeout is 0).
    for (BaseStatusSignal sig : mLowFreqSignals) {
      if (sig instanceof StatusSignal) {
        ((StatusSignal<?>) sig).waitForUpdate(0.0);
      }
    }

    // IMU data
    inputs.gyroYaw = Rotation2d.fromDegrees(mPigeonYaw.getValueAsDouble());
    inputs.gyroYawVelocityRadPerSec = Math.toRadians(mPigeonYawRate.getValueAsDouble());
    inputs.gyroPitchVelocityRadPerSec = Math.toRadians(mPigeonPitchRate.getValueAsDouble());
    inputs.gyroRollVelocityRadPerSec = Math.toRadians(mPigeonRollRate.getValueAsDouble());
    inputs.gyroPitchRad = Math.toRadians(mPigeonPitch.getValueAsDouble());
    inputs.gyroRollRad = Math.toRadians(mPigeonRoll.getValueAsDouble());

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
    if (mLastBrakeMode != null && mLastBrakeMode == enable) {
      return;
    }
    mLastBrakeMode = enable;
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

  // ============================================================
  // 250Hz ODOMETRY THREAD
  // ============================================================

  private volatile OdometrySnapshot mLatestOdometrySnapshot = new OdometrySnapshot();

  @Override
  public OdometrySnapshot getLatestOdometrySnapshot() {
    return mLatestOdometrySnapshot;
  }

  @Override
  public void startOdometryThread() {
    BaseStatusSignal pigeonYawClone = mPigeonYaw.clone();

    BaseStatusSignal[] drivePosClones =
        new BaseStatusSignal[] {
          mDrivePositions[0].clone(), mDrivePositions[1].clone(),
          mDrivePositions[2].clone(), mDrivePositions[3].clone()
        };

    BaseStatusSignal[] steerPosClones =
        new BaseStatusSignal[] {
          mSteerPositions[0].clone(), mSteerPositions[1].clone(),
          mSteerPositions[2].clone(), mSteerPositions[3].clone()
        };

    BaseStatusSignal[] odometryDriveSignals =
        new BaseStatusSignal[] {
          drivePosClones[0],
          drivePosClones[1],
          drivePosClones[2],
          drivePosClones[3],
          steerPosClones[0],
          steerPosClones[1],
          steerPosClones[2],
          steerPosClones[3]
        };

    Thread thread =
        new Thread(
            () -> {
              Threads.setCurrentThreadPriority(true, 41);
              while (!Thread.interrupted()) {
                // RIO CAN bus DOES NOT support multi-signal synchronization (-10002 error).
                // We wait for the pigeon's high frequency packet to pace the 250Hz thread,
                // and then non-blockingly refresh the remaining motor signals.
                var status = BaseStatusSignal.waitForAll(0.02, pigeonYawClone);
                if (!status.isOK()) {
                  continue;
                }
                BaseStatusSignal.refreshAll(odometryDriveSignals);

                OdometrySnapshot snap = new OdometrySnapshot();
                snap.timestamp = Timer.getFPGATimestamp();
                snap.gyroYaw = Rotation2d.fromDegrees(pigeonYawClone.getValueAsDouble());
                snap.gyroYawVelocityRadPerSec = Math.toRadians(mPigeonYawRate.getValueAsDouble());
                snap.gyroPitchVelocityRadPerSec =
                    Math.toRadians(mPigeonPitchRate.getValueAsDouble());
                snap.gyroRollVelocityRadPerSec = Math.toRadians(mPigeonRollRate.getValueAsDouble());
                snap.accelMetersPerSec2[0] = mPigeonAccelX.getValueAsDouble() * 9.81;
                snap.accelMetersPerSec2[1] = mPigeonAccelY.getValueAsDouble() * 9.81;
                snap.accelMetersPerSec2[2] = mPigeonAccelZ.getValueAsDouble() * 9.81;

                for (int i = 0; i < 4; i++) {
                  snap.drivePositionRotations[i] = drivePosClones[i].getValueAsDouble();
                  snap.steerPositionRotations[i] = steerPosClones[i].getValueAsDouble();
                  double distMeters =
                      snap.drivePositionRotations[i]
                          * Constants.Swerve.Control.kWheelCircumference
                          * Constants.Swerve.Control.kDrivePositionCoefficient;
                  snap.modulePositions[i] =
                      new SwerveModulePosition(
                          distMeters, Rotation2d.fromRotations(snap.steerPositionRotations[i]));
                }

                mLatestOdometrySnapshot = snap;
              }
            });
    thread.setDaemon(true);
    thread.setName("Odometry-250Hz");
    thread.start();
    System.out.println("[DriveIOReal] 250Hz Odometry Thread started.");
  }
}
