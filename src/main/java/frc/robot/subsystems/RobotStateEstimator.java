package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.PoseHistory;
import frc.robot.VisionConstants;
import frc.robot.framework.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;

/**
 * Single-source-of-truth pose estimator (Reference FRC 254 2025 architecture).
 *
 * <p>This class owns the ONLY {@link SwerveDrivePoseEstimator} in the system. There is no redundant
 * {@code SwerveDriveOdometry} in Drive — all pose estimation flows through this class.
 *
 * <h3>Data Flow</h3>
 *
 * <pre>
 * DriveIOReal (250Hz thread)
 *   └─ OdometrySnapshot (volatile, immutable per cycle)
 *        └─ RobotStateEstimator.readPeriodicInputs() (250Hz via Looper)
 *             ├─ SwerveDrivePoseEstimator.updateWithTime()  ← odometry prediction
 *             ├─ PoseHistory.addFieldToVehicleObservation() ← fused pose published
 *             └─ PoseHistory.addFieldVelocityObservation()  ← field velocity published
 *
 * VisionSubsystem.writePeriodicOutputs()
 *   └─ PoseHistory.updateMegatagEstimate()
 *        └─ acceptVisionEstimate()
 *             └─ SwerveDrivePoseEstimator.addVisionMeasurement() ← vision correction
 * </pre>
 *
 * <h3>Standard Deviation Tuning (Reference 254 2025)</h3>
 *
 * <ul>
 *   <li>Odometry: (0.3, 0.3, 0.2) — allows vision to influence the estimate
 *   <li>Vision: dynamically scaled per-measurement by tag count, distance, and robot state
 * </ul>
 */
public class RobotStateEstimator extends Subsystem {
  private static RobotStateEstimator mInstance;

  public static RobotStateEstimator getInstance() {
    if (mInstance == null) {
      mInstance = new RobotStateEstimator();
    }
    return mInstance;
  }

  public static boolean hasInstance() {
    return mInstance != null;
  }

  private final SwerveDrivePoseEstimator mPoseEstimator;
  private final Drive mDrive = Drive.getInstance();
  private final PoseHistory mPoseHistory = PoseHistory.getInstance();

  // Fault state
  private boolean mIsOdometryStale = false;
  private boolean mIsTrusted = true;
  private int mConsecutiveTrustedCycles = 0;

  // Vision update tracking
  private double mLastVisionUpdateTime = 0;

  // 250Hz snapshot tracking — skip duplicate updates
  private double mLastOdometrySnapshotTimestamp = 0;

  // Continuous trust score (0-1) for auto-shoot decisions
  private double mTrustScore = 0.0;

  // Odometry drift estimator: accumulates distance travelled since last vision
  // update. Used to bound maximum expected position error from dead-reckoning.
  private double mOdometryDriftMeters = 0.0;

  // Pose stability ring buffer
  private final Pose2d[] mRecentPoses = new Pose2d[VisionConstants.kPoseStabilityWindowSize];
  private int mRecentPoseIdx = 0;
  private int mRecentPoseCount = 0;

  private final Field2d mField2d = new Field2d();

  private RobotStateEstimator() {
    // 254 2025 reference: enabled (0.3, 0.3, 0.2), disabled (1.0, 1.0, 1.0)
    Vector<N3> stateStdDevs = VecBuilder.fill(0.3, 0.3, 0.2);
    Vector<N3> visionStdDevs = VecBuilder.fill(0.7, 0.7, 0.9);

    Rotation2d initialGyro = mDrive.getHeading();
    SwerveModulePosition[] initialPositions = mDrive.getModulePositions();

    mPoseEstimator =
        new SwerveDrivePoseEstimator(
            mDrive.getKinematics(),
            initialGyro,
            initialPositions,
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);

    mPoseHistory.setVisionEstimateConsumer(this::acceptVisionEstimate);
  }

  // ============================================================
  // SUBSYSTEM LIFECYCLE
  // ============================================================

  @Override
  public void registerEnabledLoops(frc.robot.framework.Looper enabledLooper) {
    // No separate loop needed — all work is done in readPeriodicInputs().
  }

  /**
   * Core 250Hz update — reads the latest OdometrySnapshot from the dedicated thread, advances the
   * EKF prediction step, publishes the fused pose and velocity to PoseHistory, and runs fault
   * detection.
   *
   * <p>Execution order guarantee: SubsystemManager calls readPeriodicInputs() on subsystems in
   * registration order. Drive is registered before RobotStateEstimator, so Drive's sensor data
   * (used for slip/impact detection) is always fresh when this method runs.
   */
  @Override
  public void readPeriodicInputs() {
    DriveIO.OdometrySnapshot snap = mDrive.getIO().getLatestOdometrySnapshot();

    // Skip if snapshot hasn't been updated (avoids double-processing same data)
    if (snap.timestamp <= mLastOdometrySnapshotTimestamp || snap.timestamp == 0) {
      return;
    }

    // --- Delayed Initial Pose Calibration (Anti-Race Condition) ---
    // 等到我們真正拿到第一包有效且非 0 的 250Hz 封包時，才用裡面的 modulePositions 去做起點對位
    if (frc.robot.GlobalData.pendingInitialPose != null) {
      mPoseEstimator.resetPosition(
          snap.gyroYaw, snap.modulePositions, frc.robot.GlobalData.pendingInitialPose);
      mPoseHistory.reset(snap.timestamp, frc.robot.GlobalData.pendingInitialPose);

      frc.robot.GlobalData.pendingInitialPose = null; // Consume
    }

    mLastOdometrySnapshotTimestamp = snap.timestamp;

    // --- EKF Prediction Step ---
    synchronized (this) {
      mPoseEstimator.updateWithTime(snap.timestamp, snap.gyroYaw, snap.modulePositions);
    }

    // --- Publish fused pose to PoseHistory ---
    Pose2d fusedPose = getEstimatedPose();
    mPoseHistory.addFieldToVehicleObservation(snap.timestamp, fusedPose);

    // --- Publish field velocity using EKF heading ---
    ChassisSpeeds robotVel = mDrive.getRobotVelocity();
    ChassisSpeeds fieldVel =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, fusedPose.getRotation());
    mPoseHistory.addFieldVelocityObservation(snap.timestamp, fieldVel);

    // --- Pose stability tracking ---
    mRecentPoses[mRecentPoseIdx] = fusedPose;
    mRecentPoseIdx = (mRecentPoseIdx + 1) % mRecentPoses.length;
    if (mRecentPoseCount < mRecentPoses.length) mRecentPoseCount++;

    // --- Odometry drift estimator ---
    // Accumulate estimated position error from dead-reckoning since last vision
    double speed = Math.hypot(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);

    // [KINEMATICS EXPORT] Export true chassis speed to GlobalData for feed-forward
    // compensation
    frc.robot.GlobalData.chassisSpeedMetersPerSec = speed;

    double dtSnap = frc.robot.Constants.kLooperDt;
    mOdometryDriftMeters += speed * dtSnap * VisionConstants.kOdometryDriftRatePerMeter;

    // --- Fault detection ---
    mIsOdometryStale = checkOdometryStale(snap);
    updateTrustState();
  }

  @Override
  public void writePeriodicOutputs() {}

  // ============================================================
  // VISION FUSION
  // ============================================================

  /**
   * Accepts vision estimate — called via PoseHistory consumer.
   *
   * <p>Innovation gating: if the vision measurement is far from the current EKF estimate, its
   * stdDevs are inflated so the Kalman gain is reduced (soft reject instead of hard reject). This
   * protects against occasional outliers while still allowing the filter to converge after large
   * resets.
   */
  public synchronized void acceptVisionEstimate(VisionFieldPoseEstimate estimate) {
    double timestamp = Timer.getFPGATimestamp();

    Matrix<N3, N1> adjustedStdDevs = estimate.getVisionMeasurementStdDevs();

    double innovation =
        getEstimatedPose()
            .getTranslation()
            .getDistance(estimate.getVisionRobotPoseMeters().getTranslation());

    if (innovation > VisionConstants.kMaxInnovationMeters) {
      double scale =
          Math.pow(
              innovation / VisionConstants.kMaxInnovationMeters,
              VisionConstants.kInnovationScalingExponent);
      adjustedStdDevs = adjustedStdDevs.times(scale);
    }

    mPoseEstimator.addVisionMeasurement(
        estimate.getVisionRobotPoseMeters(), estimate.getTimestampSeconds(), adjustedStdDevs);

    mLastVisionUpdateTime = timestamp;
    mOdometryDriftMeters = 0.0;
  }

  // ============================================================
  // FAULT DETECTION
  // ============================================================

  private boolean checkOdometryStale(DriveIO.OdometrySnapshot snap) {
    double timeSinceUpdate = Timer.getFPGATimestamp() - snap.timestamp;
    SwerveModuleState[] states = mDrive.getModuleStates();

    double sum = 0;
    for (SwerveModuleState s : states) sum += Math.abs(s.speedMetersPerSecond);
    double avgSpeed = sum / states.length;

    boolean isMoving = avgSpeed > VisionConstants.kMovementThresholdMps;
    return isMoving && timeSinceUpdate > VisionConstants.kOdometryStaleTimeoutSeconds;
  }

  private void updateTrustState() {
    // Trust requires: no active faults AND estimated drift within budget.
    // The drift budget is generous (default 0.30m) — it represents the point
    // at which pure dead-reckoning error becomes too large for auto-scoring.
    boolean currentlyTrusted =
        !mIsOdometryStale && mOdometryDriftMeters < VisionConstants.kMaxOdometryDriftMeters;

    if (currentlyTrusted) {
      mConsecutiveTrustedCycles++;
    } else {
      mConsecutiveTrustedCycles = 0;
    }

    mIsTrusted = mConsecutiveTrustedCycles >= VisionConstants.kMinConsecutiveTrustedCycles;

    // --- Continuous trust score (EMA) ---
    double faultFactor = mIsOdometryStale ? 0.3 : 1.0;

    // Drift-based freshness: 1.0 at zero drift, decays to 0 at max drift
    double driftFreshness =
        Math.max(0.0, 1.0 - mOdometryDriftMeters / VisionConstants.kMaxOdometryDriftMeters);

    double stabilityFactor = isPoseStable() ? 1.0 : 0.5;

    double targetScore = faultFactor * driftFreshness * stabilityFactor;
    double alpha = VisionConstants.kTrustScoreAlpha;
    mTrustScore = mTrustScore * (1.0 - alpha) + targetScore * alpha;
  }

  /**
   * Checks whether recent poses are consistent (low jitter). Returns false if the stability window
   * is not yet filled.
   */
  private boolean isPoseStable() {
    if (mRecentPoseCount < VisionConstants.kPoseStabilityMinSamples) return false;

    int latestIdx = (mRecentPoseIdx - 1 + mRecentPoses.length) % mRecentPoses.length;
    Pose2d latest = mRecentPoses[latestIdx];
    if (latest == null) return false;

    double dt = frc.robot.Constants.kLooperDt;
    double maxSpeedMps = 5.0; // Assume max robot speed of 5 m/s

    for (int i = 0; i < mRecentPoseCount; i++) {
      if (i == latestIdx) continue;
      Pose2d p = mRecentPoses[i];
      if (p == null) continue;

      int ageCycles = (latestIdx - i + mRecentPoses.length) % mRecentPoses.length;
      double allowedDist = VisionConstants.kMaxPoseJitterMeters + (ageCycles * dt * maxSpeedMps);

      if (latest.getTranslation().getDistance(p.getTranslation()) > allowedDist) {
        return false;
      }
    }
    return true;
  }

  // ============================================================
  // PUBLIC API
  // ============================================================

  public synchronized Pose2d getEstimatedPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  public boolean isPoseTrusted() {
    // 💡 反直覺工程觀點 (Counter-Intuitive Engineering Insight):
    // 雖然 Swerve Odometry 在短時間內能精準推算，但如果完全失去視覺校正，
    // 其累積誤差會在幾秒後大到足以讓自動瞄準 (Auto-Shoot) 失准。
    // 因此除了里程計漂移估計 (mIsTrusted) 以外，增加硬性的「3秒內必定要有一次有效視覺更新」
    // (不是只有看到 Tag，而是真正送入 EKF 進行更新)，才是最安全的失效保護機制。
    double timeSinceLastVision =
        edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - mLastVisionUpdateTime;
    boolean hasRecentVision = timeSinceLastVision <= 3.0;

    return mIsTrusted && !mIsOdometryStale && hasRecentVision;
  }

  /**
   * Estimated odometry drift in meters since the last vision correction. Useful for UI and
   * debugging.
   */
  public double getOdometryDriftMeters() {
    return mOdometryDriftMeters;
  }

  /**
   * Returns whether the pose is accurate and stable enough for autonomous shooting. Combines
   * continuous trust score, pose stability, and fault detection into a single gate.
   */
  public boolean isReadyToShoot() {
    return mTrustScore >= VisionConstants.kTrustScoreForAutoShoot && isPoseStable();
  }

  /**
   * Continuous trust score (0–1). Suitable for UI display, interpolating shot parameters, or
   * gradual autonomous decisions.
   */
  public double getPoseTrustScore() {
    return mTrustScore;
  }

  public synchronized boolean isTrusted() {
    return mIsTrusted;
  }

  public synchronized void resetPose(Pose2d pose) {
    mPoseEstimator.resetPosition(mDrive.getHeading(), mDrive.getModulePositions(), pose);
    mOdometryDriftMeters = 0.0;
  }

  @Override
  public void stop() {}

  @Override
  public boolean checkConnectionActive() {
    return true;
  }

  @Override
  public boolean checkConnectionPassive() {
    return true;
  }

  @Override
  public boolean checkSanityPassive() {
    return mIsTrusted;
  }

  @Override
  public void outputTelemetry() {
    Pose2d currentPose = getEstimatedPose();

    var dashboard = frc.robot.DashboardState.getInstance();
    dashboard.robotPose = currentPose;
    dashboard.odometryDrift = mOdometryDriftMeters;
    dashboard.poseStable = isPoseStable();
    dashboard.isTrusted = mIsTrusted;
    dashboard.trustScore = mTrustScore;

    double distanceToHub =
        currentPose
            .getTranslation()
            .getDistance(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    SmartDashboard.putNumber("Field/DistanceToHub", distanceToHub);
    SmartDashboard.putNumber(
        "RobotStateEstimator/LastVisionSecsAgoToNow",
        Timer.getFPGATimestamp() - mLastVisionUpdateTime);
    SmartDashboard.putBoolean("RobotStateEstimator/IsTrusted", mIsTrusted);
    SmartDashboard.putNumber("RobotStateEstimator/TrustScore", mTrustScore);
    SmartDashboard.putNumber("RobotStateEstimator/OdometryDrift", mOdometryDriftMeters);
    SmartDashboard.putBoolean("RobotStateEstimator/ReadyToShoot", isReadyToShoot());
    SmartDashboard.putBoolean("RobotStateEstimator/PoseStable", isPoseStable());

    SmartDashboard.putNumber("RobotStateEstimator/RawGyroDeg", mDrive.getHeading().getDegrees());
    SmartDashboard.putNumber(
        "RobotStateEstimator/EKFHeadingDeg", currentPose.getRotation().getDegrees());

    mField2d.setRobotPose(currentPose);
    SmartDashboard.putData("Field", mField2d);
  }

  @Override
  public void zeroSensors() {
    resetPose(new Pose2d());
  }
}
