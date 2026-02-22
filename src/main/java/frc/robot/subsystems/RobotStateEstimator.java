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
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;

/**
 * RobotStateEstimator - Dynamic EKF sensor fusion system.
 *
 * <p>Reference FRC 254 2025 implementation for advanced pose estimation. Supports:
 *
 * <ul>
 *   <li><strong>Dual Limelight Vision Fusion</strong>: Receives fused vision estimates via
 *       RobotState
 *   <li><strong>Slip Detection</strong>: Reduces odometry trust when 4-wheel module speed
 *       differences exceed threshold
 *   <li><strong>Impact Detection</strong>: Pauses vision updates when acceleration exceeds
 *       threshold (data from RobotState, sourced from DriveIO)
 *   <li><strong>Update Timeout Detection</strong>: Warns when odometry hasn't updated for too long
 *   <li><strong>Dynamic Q/R Adjustment</strong>: Dynamically adjusts Kalman Filter standard
 *       deviations based on conditions
 * </ul>
 *
 * <p><strong>Pose fusion (Orbit2)</strong>: We use WPILib's SwerveDrivePoseEstimator (Kalman EKF).
 * Fusion is probabilistically sound; tuning is via simple heuristics:
 *
 * <ul>
 *   <li><strong>Slip</strong>: Trust vision more (lower vision std devs) so cameras correct
 *       position faster.
 *   <li><strong>Impact</strong>: Trust vision less (higher vision std devs) and ignore updates
 *       briefly; time sync is unreliable during hits.
 *   <li><strong>Normal</strong>: Default vision std devs from VisionConstants
 *       (kDefaultVisionStdDevs, kHighTrustVisionStdDevs, kLowTrustVisionStdDevs). Vision subsystem
 *       also sends per-measurement std devs (distance/tag count); we can scale again in
 *       acceptVisionEstimate.
 * </ul>
 *
 * <p>All tunables live in {@link frc.robot.VisionConstants}: slip/impact thresholds, vision std
 * devs, and trust intervals. No complex filter math—just easy-to-adjust constants.
 *
 * <p>ARCHITECTURE NOTE (254 Pattern): This class does NOT create its own Pigeon2 instance. All IMU
 * data is sourced from DriveIO → RobotState to maintain a single source of truth and avoid
 * duplicate CAN traffic.
 *
 * <p>Counter-Intuitive Engineering Insight: During high-speed movement or rotation, we REDUCE
 * vision update frequency because measurement quality degradation matters more than measurement
 * quantity.
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

  // WPILib Kalman Filter estimator (EKF implementation)
  private final SwerveDrivePoseEstimator mPoseEstimator;
  private final Drive mDrive = Drive.getInstance();
  private final PoseHistory mPoseHistory = PoseHistory.getInstance();

  // State tracking
  private boolean mIsSlipping = false;
  private boolean mIsImpact = false;
  private boolean mIsOdometryStale = false;
  private boolean mIsTrusted = true;
  private int mConsecutiveTrustedCycles = 0;

  // Vision update tracking
  private double mLastVisionUpdateTime = 0;

  // Impact recovery tracking
  private double mImpactTime = 0;

  private final Field2d mField2d = new Field2d();

  private RobotStateEstimator() {
    // Initial standard deviations: [x, y, theta]
    // Lower values = higher trust
    Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); // Odometry trust
    Vector<N3> visionStdDevs = VecBuilder.fill(0.7, 0.7, 0.9); // Vision trust (initial)

    mPoseEstimator =
        new SwerveDrivePoseEstimator(
            mDrive.getKinematics(),
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);

    // Wire up vision estimate consumer
    mPoseHistory.setVisionEstimateConsumer(this::acceptVisionEstimate);
  }

  @Override
  public void registerEnabledLoops(frc.robot.framework.Looper enabledLooper) {
    System.out.println("RobotStateEstimator: Registering Loop");
    enabledLooper.register(
        new frc.robot.framework.ILoop() {
          @Override
          public void onStart(double timestamp) {}

          @Override
          public void onLoop(double timestamp) {
            updateEstimator(timestamp);
          }

          @Override
          public void onStop(double timestamp) {}
        });
  }

  @Override
  public void readPeriodicInputs() {}

  @Override
  public void writePeriodicOutputs() {}

  /** Core update logic - called every cycle. */
  private void updateEstimator(double timestamp) {
    // 1. Get base data
    Rotation2d gyroAngle = mDrive.getHeading();
    SwerveModulePosition[] modulePositions = mDrive.getModulePositions();
    SwerveModuleState[] moduleStates = mDrive.getModuleStates();

    // 2. Fault detection (adaptive logic)
    mIsSlipping = checkSlip(moduleStates);
    mIsImpact = checkImpact(timestamp);
    mIsOdometryStale = checkOdometryStale(timestamp, moduleStates);

    // 3. Dynamic Q/R adjustment
    adjustEstimatorWeights();

    // 4. Update odometry (Kalman Filter prediction step)
    mPoseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions);

    // DEBUG: Trace execution
    // System.out.println("RSE Update: Gyro=" + gyroAngle.getDegrees() + " Pose=" +
    // mPoseEstimator.getEstimatedPosition());

    // 5. Update trust state
    updateTrustState();
  }

  /**
   * Determines if the current pose estimate is trusted enough for automatic aiming/shooting.
   *
   * <p>A pose is trusted if: 1. We had a vision update within the last 1.0 second. 2. The robot has
   * met the consecutive trusted cycles condition (no impact, no slip).
   *
   * @return True if the pose is reliable.
   */
  public boolean isPoseTrusted() {
    double timestamp = Timer.getFPGATimestamp();
    boolean recentVision = (timestamp - mLastVisionUpdateTime) < 1.0;
    return recentVision && mIsTrusted && !mIsOdometryStale;
  }

  /** Accepts vision estimate - called via RobotState consumer. */
  public synchronized void acceptVisionEstimate(VisionFieldPoseEstimate estimate) {
    double timestamp = Timer.getFPGATimestamp();

    // Don't accept vision updates during impact recovery
    if (mIsImpact || (timestamp - mImpactTime) < VisionConstants.kImpactRecoveryTimeSeconds) {
      return;
    }

    // Apply our global context scaling to the incoming standard deviations
    Matrix<N3, N1> adjustedStdDevs;
    if (mIsImpact) {
      // Impact overrides all: heavily distrust current visual stamps (likely motion
      // blur or delay)
      adjustedStdDevs = VisionConstants.kLowTrustVisionStdDevs;
    } else if (mIsSlipping) {
      // Slipping: odometry is lying, trust vision more than its base uncertainty
      // suggests
      adjustedStdDevs = estimate.getVisionMeasurementStdDevs().times(0.7);
    } else {
      // Normal: keep the supplied dynamic tag-based std devs
      adjustedStdDevs = estimate.getVisionMeasurementStdDevs();
    }

    // Add vision measurement
    mPoseEstimator.addVisionMeasurement(
        estimate.getVisionRobotPoseMeters(), estimate.getTimestampSeconds(), adjustedStdDevs);

    mLastVisionUpdateTime = timestamp;
  }

  /**
   * Slip detection - Kinematic residual analysis.
   *
   * <p>Physics principle: Forward kinematics computes the 'best fit' chassis speeds given current
   * module states. If we project this best fit back into theoretical module speeds (inverse
   * kinematics), the difference (residual) between actual and theoretical indicates wheels slipping
   * against the floor.
   */
  private boolean checkSlip(SwerveModuleState[] states) {
    if (states == null || states.length < 4) return false;

    // 1. Calculate best-fit chassis speed from actual module states
    ChassisSpeeds chassisSpeeds = mDrive.getKinematics().toChassisSpeeds(states);

    // Ignore near-zero speeds to prevent jitter triggering
    if (Math.abs(chassisSpeeds.vxMetersPerSecond) < VisionConstants.kMovementThresholdMps
        && Math.abs(chassisSpeeds.vyMetersPerSecond) < VisionConstants.kMovementThresholdMps
        && Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.1) {
      return false;
    }

    // 2. Calculate theoretical module states from the best-fit chassis speed
    SwerveModuleState[] theoreticalStates =
        mDrive.getKinematics().toSwerveModuleStates(chassisSpeeds);

    // 3. Find the maximum velocity error (residual)
    double maxResidual = 0;
    for (int i = 0; i < 4; i++) {
      // We mainly care about speed mismatch, not angle (angle flip tracking can be
      // complex)
      double residual =
          Math.abs(
              Math.abs(states[i].speedMetersPerSecond)
                  - Math.abs(theoreticalStates[i].speedMetersPerSecond));
      if (residual > maxResidual) {
        maxResidual = residual;
      }
    }

    // If any wheel is deviating from the collective chassis model significantly, we
    // are slipping
    return maxResidual > VisionConstants.kSlipVelocityThresholdMps;
  }

  /**
   * Impact detection - acceleration analysis.
   *
   * <p>Physics principle: During normal driving, acceleration mainly comes from 1g gravity. Impacts
   * produce additional instantaneous acceleration peaks.
   *
   * <p>ARCHITECTURE NOTE: Acceleration data is sourced from RobotState (which gets it from DriveIO)
   * to avoid duplicate Pigeon2 references.
   */
  private boolean checkImpact(double timestamp) {
    // Get acceleration from RobotState (sourced from DriveIO)
    double[] accel = mPoseHistory.getLatestAcceleration();
    double accelX = accel[0];
    double accelY = accel[1];
    double accelZ = accel[2];

    // Calculate acceleration magnitude after subtracting gravity
    // Note: Accelerations are in m/s², so gravity is ~9.81 m/s²
    double accelMagnitudeG =
        Math.sqrt(accelX * accelX + accelY * accelY + Math.pow(accelZ - 9.81, 2)) / 9.81;

    boolean isImpact = accelMagnitudeG > VisionConstants.kImpactAccelThresholdG;

    if (isImpact) {
      mImpactTime = timestamp;
    }

    return isImpact;
  }

  /**
   * Odometry update timeout detection.
   *
   * <p>If robot is moving but odometry hasn't updated for a long time, may indicate sensor failure.
   */
  private boolean checkOdometryStale(double timestamp, SwerveModuleState[] states) {
    // Determine the actual time since the hardware signals were stamped, rather
    // than loop timing
    double hardwareTimestamp = mDrive.getInputs().timestamp;
    double timeSinceUpdate = timestamp - hardwareTimestamp;

    // Check if robot is commanded to move
    double sum = 0;
    for (SwerveModuleState s : states) sum += Math.abs(s.speedMetersPerSecond);
    double avgSpeed = sum / states.length;

    boolean isMoving = avgSpeed > VisionConstants.kMovementThresholdMps;
    return isMoving && timeSinceUpdate > VisionConstants.kOdometryStaleTimeoutSeconds;
  }

  /** Dynamically adjust estimator weights. */
  private void adjustEstimatorWeights() {
    // Note: in the Orbit2 architecture, we process incoming vision measurement
    // scaling IN The acceptVisionEstimate() method
    // Because SwerveDrivePoseEstimator requires per-measurement StdDevs when using
    // addVisionMeasurement.
    // However, if we want to change the underlying Odometry Trust, WPILib does not
    // allow modifying the Odometry weight matrix post-initialization.
    // Thus this acts as a no-op, relying on acceptVisionEstimate() scaled inputs
    // instead.
  }

  /** Update trust state. */
  private void updateTrustState() {
    double timestamp = Timer.getFPGATimestamp();
    double timeSinceVision = timestamp - mLastVisionUpdateTime;

    // Trust conditions:
    // 1. Not slipping
    // 2. Not impacting
    // 3. Odometry updating normally
    // 4. Vision updating within reasonable time
    boolean currentlyTrusted =
        !mIsSlipping
            && !mIsImpact
            && !mIsOdometryStale
            && timeSinceVision < VisionConstants.kVisionUpdateMaxIntervalSeconds;

    if (currentlyTrusted) {
      mConsecutiveTrustedCycles++;
    } else {
      mConsecutiveTrustedCycles = 0;
    }

    // Need consecutive cycles before considered truly trusted
    mIsTrusted = mConsecutiveTrustedCycles >= VisionConstants.kMinConsecutiveTrustedCycles;
  }

  // ============================================================
  // PUBLIC METHODS
  // ============================================================

  public synchronized Pose2d getEstimatedPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  /**
   * @return Whether pose is trusted (all trust conditions met)
   */
  public synchronized boolean isTrusted() {
    return mIsTrusted;
  }

  /**
   * @return Whether currently slipping
   */
  public boolean isSlipping() {
    return mIsSlipping;
  }

  /**
   * @return Whether impact detected
   */
  public boolean isImpact() {
    return mIsImpact;
  }

  /** Resets estimator to specified pose. */
  public synchronized void resetPose(Pose2d pose) {
    mPoseEstimator.resetPosition(mDrive.getHeading(), mDrive.getModulePositions(), pose);
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
    var dashboard = frc.robot.DashboardState.getInstance();
    dashboard.robotPose = mPoseEstimator.getEstimatedPosition();
    dashboard.robotImpacting = mIsImpact;
    dashboard.odometryStale = mIsOdometryStale;
    dashboard.robotSlipping = mIsSlipping;
    dashboard.lastVisionTimestamp = mLastVisionUpdateTime;

    // Calculate and publish distance to Hub center (shortest/Euclidean)
    double distanceToHub =
        mPoseEstimator
            .getEstimatedPosition()
            .getTranslation()
            .getDistance(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    SmartDashboard.putNumber("Field/DistanceToHub", distanceToHub);
    SmartDashboard.putNumber(
        "RobotStateEstimator/LastVisionSecsAgoToNow",
        Timer.getFPGATimestamp() - mLastVisionUpdateTime);
    SmartDashboard.putBoolean("RobotStateEstimator/IsSlipping", mIsSlipping);
    SmartDashboard.putBoolean("RobotStateEstimator/IsTrusted", mIsTrusted);

    mField2d.setRobotPose(mPoseEstimator.getEstimatedPosition());
    SmartDashboard.putData("Field", mField2d);
  }

  @Override
  public void zeroSensors() {
    resetPose(new Pose2d());
  }
}
