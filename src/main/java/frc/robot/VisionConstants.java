package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Vision system constants configuration - Inspired by FRC 254 2025 design.
 *
 * <p>Contains all thresholds for vision fusion, odometry trust detection, and dynamic Q/R
 * adjustment.
 *
 * <p>Design rationale based on physics considerations:
 *
 * <ul>
 *   <li>Slip detection: Triggered when wheel speed differences exceed expected error
 *   <li>Impact detection: Triggered when acceleration exceeds gravity background + safety margin
 *   <li>Vision rejection: Based on AprilTag geometry properties and ambiguity
 * </ul>
 */
public final class VisionConstants {

  // ============================================================
  // LIMELIGHT CAMERA NAMES
  // ============================================================

  /** Front Limelight name (NetworkTables) */
  public static final String kFrontLimelightName = "limelight-front";

  /** Left Limelight name (NetworkTables) - Chassis mounted */
  public static final String kLeftLimelightName = "limelight-left";

  /** Up Limelight name (NetworkTables) - Static mounted */
  public static final String kUpLimelightName = "limelight-up";

  /** NetworkTables key for disabling vision pose updates. */
  public static final String kApriltagDisabledKey = "ApriltagDisabled";

  // ============================================================
  // PER-CAMERA REPROJECTION ERROR (measured calibration quality)
  // Lower = better calibrated. Used to scale stdDevs proportionally.
  // ============================================================

  public static final double kFrontReprojectionError = 0.67;
  public static final double kLeftReprojectionError = 0.50;
  public static final double kUpReprojectionError = 0.47;
  public static final double kReferenceReprojectionError = 0.47;

  // ============================================================
  // CAMERA POSITIONS (Robot Space)
  // Camera positions: relative to robot center (forward +X, left +Y, up +Z)
  // ============================================================

  /** Front camera transform relative to robot center (x forward, y left, rotation) */
  public static final Transform2d kRobotToFrontCamera =
      new Transform2d(
          new Translation2d(0.2402851, -0.2318007), // 30cm forward
          Rotation2d.fromDegrees(50) // Facing forward
          );

  /** Left camera transform relative to robot center (x forward, y left, rotation) */
  public static final Transform2d kRobotToLeftCamera =
      new Transform2d(
          new Translation2d(0.2402851, 0.2318007), // 15cm forward, 25cm left
          Rotation2d.fromDegrees(-50.0));

  // ============================================================
  // TURRET 3D TRANSFORMS (Vision Kinematics)
  // ============================================================

  /** Robot center to Turret center (3D) */
  public static final edu.wpi.first.math.geometry.Transform3d kRobotToTurretCenter =
      new edu.wpi.first.math.geometry.Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(0.2302, 0.1552, 0.0),
          new edu.wpi.first.math.geometry.Rotation3d());

  /** Turret center to Right Camera (3D) */
  public static final edu.wpi.first.math.geometry.Transform3d kTurretCenterToRightCamera =
      new edu.wpi.first.math.geometry.Transform3d(
          new edu.wpi.first.math.geometry.Translation3d(-0.5, 0.0, 0.60),
          new edu.wpi.first.math.geometry.Rotation3d(0.0, Math.toRadians(-20.0), 0.0));

  // ============================================================
  // VISION ACCEPTANCE THRESHOLDS (Reference: 254 2025)
  // ============================================================

  /** Max trust distance for single tag (meters) - single tag estimates beyond this are rejected */
  public static final double kMaxSingleTagDistanceMeters = 1.8;

  /** Max trust distance for multi-tag (meters) */
  public static final double kMaxMultiTagDistanceMeters = 2.5;

  /** Min tag area (% of image 0-100) - tags below this are ignored */
  public static final double kMinTagArea = 0.02;

  /** Min tag area for single-tag MegaTag estimate (254 uses 1.0; 0.2 ≈ accept up to ~4m) */
  public static final double kTagMinAreaForSingleTagMegatag = 0.2;

  /** Tag area threshold that triggers yaw check (254 uses 2.0) */
  public static final double kTagAreaThresholdForYawCheck = 0.5;

  /** Default ambiguity threshold - single tag estimates above this are rejected (254 uses 0.19) */
  public static final double kDefaultAmbiguityThreshold = 0.20;

  /**
   * Default yaw difference threshold (degrees) - rejected if vision yaw differs too much from
   * odometry (254 uses 5.0; 8.0 balances precision vs. acceptance rate for shooting)
   */
  public static final double kDefaultYawDiffThreshold = 8.0;

  /** Max robot Z height from vision pose (meters); rejects pose if robot appears airborne (254) */
  public static final double kMaxZHeightMeters = 0.25;

  /** Min position norm (meters); rejects pose near field origin which is almost always invalid */
  public static final double kMinPositionNormMeters = 0.5;

  /** Field length (meters) for bounds checking */
  public static final double kFieldLengthMeters = 16.54;

  /** Field width (meters) for bounds checking */
  public static final double kFieldWidthMeters = 8.21;

  /** High yaw rate lookback time (seconds) - checks max angular velocity in this time window */
  public static final double kHighYawLookbackSeconds = 0.3;

  /**
   * High yaw velocity threshold (rad/s) - vision updates rejected above this (time sync unreliable)
   */
  public static final double kHighYawVelocityRadPerSec = 5.0;

  /** Minimum tag count for high trust estimate */
  public static final int kMinTagCountForHighTrust = 2;

  // ============================================================
  // STANDARD DEVIATION CONFIGURATION
  // Dynamic Q/R adjustment basis
  // ============================================================

  /** Large variance value - used when this dimension should be ignored */
  public static final double kLargeVariance = 1e6;

  /** Single-tag theta stdDev (rad). Finite but large so the EKF transition is gradual. */
  public static final double kSingleTagThetaStdDev = 5.0;

  /**
   * Distance (m) above which single-tag stdDev is scaled up to reduce far-tag jitter. EKF trusts
   * these measurements less, smoothing pose.
   */
  public static final double kSingleTagFarStdDevDistanceThresholdMeters = 2.5;

  /**
   * Extra stdDev scale per meter above threshold for single-tag. Multiplier = 1 + (dist -
   * threshold) * this (e.g. at 4 m with threshold 2.5, scale ≈ 1.75).
   */
  public static final double kSingleTagFarStdDevScalePerMeter = 0.5;

  /** EMA alpha for per-camera stdDev smoothing (0 = no update, 1 = no smoothing) */
  public static final double kStdDevSmoothingAlpha = 0.4;

  /** Base XY standard deviation (meters) - vision measurement std dev under normal conditions */
  public static final double kBaseXYStdDev = 0.3;

  /** Base theta standard deviation (radians) */
  public static final double kBaseThetaStdDev = 0.5;

  /** XY standard deviation increase per meter of distance (linear term) */
  public static final double kXYStdDevPerMeter = 0.08;

  /** XY standard deviation increase per meter² of distance (quadratic term; PnP error physics) */
  public static final double kXYStdDevPerMeterSq = 0.02;

  /** MegaTag1 X std dev index in standardDeviations array */
  public static final int kMegatag1XStdDevIndex = 0;

  /** MegaTag1 Y std dev index in standardDeviations array */
  public static final int kMegatag1YStdDevIndex = 1;

  /** Default vision measurement std devs (normal trust) */
  public static final Matrix<N3, N1> kDefaultVisionStdDevs = VecBuilder.fill(0.7, 0.7, 0.9);

  /** High trust vision measurement std devs (multi-tag, close range) */
  public static final Matrix<N3, N1> kHighTrustVisionStdDevs = VecBuilder.fill(0.3, 0.3, 0.5);

  /** Low trust vision measurement std devs (single tag, far range) */
  public static final Matrix<N3, N1> kLowTrustVisionStdDevs = VecBuilder.fill(1.5, 1.5, 2.0);

  // ============================================================
  // ODOMETRY TRUST DETECTION THRESHOLDS
  // ============================================================

  /** Slip velocity threshold (m/s) - legacy, kept for reference */
  public static final double kSlipVelocityThresholdMps = 0.5;

  /**
   * Slip ratio threshold: |measured - setpoint| / max(|setpoint|, 0.1). Triggered when any module
   * deviates more than 30% from its commanded speed.
   */
  public static final double kSlipRatioThreshold = 0.3;

  /**
   * Impact acceleration threshold (g) - impact detected when accel magnitude exceeds this.
   * Calculation should subtract gravity (1g).
   */
  public static final double kImpactAccelThresholdG = 2.0;

  /** Odometry update timeout threshold (seconds) - triggered if no update while robot moving */
  public static final double kOdometryStaleTimeoutSeconds = 0.5;

  /** Minimum speed considered as "robot is moving" (m/s) */
  public static final double kMovementThresholdMps = 0.05;

  /** Time to reject vision updates after impact (seconds) */
  public static final double kImpactRecoveryTimeSeconds = 0.2;

  // ============================================================
  // TURRET LIMELIGHT SPECIFIC
  // ============================================================

  /** Turret Limelight effective horizontal distance range (meters) */
  public static final double kTurretLimelightMaxRangeMeters = 3.0;

  /** Turret Limelight minimum valid target area */
  public static final double kTurretLimelightMinTargetArea = 0.5;

  /** Turret Limelight lock angle tolerance (degrees) - max offset considered "locked" */
  public static final double kTurretLimelightLockAngleTolerance = 5.0;

  // ============================================================
  // POSE TRUST EVALUATION
  // ============================================================

  /** Minimum consecutive trusted cycles - required before pose is considered trusted */
  public static final int kMinConsecutiveTrustedCycles = 3;

  /** Vision update max interval (seconds) - vision considered unavailable if exceeded */
  public static final double kVisionUpdateMaxIntervalSeconds = 0.5;

  /** Pose trust distance threshold (meters) - for comparison with known position */
  public static final double kPoseTrustDistanceThresholdMeters = 0.5;

  /**
   * Odometry drift rate per meter travelled. Swerve odometry typically drifts 1-3% of distance.
   * This factor converts distance driven into estimated position error.
   */
  public static final double kOdometryDriftRatePerMeter = 0.10;

  /**
   * Max acceptable odometry drift (meters) before pose trust is revoked. At 2% drift rate, this
   * allows ~15m of travel without vision before the robot stops auto-shooting. Hub scoring
   * tolerance is roughly 30cm.
   */
  public static final double kMaxOdometryDriftMeters = 0.30;

  // ============================================================
  // AUTO-SHOOT TRUST EVALUATION
  // Continuous trust score (0–1) for autonomous shooting decisions.
  // ============================================================

  /** Trust score threshold for allowing auto-shoot */
  public static final double kTrustScoreForAutoShoot = 0.70;

  /** Smoothing factor (EMA alpha) for trust score updates */
  public static final double kTrustScoreAlpha = 0.12;

  /** Max pose jitter (meters) between consecutive cycles to be considered "stable" */
  public static final double kMaxPoseJitterMeters = 0.15;

  /** Number of recent poses kept for stability evaluation */
  public static final int kPoseStabilityWindowSize = 10;

  /** Min filled slots in the stability window before stability can be evaluated */
  public static final int kPoseStabilityMinSamples = 5;

  /** Max innovation (meters) before vision stdDevs are inflated to protect the EKF */
  public static final double kMaxInnovationMeters = 1.0;

  /** Innovation scaling exponent: stdDevs *= (innovation / kMaxInnovation)^exponent */
  public static final double kInnovationScalingExponent = 1.5;

  // ============================================================
  // GYRO RESET FROM VISION (Orbit2: reset gyro when close / trusted)
  // ============================================================

  /**
   * Max distance (m) for single-tag gyro reset. When closer than this and heading is trusted we may
   * set Pigeon yaw from vision.
   */
  public static final double kGyroResetMaxDistanceMeters = 2.0;

  /** Min tag count to allow gyro reset (multi-tag = trust heading). */
  public static final int kGyroResetMinTagCount = 2;

  /** When single-tag, min avg tag area to allow gyro reset. */
  public static final double kGyroResetMinTagArea = 0.08;

  /** Debounce: min seconds between gyro resets to avoid spamming Pigeon. */
  public static final double kGyroResetDebounceSeconds = 0.5;

  /** Enable gyro reset from vision. Set to false to prevent vision from adjusting gyro hardware. */
  public static final boolean kEnableGyroReset = false;

  /** Force vision pose estimates to use historical gyro heading. */
  public static final boolean kForceGyroHeading = false;

  // ============================================================
  // DYNAMIC PIPELINE & CROP (Orbit2: pipeline 0 = search, 1 = track + crop)
  // ============================================================

  /** Pipeline index: high resolution, wide FOV, 120 fps (use when no target). */
  public static final int kPipelineSearch = 0;

  /** Pipeline index: 640x480, 240 fps (use when target visible for dynamic crop). */
  public static final int kPipelineTrack = 1;

  /** Crop half-size horizontal in normalized coords (0-1). Wider to capture neighbors. */
  public static final double kCropHalfSizeHoriz = 0.35;

  /** Crop half-size vertical in normalized coords (0-1). */
  public static final double kCropHalfSizeVert = 0.28;

  /** Extra crop margin per ms pipeline latency (target may have moved). */
  public static final double kCropMarginPerMsLatency = 0.001;

  /** Horizontal FOV (deg) for tx -> normalized X. */
  public static final double kFovHorizDeg = 60.0;

  /** Vertical FOV (deg) for ty -> normalized Y. */
  public static final double kFovVertDeg = 49.0;

  /** Max field-space distance (m) to search for neighbor tags when expanding the crop. */
  public static final double kNeighborSearchRadiusMeters = 2.0;

  /** Extra margin (deg) around a predicted neighbor tag position in the crop window. */
  public static final double kCropNeighborMarginDeg = 5.0;

  private VisionConstants() {
    // Prevent instantiation
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
