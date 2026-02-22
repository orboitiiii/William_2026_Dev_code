package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import java.util.Optional;
import java.util.TreeMap;
import java.util.function.Consumer;

/**
 * Global robot state repository (Blackboard Pattern).
 *
 * <p>This singleton class serves as the central data exchange for subsystems, storing the robot's
 * estimated pose, velocity, and IMU history for time-synchronized queries. This is essential for:
 *
 * <ul>
 *   <li><strong>Vision Latency Compensation</strong>: Measurements arrive 50-200ms after capture.
 *   <li><strong>Shoot-On-Move</strong>: Velocity history enables projectile lead calculations.
 *   <li><strong>Sensor Fusion</strong>: Angular velocity tracking for vision rejection logic.
 * </ul>
 *
 * <p><strong>Architecture</strong>: Based on FRC 254 2025 PoseHistory pattern with extensions for
 * angular velocity tracking and vision estimate management.
 *
 * <p><strong>Thread Safety</strong>: All public methods are synchronized to allow safe access from
 * multiple threads (main loop, vision processing, autonomous).
 *
 * <p><strong>Coordinate Frame</strong>: All poses and velocities are in the field coordinate system
 * (origin at field corner, +X toward opposing alliance, +Y to the left, +θ CCW).
 *
 * @see TreeMap
 */
public class PoseHistory {
  private static PoseHistory mInstance;

  /**
   * Returns the singleton instance of PoseHistory.
   *
   * @return The global PoseHistory instance.
   */
  public static PoseHistory getInstance() {
    if (mInstance == null) {
      mInstance = new PoseHistory();
    }
    return mInstance;
  }

  /** Lookback time for interpolation buffers (seconds). */
  public static final double LOOKBACK_TIME = 1.0;

  // ============================================================
  // POSE AND VELOCITY HISTORY
  // ============================================================

  /**
   * Time-indexed pose history for interpolation.
   *
   * <p>Key: FPGA timestamp (seconds). Value: Field-to-Vehicle pose.
   */
  private final TreeMap<Double, Pose2d> mFieldToVehicleMap = new TreeMap<>();

  /**
   * Time-indexed velocity history for interpolation.
   *
   * <p>Key: FPGA timestamp (seconds). Value: Field-relative ChassisSpeeds.
   */
  private final TreeMap<Double, ChassisSpeeds> mFieldVelocityMap = new TreeMap<>();

  // ============================================================
  // ANGULAR VELOCITY HISTORY (254 Pattern)
  // ============================================================

  /** Time-indexed yaw angular velocity history (rad/s). */
  private final TreeMap<Double, Double> mYawAngularVelocityMap = new TreeMap<>();

  /** Time-indexed pitch angular velocity history (rad/s). */
  private final TreeMap<Double, Double> mPitchAngularVelocityMap = new TreeMap<>();

  /** Time-indexed roll angular velocity history (rad/s). */
  private final TreeMap<Double, Double> mRollAngularVelocityMap = new TreeMap<>();

  // ============================================================
  // ACCELERATION HISTORY (for impact detection)
  // ============================================================

  /** Time-indexed X acceleration history (m/s²). */
  private final TreeMap<Double, Double> mAccelXMap = new TreeMap<>();

  /** Time-indexed Y acceleration history (m/s²). */
  private final TreeMap<Double, Double> mAccelYMap = new TreeMap<>();

  // ============================================================
  // CACHED LATEST VALUES (fast access)
  // ============================================================

  private Pose2d mLatestFieldToVehicle = new Pose2d();
  private ChassisSpeeds mLatestFieldVelocity = new ChassisSpeeds();
  private double mLatestYawRate = 0.0;
  private double[] mLatestAccel = new double[] {0.0, 0.0, 0.0};

  // ============================================================
  // VISION STATE (254 Pattern)
  // ============================================================

  /** Timestamp of the last used MegaTag estimate. */
  private double mLastUsedMegatagTimestamp = 0;

  /** Pose from the last used MegaTag estimate. */
  private Pose2d mLastUsedMegatagPose = new Pose2d();

  /** Consumer for vision estimates (typically SwerveDrivePoseEstimator). */
  private Consumer<VisionFieldPoseEstimate> mVisionEstimateConsumer;

  private PoseHistory() {
    reset(0.0, new Pose2d());
  }

  // ============================================================
  // CONFIGURATION
  // ============================================================

  /**
   * Sets the vision estimate consumer.
   *
   * <p>Called during robot initialization to wire VisionSubsystem to RobotStateEstimator.
   *
   * @param consumer Consumer that receives vision estimates.
   */
  public synchronized void setVisionEstimateConsumer(Consumer<VisionFieldPoseEstimate> consumer) {
    mVisionEstimateConsumer = consumer;
  }

  // ============================================================
  // RESET
  // ============================================================

  /**
   * Resets the pose and velocity history to a known initial state.
   *
   * <p>Call this at the start of autonomous or when resetting odometry.
   *
   * @param startTime The FPGA timestamp for the initial observation.
   * @param initialFieldToVehicle The initial robot pose in field coordinates.
   */
  public synchronized void reset(double startTime, Pose2d initialFieldToVehicle) {
    mFieldToVehicleMap.clear();
    mFieldToVehicleMap.put(startTime, initialFieldToVehicle);
    mLatestFieldToVehicle = initialFieldToVehicle;

    mFieldVelocityMap.clear();
    mFieldVelocityMap.put(startTime, new ChassisSpeeds());
    mLatestFieldVelocity = new ChassisSpeeds();

    mYawAngularVelocityMap.clear();
    mYawAngularVelocityMap.put(startTime, 0.0);
    mPitchAngularVelocityMap.clear();
    mPitchAngularVelocityMap.put(startTime, 0.0);
    mRollAngularVelocityMap.clear();
    mRollAngularVelocityMap.put(startTime, 0.0);

    mAccelXMap.clear();
    mAccelXMap.put(startTime, 0.0);
    mAccelYMap.clear();
    mAccelYMap.put(startTime, 0.0);

    mLastUsedMegatagTimestamp = 0;
    mLastUsedMegatagPose = new Pose2d();
  }

  // ============================================================
  // POSE QUERIES
  // ============================================================

  /**
   * Queries the robot pose at a specific timestamp using interpolation.
   *
   * @param timestamp The FPGA timestamp to query.
   * @return The interpolated pose at the requested time, or empty if not available.
   */
  public synchronized Optional<Pose2d> getFieldToRobot(double timestamp) {
    if (mFieldToVehicleMap.isEmpty()) return Optional.empty();

    var floorEntry = mFieldToVehicleMap.floorEntry(timestamp);
    var ceilingEntry = mFieldToVehicleMap.ceilingEntry(timestamp);

    if (floorEntry == null && ceilingEntry == null) return Optional.empty();
    if (floorEntry == null) return Optional.of(ceilingEntry.getValue());
    if (ceilingEntry == null) return Optional.of(floorEntry.getValue());
    if (floorEntry.getKey().equals(ceilingEntry.getKey()))
      return Optional.of(floorEntry.getValue());

    double t = (timestamp - floorEntry.getKey()) / (ceilingEntry.getKey() - floorEntry.getKey());
    return Optional.of(floorEntry.getValue().interpolate(ceilingEntry.getValue(), t));
  }

  /**
   * Returns the most recent pose observation.
   *
   * @return The latest field-to-vehicle pose.
   */
  public synchronized Pose2d getLatestFieldToVehicle() {
    return mLatestFieldToVehicle;
  }

  // ============================================================
  // VELOCITY QUERIES
  // ============================================================

  /**
   * Queries the robot velocity at a specific timestamp using interpolation.
   *
   * @param timestamp The FPGA timestamp to query.
   * @return The interpolated field-relative velocity at the requested time.
   */
  public synchronized ChassisSpeeds getFieldVelocity(double timestamp) {
    if (mFieldVelocityMap.isEmpty()) return new ChassisSpeeds();

    var floorEntry = mFieldVelocityMap.floorEntry(timestamp);
    var ceilingEntry = mFieldVelocityMap.ceilingEntry(timestamp);

    if (floorEntry == null && ceilingEntry == null) return new ChassisSpeeds();
    if (floorEntry == null) return ceilingEntry.getValue();
    if (ceilingEntry == null) return floorEntry.getValue();
    if (floorEntry.getKey().equals(ceilingEntry.getKey())) return floorEntry.getValue();

    double t = (timestamp - floorEntry.getKey()) / (ceilingEntry.getKey() - floorEntry.getKey());
    var start = floorEntry.getValue();
    var end = ceilingEntry.getValue();
    return new ChassisSpeeds(
        MathUtil.interpolate(start.vxMetersPerSecond, end.vxMetersPerSecond, t),
        MathUtil.interpolate(start.vyMetersPerSecond, end.vyMetersPerSecond, t),
        MathUtil.interpolate(start.omegaRadiansPerSecond, end.omegaRadiansPerSecond, t));
  }

  /**
   * Returns the most recent velocity observation.
   *
   * @return The latest field-relative velocity.
   */
  public synchronized ChassisSpeeds getLatestFieldVelocity() {
    return mLatestFieldVelocity;
  }

  // ============================================================
  // ANGULAR VELOCITY QUERIES (254 Pattern)
  // ============================================================

  /**
   * Returns the maximum absolute yaw angular velocity in a time range.
   *
   * <p>Used for vision rejection logic - high angular velocity means time synchronization is
   * unreliable.
   *
   * @param minTime Start of time range (inclusive).
   * @param maxTime End of time range (inclusive).
   * @return Maximum absolute yaw rate in the range, or empty if no data.
   */
  public synchronized Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(
      double minTime, double maxTime) {
    return getMaxAbsValueInRange(mYawAngularVelocityMap, minTime, maxTime);
  }

  /**
   * Returns the maximum absolute pitch angular velocity in a time range.
   *
   * @param minTime Start of time range (inclusive).
   * @param maxTime End of time range (inclusive).
   * @return Maximum absolute pitch rate in the range, or empty if no data.
   */
  public synchronized Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(
      double minTime, double maxTime) {
    return getMaxAbsValueInRange(mPitchAngularVelocityMap, minTime, maxTime);
  }

  /**
   * Returns the maximum absolute roll angular velocity in a time range.
   *
   * @param minTime Start of time range (inclusive).
   * @param maxTime End of time range (inclusive).
   * @return Maximum absolute roll rate in the range, or empty if no data.
   */
  public synchronized Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(
      double minTime, double maxTime) {
    return getMaxAbsValueInRange(mRollAngularVelocityMap, minTime, maxTime);
  }

  /** Returns the latest yaw angular velocity. */
  public synchronized double getLatestYawRate() {
    return mLatestYawRate;
  }

  /** Returns the latest acceleration [x, y, z] in m/s². */
  public synchronized double[] getLatestAcceleration() {
    return mLatestAccel.clone();
  }

  // ============================================================
  // DATA INPUT METHODS
  // ============================================================

  /**
   * Adds a new pose observation to the history.
   *
   * <p>Called by the Drive subsystem every control cycle.
   *
   * @param timestamp The FPGA timestamp of the observation.
   * @param pose The observed robot pose in field coordinates.
   */
  public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d pose) {
    mFieldToVehicleMap.put(timestamp, pose);
    mLatestFieldToVehicle = pose;
    prune(timestamp);
  }

  /**
   * Adds a new velocity observation to the history.
   *
   * @param timestamp The FPGA timestamp of the observation.
   * @param velocity The field-relative velocity.
   */
  public synchronized void addFieldVelocityObservation(double timestamp, ChassisSpeeds velocity) {
    mFieldVelocityMap.put(timestamp, velocity);
    mLatestFieldVelocity = velocity;
  }

  /**
   * Adds drive motion measurements from IMU (254 pattern).
   *
   * <p>Called by Drive subsystem every control cycle to publish all IMU data.
   *
   * @param timestamp FPGA timestamp.
   * @param yawRateRadPerSec Yaw angular velocity in rad/s.
   * @param pitchRateRadPerSec Pitch angular velocity in rad/s.
   * @param rollRateRadPerSec Roll angular velocity in rad/s.
   * @param accelX X acceleration in m/s².
   * @param accelY Y acceleration in m/s².
   * @param accelZ Z acceleration in m/s².
   */
  public synchronized void addDriveMotionMeasurements(
      double timestamp,
      double yawRateRadPerSec,
      double pitchRateRadPerSec,
      double rollRateRadPerSec,
      double accelX,
      double accelY,
      double accelZ) {
    mYawAngularVelocityMap.put(timestamp, yawRateRadPerSec);
    mPitchAngularVelocityMap.put(timestamp, pitchRateRadPerSec);
    mRollAngularVelocityMap.put(timestamp, rollRateRadPerSec);
    mAccelXMap.put(timestamp, accelX);
    mAccelYMap.put(timestamp, accelY);

    mLatestYawRate = yawRateRadPerSec;
    mLatestAccel[0] = accelX;
    mLatestAccel[1] = accelY;
    mLatestAccel[2] = accelZ;

    prune(timestamp);
  }

  // ============================================================
  // VISION ESTIMATE MANAGEMENT (254 Pattern)
  // ============================================================

  /**
   * Updates with a new MegaTag vision estimate.
   *
   * <p>Called by VisionSubsystem when a valid estimate is accepted.
   *
   * @param estimate The accepted vision estimate.
   */
  public synchronized void updateMegatagEstimate(VisionFieldPoseEstimate estimate) {
    mLastUsedMegatagTimestamp = estimate.getTimestampSeconds();
    mLastUsedMegatagPose = estimate.getVisionRobotPoseMeters();

    if (mVisionEstimateConsumer != null) {
      mVisionEstimateConsumer.accept(estimate);
    }
  }

  /**
   * @return Timestamp of the last used MegaTag estimate.
   */
  public synchronized double lastUsedMegatagTimestamp() {
    return mLastUsedMegatagTimestamp;
  }

  /**
   * @return Pose from the last used MegaTag estimate.
   */
  public synchronized Pose2d lastUsedMegatagPose() {
    return mLastUsedMegatagPose;
  }

  // ============================================================
  // HELPER METHODS
  // ============================================================

  /**
   * Gets the max absolute value in a time range from an interpolating map.
   *
   * <p>Uses the internal TreeMap's subMap for efficient range queries.
   */
  private Optional<Double> getMaxAbsValueInRange(
      TreeMap<Double, Double> map, double minTime, double maxTime) {
    var subMap = map.subMap(minTime, true, maxTime, true);
    if (subMap.isEmpty()) {
      return Optional.empty();
    }

    double maxAbs = 0.0;
    for (double val : subMap.values()) {
      maxAbs = Math.max(maxAbs, Math.abs(val));
    }

    return Optional.of(maxAbs);
  }

  /**
   * Prunes old data from all interpolating maps to prevent memory leaks and maintain O(log N)
   * performance.
   *
   * @param currentTime Current FPGA timestamp.
   */
  private void prune(double currentTime) {
    double threshold = currentTime - LOOKBACK_TIME;

    mFieldToVehicleMap.headMap(threshold).clear();
    mFieldVelocityMap.headMap(threshold).clear();
    mYawAngularVelocityMap.headMap(threshold).clear();
    mPitchAngularVelocityMap.headMap(threshold).clear();
    mRollAngularVelocityMap.headMap(threshold).clear();
    mAccelXMap.headMap(threshold).clear();
    mAccelYMap.headMap(threshold).clear();
  }
}
