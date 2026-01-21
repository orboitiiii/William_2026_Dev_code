package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * Global robot state repository (Blackboard Pattern).
 *
 * <p>This singleton class serves as the central data exchange for subsystems, storing the robot's
 * estimated pose history for time-synchronized queries. This is essential for vision latency
 * compensation where measurements arrive 50-200ms after capture.
 *
 * <p><strong>Architecture</strong>: Based on Team 254's RobotState pattern.
 *
 * <p><strong>Thread Safety</strong>: All public methods are synchronized to allow safe access from
 * multiple threads (main loop, vision processing, autonomous).
 *
 * <p><strong>Coordinate Frame</strong>: All poses are in the field coordinate system (origin at
 * field corner, +X toward opposing alliance, +Y to the left, +θ CCW).
 *
 * @see InterpolatingTreeMap
 */
public class RobotState {
  private static RobotState mInstance;

  /**
   * Returns the singleton instance of RobotState.
   *
   * @return The global RobotState instance.
   */
  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  /**
   * Time-indexed pose history for interpolation.
   *
   * <p>Key: FPGA timestamp (seconds). Value: Field-to-Vehicle pose.
   */
  private final InterpolatingTreeMap<Double, Pose2d> mFieldToVehicleMap;

  /** Cached reference to the most recent pose for fast access. */
  private Pose2d mLatestFieldToVehicle = new Pose2d();

  private RobotState() {
    mFieldToVehicleMap =
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), (start, end, t) -> start.interpolate(end, t));
    reset(0.0, new Pose2d());
  }

  /**
   * Resets the pose history to a known initial state.
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
  }

  /**
   * Queries the robot pose at a specific timestamp using interpolation.
   *
   * <p>If the timestamp falls between two observations, linear interpolation is applied to both
   * translation and rotation components.
   *
   * @param timestamp The FPGA timestamp to query.
   * @return The interpolated pose at the requested time.
   */
  public synchronized Pose2d getFieldToVehicle(double timestamp) {
    return mFieldToVehicleMap.get(timestamp);
  }

  /**
   * Returns the most recent pose observation.
   *
   * <p>This is faster than {@link #getFieldToVehicle(double)} when historical data is not needed.
   *
   * @return The latest field-to-vehicle pose.
   */
  public synchronized Pose2d getLatestFieldToVehicle() {
    return mLatestFieldToVehicle;
  }

  /**
   * Adds a new pose observation to the history.
   *
   * <p>Called by the Drive subsystem's estimator (ESUKF) every control cycle.
   *
   * <p><strong>Memory Note</strong>: The InterpolatingTreeMap does not automatically prune old
   * entries. Long matches may accumulate significant memory. Consider implementing a sliding window
   * if memory becomes constrained.
   *
   * @param timestamp The FPGA timestamp of the observation.
   * @param pose The observed robot pose in field coordinates.
   */
  public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d pose) {
    mFieldToVehicleMap.put(timestamp, pose);
    mLatestFieldToVehicle = pose;
    // TODO: Implement pruning to prevent memory leak
  }
}
