package frc.robot.auto.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Immutable container for a time-parameterized trajectory.
 *
 * <p>A trajectory is a sequence of {@link TrajectoryPoint}s representing the robot's desired state
 * (pose + velocity) at each timestep. This class provides time-based sampling with interpolation.
 *
 * <p><strong>Design Principles</strong>:
 *
 * <ul>
 *   <li><strong>Immutability</strong>: Once created, the trajectory cannot be modified.
 *   <li><strong>O(log n) Lookup</strong>: Binary search for efficient time-based sampling.
 *   <li><strong>Boundary Safety</strong>: Queries outside the time range return endpoint values.
 * </ul>
 *
 * <p><strong>Interpolation Method</strong>: Linear interpolation for pose and velocity components
 * between the two nearest waypoints.
 *
 * @see TrajectoryPoint
 * @see TrajectoryReader
 */
public class Trajectory {
  private final List<TrajectoryPoint> mPoints;
  private final double mTotalTimeSeconds;

  /**
   * Private constructor for trajectory creation.
   *
   * @param points List of trajectory points (must be sorted by time).
   * @throws IllegalArgumentException If points is null or empty.
   */
  private Trajectory(List<TrajectoryPoint> points) {
    if (points == null || points.isEmpty()) {
      throw new IllegalArgumentException("Trajectory must have at least one point");
    }
    this.mPoints = Collections.unmodifiableList(new ArrayList<>(points));
    this.mTotalTimeSeconds = mPoints.get(mPoints.size() - 1).timeSeconds();
  }

  /**
   * Creates a trajectory from a list of points.
   *
   * @param points Trajectory points, sorted by ascending time.
   * @return A new immutable Trajectory instance.
   */
  public static Trajectory fromPoints(List<TrajectoryPoint> points) {
    return new Trajectory(points);
  }

  /**
   * Samples the trajectory at a specific time using linear interpolation.
   *
   * <p>If the time is before the start, returns the first point. If the time is after the end,
   * returns the last point.
   *
   * <p><strong>Complexity</strong>: O(log n) using binary search.
   *
   * @param timeSeconds Query time relative to trajectory start.
   * @return The interpolated trajectory point.
   */
  public TrajectoryPoint sample(double timeSeconds) {
    // Boundary conditions
    if (timeSeconds <= 0) {
      return mPoints.get(0);
    }
    if (timeSeconds >= mTotalTimeSeconds) {
      return mPoints.get(mPoints.size() - 1);
    }

    // Binary search for the bracketing interval
    int low = 0;
    int high = mPoints.size() - 1;

    while (high - low > 1) {
      int mid = (low + high) / 2;
      if (mPoints.get(mid).timeSeconds() <= timeSeconds) {
        low = mid;
      } else {
        high = mid;
      }
    }

    // Linear interpolation within [low, high]
    TrajectoryPoint start = mPoints.get(low);
    TrajectoryPoint end = mPoints.get(high);

    double t = (timeSeconds - start.timeSeconds()) / (end.timeSeconds() - start.timeSeconds());

    return start.interpolate(end, t);
  }

  /**
   * Returns the total trajectory duration.
   *
   * @return Duration in seconds.
   */
  public double getTotalTimeSeconds() {
    return mTotalTimeSeconds;
  }

  /**
   * Checks whether a given time exceeds the trajectory end.
   *
   * @param timeSeconds Query time.
   * @return True if the trajectory is complete.
   */
  public boolean isFinished(double timeSeconds) {
    return timeSeconds >= mTotalTimeSeconds;
  }

  /**
   * Returns the starting pose of the trajectory.
   *
   * @return Initial robot pose in field coordinates.
   */
  public Pose2d getInitialPose() {
    return mPoints.get(0).pose();
  }

  /**
   * Returns the ending pose of the trajectory.
   *
   * @return Final robot pose in field coordinates.
   */
  public Pose2d getFinalPose() {
    return mPoints.get(mPoints.size() - 1).pose();
  }

  /**
   * Returns the number of waypoints in the trajectory.
   *
   * @return Point count.
   */
  public int size() {
    return mPoints.size();
  }

  /**
   * Returns an unmodifiable view of the trajectory points.
   *
   * @return Immutable list of trajectory points.
   */
  /**
   * Returns the timestamp of the n-th point from the end.
   *
   * @param nPointsFromEnd Number of points back from the end (e.g., 3 means 3rd from last).
   * @return Timestamp in seconds. Returns 0 if trajectory is too short.
   */
  public double getTimeAtLastPointMinus(int nPointsFromEnd) {
    if (mPoints.size() <= nPointsFromEnd) {
      return 0.0;
    }
    return mPoints.get(mPoints.size() - 1 - nPointsFromEnd).timeSeconds();
  }

  public List<TrajectoryPoint> getPoints() {
    return mPoints;
  }
}
