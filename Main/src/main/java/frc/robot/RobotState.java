package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

/**
 * RobotState to hold global robot state (odometry, game pieces, etc.).
 *
 * <p>Acts as a central blackboard for subsystems to exchange data. Updated to use
 * InterpolatingTreeMap for historical queries (Vision latency compensation).
 *
 * @author Team 254 (Concept)
 */
public class RobotState {
  private static RobotState mInstance;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  // Maps timestamp (seconds) to Pose2d
  private final InterpolatingTreeMap<Double, Pose2d> mFieldToVehicleMap;

  // Keep a reference to the latest for quick access
  private Pose2d mLatestFieldToVehicle = new Pose2d();
  private final double kObservationBufferSize = 1.5; // Seconds of history to keep

  private RobotState() {
    // Configured to interpolate Doubles (keys) and Pose2d (values)
    mFieldToVehicleMap =
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), (start, end, t) -> start.interpolate(end, t));
    reset(0.0, new Pose2d());
  }

  public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
    mFieldToVehicleMap.clear();
    mFieldToVehicleMap.put(start_time, initial_field_to_vehicle);
    mLatestFieldToVehicle = initial_field_to_vehicle;
  }

  public synchronized Pose2d getFieldToVehicle(double timestamp) {
    return mFieldToVehicleMap.get(timestamp);
  }

  public synchronized Pose2d getLatestFieldToVehicle() {
    return mLatestFieldToVehicle;
  }

  public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d pose) {
    mFieldToVehicleMap.put(timestamp, pose);
    mLatestFieldToVehicle = pose;
  }
}
