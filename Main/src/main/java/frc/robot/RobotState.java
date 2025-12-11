package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotState to hold global robot state (odometry, game pieces, etc.).
 *
 * <p>Acts as a central blackboard for subsystems to exchange data.
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

  // Example Fields
  private Pose2d mFieldToVehicle = new Pose2d();
  private Rotation2d mHeading = new Rotation2d();

  private RobotState() {
    reset(0.0, new Pose2d());
  }

  public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
    mFieldToVehicle = initial_field_to_vehicle;
    mHeading = initial_field_to_vehicle.getRotation();
  }

  public synchronized Pose2d getFieldToVehicle(double timestamp) {
    // In a real implementation, this would interpolate based on timestamp
    return mFieldToVehicle;
  }

  public synchronized void setFieldToVehicle(Pose2d pose) {
    mFieldToVehicle = pose;
  }

  public synchronized void outputToSmartDashboard() {
    SmartDashboard.putString("robot_pose", mFieldToVehicle.toString());
    SmartDashboard.putNumber("robot_heading", mHeading.getDegrees());
  }
}
