package frc.robot.libraries.lib9427;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Java Wrapper for Native Error-State UKF. Implements typical IMU + Vision fusion logic with
 * Mahalanobis Gating.
 */
public class ESUKF implements AutoCloseable {
  private long nativeHandle;
  private final double[] m_R = new double[9]; // Reusable R matrix buffer

  public ESUKF() {
    nativeHandle = Team9427JNI.createESUKF();
  }

  /** Initialize the filter at a specific pose. */
  public void initialize(Pose3d initPose) {
    Quaternion q = initPose.getRotation().getQuaternion();
    Team9427JNI.initESUKF(
        nativeHandle,
        initPose.getX(),
        initPose.getY(),
        initPose.getZ(),
        q.getW(),
        q.getX(),
        q.getY(),
        q.getZ());
  }

  /**
   * Prediction Step using IMU data. Coordinate Frame: Should match the robot/vision frame
   * (typically NWU or ENU).
   *
   * @param accX Accel X (m/s^2)
   * @param accY Accel Y (m/s^2)
   * @param accZ Accel Z (m/s^2)
   * @param gyroX Gyro X (rad/s)
   * @param gyroY Gyro Y (rad/s)
   * @param gyroZ Gyro Z (rad/s)
   * @param dt Time step (seconds)
   */
  public void predict(
      double accX, double accY, double accZ, double gyroX, double gyroY, double gyroZ, double dt) {
    Team9427JNI.predictESUKF(nativeHandle, accX, accY, accZ, gyroX, gyroY, gyroZ, dt);
  }

  /**
   * Measurement Update Step with outlier rejection. Note: Currently only processes Position (3D).
   * Orientation updates are handled implicitly via IMU or could be added.
   *
   * @param measurement Measurement Pose
   * @param posStdDevs Standard deviation of position (x, y, z)
   * @param gateThreshold Chi-square threshold (e.g., 7.8 for 3 DOF 95%, 3.0 stricter)
   * @return true if accepted, false if rejected (outlier)
   */
  public boolean correct(Pose3d measurement, double[] posStdDevs, double gateThreshold) {
    if (posStdDevs.length != 3)
      throw new IllegalArgumentException("StdDevs must be size 3 (x, y, z)");

    // Populate Diagonal R matrix (assuming independent axes)
    // Row 0
    m_R[0] = posStdDevs[0] * posStdDevs[0];
    m_R[1] = 0;
    m_R[2] = 0;
    // Row 1
    m_R[3] = 0;
    m_R[4] = posStdDevs[1] * posStdDevs[1];
    m_R[5] = 0;
    // Row 2
    m_R[6] = 0;
    m_R[7] = 0;
    m_R[8] = posStdDevs[2] * posStdDevs[2];

    return Team9427JNI.correctESUKF(
        nativeHandle,
        measurement.getX(),
        measurement.getY(),
        measurement.getZ(),
        m_R,
        gateThreshold);
  }

  /** Get the current estimated Pose. */
  public Pose3d getPose() {
    double[] state = Team9427JNI.getESUKFState(nativeHandle);
    // [0-2] Pos, [3-5] Vel, [6-9] Quat (w, x, y, z)
    return new Pose3d(
        new Translation3d(state[0], state[1], state[2]),
        new Rotation3d(new Quaternion(state[6], state[7], state[8], state[9])));
  }

  /** Get current estimated Velocity. */
  public Translation3d getVelocity() {
    double[] state = Team9427JNI.getESUKFState(nativeHandle);
    return new Translation3d(state[3], state[4], state[5]);
  }

  // --- Pose2d Convenience Wrappers ---

  /** Get the current estimated Pose converted to Pose2d (ignoring Z, Roll, Pitch). */
  public Pose2d getPose2d() {
    Pose3d p3d = getPose();
    return p3d.toPose2d();
  }

  /** Initialize using Pose2d (assumes Z=0, flat on ground). */
  public void initialize(Pose2d initPose) {
    initialize(new Pose3d(initPose));
  }

  /**
   * Measurement Update using Pose2d (Vision/Odometry). Automatically converts to 3D (Z=0) for the
   * filter.
   *
   * @param measurement Measurement Pose2d
   * @param posStdDevs Standard deviation of position (x, y) - size 2
   * @param gateThreshold Chi-square threshold
   * @return true if accepted, false if rejected
   */
  public boolean correct(
      edu.wpi.first.math.geometry.Pose2d measurement, double[] posStdDevs, double gateThreshold) {
    if (posStdDevs.length != 2)
      throw new IllegalArgumentException("For Pose2d, StdDevs must be size 2 (x, y)");

    // Map 2D std devs to 3D: [x, y, z_fixed]
    // We give Z a moderate value since we are clamping it to 0
    double[] stdDevs3d = new double[] {posStdDevs[0], posStdDevs[1], 0.1};

    return correct(new Pose3d(measurement), stdDevs3d, gateThreshold);
  }

  @Override
  public void close() {
    if (nativeHandle != 0) {
      Team9427JNI.deleteESUKF(nativeHandle);
      nativeHandle = 0;
    }
  }
}
