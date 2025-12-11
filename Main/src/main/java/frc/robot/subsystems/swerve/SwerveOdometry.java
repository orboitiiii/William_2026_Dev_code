package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/**
 * Custom Swerve Odometry (1690/254 Pure Implementation). Features: 1. Midpoint Integration (Gyro)
 * 2. Physics-Based Gating (Rigidity Check/Skid Detection)
 */
public class SwerveOdometry {
  private final SwerveKinematics mKinematics;
  private Pose2d mPose;
  private Rotation2d mLastGyroAngle;
  private double[] mLastModuleRotations; // In meters

  // Module locations for Rigidity Check
  private final Translation2d[] mModuleLocations;

  public SwerveOdometry(
      SwerveKinematics kinematics,
      Pose2d initialPose,
      Rotation2d initialGyroAngle,
      double[] currentModulePositions) {
    mKinematics = kinematics;
    mPose = initialPose;
    mLastGyroAngle = initialGyroAngle;
    mLastModuleRotations = currentModulePositions.clone();

    mModuleLocations =
        new Translation2d[] {
          new Translation2d(Constants.Swerve.kWheelBase / 2.0, Constants.Swerve.kTrackWidth / 2.0),
          new Translation2d(Constants.Swerve.kWheelBase / 2.0, -Constants.Swerve.kTrackWidth / 2.0),
          new Translation2d(-Constants.Swerve.kWheelBase / 2.0, Constants.Swerve.kTrackWidth / 2.0),
          new Translation2d(-Constants.Swerve.kWheelBase / 2.0, -Constants.Swerve.kTrackWidth / 2.0)
        };
  }

  public synchronized Pose2d update(
      Rotation2d currentGyroAngle,
      double[] currentModulePositions,
      Rotation2d[] currentModuleAngles,
      double[] currentModuleVelocities,
      double gyroRateRadPerS) {
    // Calculate Deltas
    double[] moduleDeltas = new double[4];
    for (int i = 0; i < 4; i++) {
      moduleDeltas[i] = currentModulePositions[i] - mLastModuleRotations[i];
      mLastModuleRotations[i] = currentModulePositions[i];
    }

    // --- Step 2.1: Rigidity Check (Physics Gating) ---
    // Verify if the robot is behaving like a rigid body (no skid).
    // V_est_i = V_meas_i - (Omega x R_i)
    // If Variance(V_est) is high -> Skid.
    double rigidityMetric =
        calculateRigidityMetric(currentModuleVelocities, currentModuleAngles, gyroRateRadPerS);

    // Gating Logic
    // If Metric > Threshold, skip odometry update (or de-weight it heavily).
    // Also check "Collision" via IMU (assumed external check or passed in... simplified here).
    // For strict 1690: "If std dev > THRESHOLD, FOM = infinity (discard)".

    if (rigidityMetric > Constants.Swerve.kSkidThreshold) {
      // SKID DETECTED!
      // Update only gyro angle roughly? Or trust nothing?
      // Usually we trust Gyro for rotation still, but ignore Translation.
      mLastGyroAngle = currentGyroAngle;
      return mPose; // No translation update
    }

    // --- Step 1.1: Translation Decomposition (Robot Relative) ---
    Translation2d robotRelativeDelta =
        mKinematics.estimateRobotRelativeMove(moduleDeltas, currentModuleAngles);

    // --- Step 1.2: Rotation Handling (Midpoint Integration) ---
    // Theta_mid = (Theta_now + Theta_last) / 2
    Rotation2d midpointAngle = mLastGyroAngle.interpolate(currentGyroAngle, 0.5);

    // Rotate delta to Field Frame
    Translation2d fieldRelativeDelta = robotRelativeDelta.rotateBy(midpointAngle);

    // Update Pose
    // New Pose = Old Pose + Twisted Delta
    // For pure Twist2d exp map involving curvature:
    // Twist2d twist = new Twist2d(robotRelativeDelta.getX(), robotRelativeDelta.getY(),
    // currentGyroAngle.minus(mLastGyroAngle).getRadians());
    // mPose = mPose.exp(twist);
    // BUT 1690 explicitly mentions "Midpoint Integration" then "Field Frame".
    // This implies the simpler Midpoint Method:
    // Pose += rotated_delta

    mPose =
        new Pose2d(
            mPose.getTranslation().plus(fieldRelativeDelta), currentGyroAngle // Trust Gyro absolute
            );

    mLastGyroAngle = currentGyroAngle;

    return mPose;
  }

  private double calculateRigidityMetric(double[] velocities, Rotation2d[] angles, double omega) {
    // Vector V_est_i = V_meas_i - (Omega x R_i)
    // Omega x R_i = (-omega * ry, omega * rx)
    // V_est = (vx_meas - (-omega*ry), vy_meas - (omega*rx))

    double[] vx_est = new double[4];
    double[] vy_est = new double[4];

    double mean_vx = 0;
    double mean_vy = 0;

    for (int i = 0; i < 4; i++) {
      double vx_meas = velocities[i] * angles[i].getCos();
      double vy_meas = velocities[i] * angles[i].getSin();

      // Tangential velocity from rotation
      double v_tan_x = -omega * mModuleLocations[i].getY();
      double v_tan_y = omega * mModuleLocations[i].getX();

      // Subtract rotational component to get "pure translation"
      vx_est[i] = vx_meas - v_tan_x;
      vy_est[i] = vy_meas - v_tan_y;

      mean_vx += vx_est[i];
      mean_vy += vy_est[i];
    }

    mean_vx /= 4.0;
    mean_vy /= 4.0;

    // Calculate Standard Deviation (Magnitude of variance vectors)
    double totalVariance = 0;
    for (int i = 0; i < 4; i++) {
      double dx = vx_est[i] - mean_vx;
      double dy = vy_est[i] - mean_vy;
      totalVariance += (dx * dx + dy * dy);
    }

    return Math.sqrt(totalVariance / 4.0); // Standard Deviation
  }

  public void resetPosition(Pose2d pose, Rotation2d gyroAngle, double[] currentModulePositions) {
    mPose = pose;
    mLastGyroAngle = gyroAngle;
    mLastModuleRotations = currentModulePositions.clone();
  }
}
