package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A First-Principles implementation of a sensor fusion tracker using Inverse Variance Weighting.
 * <p>
 * Theory: The optimal estimate of a state given two independent measurements is the weighted average,
 * where weights are proportional to the inverse of the variance (uncertainty).
 */
public final class FomPoseTracker {

  private static final double MIN_UNCERTAINTY = 1e-6; // Prevent division by zero

  /** The current best estimate of the robot's pose. */
  private Pose2d currentPose = new Pose2d();

  /** Current uncertainty of the odometry (Standard Deviation in meters). */
  private double odometryUncertainty = 0.02;

  public void reset(final Pose2d startPose) {
    this.currentPose = startPose;
    this.odometryUncertainty = 0.02;
  }

  /**
   * Prediction Step (Time Update): Project the state forward using kinematics.
   *
   * @param fieldDelta Translation delta in field frame (from Odometry + Gyro).
   * @param deltaYawRad Change in yaw (from Gyro).
   */
  public void predict(final Translation2d fieldDelta, final double deltaYawRad) {
    // X_k = X_{k-1} + Delta
    currentPose = new Pose2d(
        currentPose.getTranslation().plus(fieldDelta),
        currentPose.getRotation().plus(new Rotation2d(deltaYawRad))
    );
  }

  /**
   * Correction Step (Measurement Update): Fuse an external measurement.
   * Logic: w_1 = 1/var_1, w_2 = 1/var_2. X_new = (w_1*X_1 + w_2*X_2) / (w_1 + w_2).
   *
   * @param visionPose The pose measured by vision.
   * @param visionUncertainty The uncertainty (FoM) of the vision measurement.
   */
  public void correct(final Pose2d visionPose, final double visionUncertainty) {
    // Calculate Weights (Inverse Variance)
    double varOdom = Math.pow(Math.max(MIN_UNCERTAINTY, odometryUncertainty), 2);
    double varVision = Math.pow(Math.max(MIN_UNCERTAINTY, visionUncertainty), 2);

    double wOdom = 1.0 / varOdom;
    double wVision = 1.0 / varVision;
    double totalWeight = wOdom + wVision;

    // Calculate Alpha (Vision's influence)
    // alpha = wVision / (wOdom + wVision)
    double alpha = wVision / totalWeight;

    // Linear Interpolation (Lerp) based on Alpha
    // New = Odom + alpha * (Vision - Odom)
    Translation2d newTrans = currentPose.getTranslation().interpolate(visionPose.getTranslation(), alpha);
    Rotation2d newRot = currentPose.getRotation().interpolate(visionPose.getRotation(), alpha);

    currentPose = new Pose2d(newTrans, newRot);

    // Update System Uncertainty
    // The new system uncertainty is lower because we added information.
    // 1/var_new = 1/var_1 + 1/var_2
    double newVar = 1.0 / totalWeight;
    this.odometryUncertainty = Math.sqrt(newVar);
  }

  public Pose2d get() {
    return currentPose;
  }

  public void increaseUncertainty(double amount) {
      this.odometryUncertainty += amount;
  }

  public void setUncertainty(double value) {
      this.odometryUncertainty = value;
  }
}