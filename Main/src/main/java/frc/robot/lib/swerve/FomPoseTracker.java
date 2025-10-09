package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Tracks the robot pose by fusing odometry and vision measurements. Each measurement has an
 * associated “figure of merit” (FoM) that represents uncertainty; smaller FoM values imply higher
 * confidence. Fusion uses inverse variance weighting: the weight of each measurement is 1/(FoM^2),
 * in accordance with the classic inverse‑variance weighted average
 */
public final class FomPoseTracker {
  /** Threshold: vision FoM above this is ignored (no update). */
  private static final double MAX_VISION_FOM = 1e8;

  /** Minimum FoM value to avoid division by zero. */
  private static final double MIN_FOM = 1e-6;

  /** Latest estimated pose in the field frame. */
  public Pose2d estimatedPose = new Pose2d();

  /** Current FoM for odometry (units: meters). Smaller means higher confidence. */
  public double odometryFom = 0.02;

  /** Current FoM for vision measurements (dimensionless). Smaller means higher confidence. */
  public double visionFomScore = 1e9;

  /** Resets the tracker to a known pose and default FoM values. */
  public void reset(final Pose2d start) {
    estimatedPose = start;
    odometryFom = 0.02;
    visionFomScore = 1e9;
  }

  /**
   * Predicts the next pose using a field‑frame translation and a change in yaw.
   *
   * @param fieldDelta translation delta in field coordinates
   * @param deltaYawRad change in yaw in radians
   */
  public void predictPose(final Translation2d fieldDelta, final double deltaYawRad) {
    estimatedPose =
        new Pose2d(
            estimatedPose.getX() + fieldDelta.getX(),
            estimatedPose.getY() + fieldDelta.getY(),
            estimatedPose.getRotation().plus(new Rotation2d(deltaYawRad)));
  }

  /**
   * Sets the odometry FoM directly (units: meters). Lower values increase odometry influence.
   *
   * @param newFomMeters updated odometry FoM, clamped to non‑negative
   */
  public void setOdometryFom(final double newFomMeters) {
    odometryFom = Math.max(0.0, newFomMeters);
  }

  /**
   * Corrects the pose by fusing a vision pose using explicit FoM values for odometry and vision.
   * Weights are computed as 1/(FoM^2), following inverse‑variance
   * weighting:contentReference[oaicite:1]{index=1}.
   *
   * @param visionPose pose from vision in field coordinates
   * @param odomFomExternal FoM from an external odometry source (units: meters)
   * @param visionFomScore FoM for the vision measurement (dimensionless)
   */
  public void correctPoseWithVisionAndFoms(
      final Pose2d visionPose, final double odomFomExternal, final double visionFomScore) {
    // Ignore if vision FoM is too large (i.e. untrusted vision)
    if (visionFomScore >= MAX_VISION_FOM) return;

    // Compute inverse variance weights; use MIN_FOM to avoid division by zero
    double odomWeight = 1.0 / Math.pow(Math.max(MIN_FOM, odomFomExternal), 2);
    double visionWeight = 1.0 / Math.pow(Math.max(MIN_FOM, visionFomScore), 2);

    // Fraction of vision pose to blend in: visionWeight / (odomWeight + visionWeight)
    double alpha = visionWeight / (odomWeight + visionWeight);

    // Blend poses: move estimatedPose toward visionPose by fraction alpha
    blendPose(visionPose, alpha);

    // Update stored FoMs: record vision FoM and shrink odometry FoM if vision is more confident
    this.visionFomScore = visionFomScore;
  }

  /**
   * Corrects the pose using internally stored FoM values. Vision influence is based on the ratio of
   * inverse squared FoMs.
   *
   * @param visionPose vision pose in field coordinates
   * @param visionFomScore FoM for the vision measurement
   */
  public void correctPoseWithVision(final Pose2d visionPose, final double visionFomScore) {
    // Ignore if vision FoM is too large (i.e. untrusted vision)
    if (visionFomScore >= MAX_VISION_FOM) return;

    // Use stored odometryFom; compute weights as inverse squared FoMs
    double odomWeight = 1.0 / Math.pow(Math.max(MIN_FOM, odometryFom), 2);
    double visionWeight = 1.0 / Math.pow(Math.max(MIN_FOM, visionFomScore), 2);

    double alpha = visionWeight / (odomWeight + visionWeight);
    blendPose(visionPose, alpha);

    // Update FoMs as in the other method
    this.visionFomScore = visionFomScore;
    if (visionFomScore < odometryFom) {
      odometryFom = Math.max(0.0, visionFomScore * 0.9);
    }
  }

  /**
   * Corrects the pose using arbitrary non-negative weights for odometry and vision.
   *
   * @param visionPose vision pose in field coordinates
   * @param wOdom non-negative weight for odometry
   * @param wVision non-negative weight for vision
   */
  public void correctPoseWithVisionAndWeights(
      final Pose2d visionPose, final double wOdom, final double wVision) {
    double odomWeight = Math.max(0.0, wOdom);
    double visionWeight = Math.max(0.0, wVision);
    if (odomWeight + visionWeight <= 1e-9) return;

    double alpha = visionWeight / (odomWeight + visionWeight);
    blendPose(visionPose, alpha);
  }

  /**
   * Linearly blends the current pose toward a vision pose by the given fraction. Ensures that alpha
   * is clamped to [0,1].
   *
   * @param visionPose vision pose in field coordinates
   * @param alpha fraction of the vision pose to incorporate
   */
  private void blendPose(final Pose2d visionPose, final double alpha) {
    double a = SwerveUtil.clamp(alpha, 0.0, 1.0);

    // Interpolate x and y linearly
    double newX = SwerveUtil.lerp(estimatedPose.getX(), visionPose.getX(), a);
    double newY = SwerveUtil.lerp(estimatedPose.getY(), visionPose.getY(), a);

    // Compute the shortest angular difference, then rotate toward vision pose by fraction a
    double angleDiff =
        SwerveUtil.wrap(visionPose.getRotation().minus(estimatedPose.getRotation()).getRadians());
    Rotation2d newRotation = estimatedPose.getRotation().plus(new Rotation2d(a * angleDiff));

    estimatedPose = new Pose2d(newX, newY, newRotation);
  }
}
