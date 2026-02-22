package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Vision field pose estimate - Encapsulates robot pose measurements from vision system.
 *
 * <p>Reference: FRC 254 2025 VisionFieldPoseEstimate design
 *
 * <p>This class contains:
 *
 * <ul>
 *   <li>Vision estimated Pose2d
 *   <li>Precise timestamp (for alignment with odometry history)
 *   <li>Measurement standard deviations (for Kalman Filter fusion weights)
 *   <li>Number of detected tags (for trust evaluation)
 * </ul>
 */
public class VisionFieldPoseEstimate {

  private final Pose2d visionRobotPoseMeters;
  private final double timestampSeconds;
  private final Matrix<N3, N1> visionMeasurementStdDevs;
  private final int numTags;

  /** Average tag distance (m); 0 or large value when not available. Used for gyro-reset logic. */
  private final double avgTagDistMeters;

  /**
   * Creates a vision field pose estimate.
   *
   * @param visionRobotPoseMeters Vision estimated robot pose (WPILib Blue Alliance coordinate
   *     system)
   * @param timestampSeconds FPGA timestamp (latency corrected)
   * @param visionMeasurementStdDevs Measurement standard deviations [x, y, theta]
   * @param numTags Number of AprilTags detected
   * @param avgTagDistMeters Average tag distance in meters (0 if unknown)
   */
  public VisionFieldPoseEstimate(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs,
      int numTags,
      double avgTagDistMeters) {
    this.visionRobotPoseMeters = visionRobotPoseMeters;
    this.timestampSeconds = timestampSeconds;
    this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    this.numTags = numTags;
    this.avgTagDistMeters = avgTagDistMeters;
  }

  /**
   * @return Vision estimated robot pose (WPILib Blue Alliance coordinate system)
   */
  public Pose2d getVisionRobotPoseMeters() {
    return visionRobotPoseMeters;
  }

  /**
   * @return FPGA timestamp (seconds), camera/processing latency corrected
   */
  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  /**
   * @return Measurement standard deviation matrix [x, y, theta] for Kalman Filter fusion
   */
  public Matrix<N3, N1> getVisionMeasurementStdDevs() {
    return visionMeasurementStdDevs;
  }

  /**
   * @return Number of AprilTags detected in this estimate
   */
  public int getNumTags() {
    return numTags;
  }

  /**
   * Checks if this estimate is high trust.
   *
   * @return true if >= 2 tags detected
   */
  public boolean isHighTrust() {
    return numTags >= 2;
  }

  /** Average tag distance in meters (0 if unknown). Used for gyro-reset when close. */
  public double getAvgTagDistMeters() {
    return avgTagDistMeters;
  }

  @Override
  public String toString() {
    return String.format(
        "VisionFieldPoseEstimate{pose=%s, timestamp=%.3f, tags=%d, stdDevs=%s}",
        visionRobotPoseMeters, timestampSeconds, numTags, visionMeasurementStdDevs);
  }
}
