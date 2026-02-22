package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * MegaTag pose estimate data class.
 *
 * <p>Encapsulates pose estimation results from Limelight MegaTag algorithm.
 */
public record MegatagPoseEstimate(
    /** Robot pose in field coordinate system (WPILib Blue Alliance) */
    Pose2d fieldToRobot,

    /** FPGA timestamp (latency corrected) */
    double timestampSeconds,

    /** Detected Fiducial IDs */
    int[] fiducialIds,

    /** Estimate quality (0-1, 1 is best) */
    double quality,

    /** Average tag area (% of image) */
    double avgTagArea,

    /** Average tag distance (meters) */
    double avgTagDist,

    /** Tag span (meters) - distance between furthest two tags */
    double tagSpan) {
  /** Creates an empty MegaTag estimate. */
  public static MegatagPoseEstimate empty() {
    return new MegatagPoseEstimate(new Pose2d(), 0.0, new int[0], 0.0, 0.0, 0.0, 0.0);
  }

  /** Checks if this estimate is valid. */
  public boolean isValid() {
    return fiducialIds != null && fiducialIds.length > 0 && quality > 0;
  }

  /** Checks if this estimate is multi-tag. */
  public boolean isMultiTag() {
    return fiducialIds != null && fiducialIds.length >= 2;
  }
}
