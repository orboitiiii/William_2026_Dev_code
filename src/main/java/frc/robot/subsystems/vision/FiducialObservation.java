package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import java.util.Arrays;
import java.util.Objects;

/**
 * Represents an observation of a fiducial marker (AprilTag) with position and quality data.
 *
 * <p>Reference: FRC 254 2025 FiducialObservation pattern.
 *
 * @param id The fiducial marker ID.
 * @param txnc Normalized horizontal offset (-1 to 1, 0 = center of image).
 * @param tync Normalized vertical offset (-1 to 1, 0 = center of image).
 * @param ambiguity Pose ambiguity score (0 = confident, 1 = ambiguous).
 * @param area Target area as percentage of image (0-100).
 */
public record FiducialObservation(int id, double txnc, double tync, double ambiguity, double area) {

  /**
   * Converts a Limelight RawFiducial to a FiducialObservation.
   *
   * @param fiducial The Limelight raw fiducial data.
   * @return FiducialObservation, or null if input is null.
   */
  public static FiducialObservation fromLimelight(LimelightHelpers.RawFiducial fiducial) {
    if (fiducial == null) {
      return null;
    }
    return new FiducialObservation(
        fiducial.id, fiducial.txnc, fiducial.tync, fiducial.ambiguity, fiducial.ta);
  }

  /**
   * Converts an array of Limelight RawFiducials to FiducialObservation array.
   *
   * @param fiducials The Limelight raw fiducial array.
   * @return Array of FiducialObservations (null entries filtered out).
   */
  public static FiducialObservation[] fromLimelight(LimelightHelpers.RawFiducial[] fiducials) {
    if (fiducials == null) {
      return new FiducialObservation[0];
    }
    return Arrays.stream(fiducials)
        .map(FiducialObservation::fromLimelight)
        .filter(Objects::nonNull)
        .toArray(FiducialObservation[]::new);
  }

  /**
   * Checks if this observation has high ambiguity.
   *
   * @param threshold The ambiguity threshold (typically 0.25).
   * @return true if ambiguity exceeds threshold.
   */
  public boolean isHighAmbiguity(double threshold) {
    return ambiguity > threshold;
  }

  /**
   * Checks if this observation has small area.
   *
   * @param minArea The minimum area threshold.
   * @return true if area is below threshold.
   */
  public boolean isSmallArea(double minArea) {
    return area < minArea;
  }
}
