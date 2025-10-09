package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Detects when a wheel is skidding by comparing the measured wheel speeds with the expected wheel
 * speeds based on the chassis velocity. If the ratio of fastest to slowest wheel exceeds a
 * threshold and a wheel's speed is significantly larger than expected, that wheel is flagged as
 * skidding.
 */
public final class SkidDetector {
  private final double speedRatioThreshold;

  /**
   * Creates a new skid detector.
   *
   * @param speedRatioThreshold ratio between the fastest and slowest wheel speed above which skid
   *     detection is enabled
   */
  public SkidDetector(final double speedRatioThreshold) {
    this.speedRatioThreshold = speedRatioThreshold;
  }

  /**
   * Returns a boolean array indicating which wheels are skidding.
   *
   * @param modulePositions positions of each module relative to the robot center
   * @param bodySpeeds the commanded chassis speeds in body frame
   * @param measuredWheelSpeeds absolute measured wheel speeds (m/s)
   * @return array of booleans, true if the corresponding wheel is skidding
   */
  public boolean[] detectSkid(
      final Translation2d[] modulePositions,
      final ChassisSpeeds bodySpeeds,
      final double[] measuredWheelSpeeds) {
    double[] expectedWheelSpeeds = new double[measuredWheelSpeeds.length];
    double maxMeasured = 0.0;
    double minMeasured = Double.POSITIVE_INFINITY;

    // Compute expected wheel speeds from chassis motion and track min/max measured speed
    for (int i = 0; i < measuredWheelSpeeds.length; i++) {
      double expectedVx =
          bodySpeeds.vxMetersPerSecond
              - bodySpeeds.omegaRadiansPerSecond * modulePositions[i].getY();
      double expectedVy =
          bodySpeeds.vyMetersPerSecond
              + bodySpeeds.omegaRadiansPerSecond * modulePositions[i].getX();
      expectedWheelSpeeds[i] = Math.hypot(expectedVx, expectedVy);
      maxMeasured = Math.max(maxMeasured, measuredWheelSpeeds[i]);
      minMeasured = Math.min(minMeasured, measuredWheelSpeeds[i]);
    }

    boolean[] skidDetected = new boolean[measuredWheelSpeeds.length];
    if (minMeasured < 1e-6) {
      minMeasured = 1e-6;
    }
    // Only perform skid detection if the spread between wheel speeds is significant
    if (maxMeasured / minMeasured > speedRatioThreshold) {
      double groupMax = 0.0;
      for (double v : measuredWheelSpeeds) {
        groupMax = Math.max(groupMax, v);
      }
      for (int i = 0; i < measuredWheelSpeeds.length; i++) {
        skidDetected[i] =
            (measuredWheelSpeeds[i] > 0.9 * groupMax)
                && (measuredWheelSpeeds[i] > 1.3 * expectedWheelSpeeds[i]);
      }
    }
    return skidDetected;
  }
}
