package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Detects skidding wheels by comparing measured vs expected speeds. */
public final class SkidDetector {
  private final double ratioThresh;

  public SkidDetector(double ratioThresh) {
    this.ratioThresh = ratioThresh;
  }

  public boolean[] detect(Translation2d[] r, ChassisSpeeds body, double[] measuredMps) {
    double[] expected = new double[4];
    double max = 0.0;
    double min = 1e9;
    for (int i = 0; i < 4; i++) {
      double evx = body.vxMetersPerSecond - body.omegaRadiansPerSecond * r[i].getY();
      double evy = body.vyMetersPerSecond + body.omegaRadiansPerSecond * r[i].getX();
      expected[i] = Math.hypot(evx, evy);
      max = Math.max(max, measuredMps[i]);
      min = Math.min(min, measuredMps[i]);
    }
    boolean[] skid = new boolean[4];
    if (min < 1e-6) {
      min = 1e-6;
    }
    if (max / min > ratioThresh) {
      double groupMax = 0.0;
      for (double v : measuredMps) {
        groupMax = Math.max(groupMax, v);
      }
      for (int i = 0; i < 4; i++) {
        skid[i] = (measuredMps[i] > 0.9 * groupMax) && (measuredMps[i] > 1.3 * expected[i]);
      }
    }
    return skid;
  }
}
