package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Translation2d;

/** Small math utilities. */
public final class SwerveUtil {
  private SwerveUtil() {}

  public static double wrap(double a) {
    double p = Math.PI;
    while (a > p) {
      a -= 2 * p;
    }
    while (a < -p) {
      a += 2 * p;
    }
    return a;
  }

  public static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  public static double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }

  public static double sign(double v) {
    if (Math.abs(v) < 1e-6) {
      return 0.0;
    }
    return Math.signum(v);
  }

  public static Translation2d rotate(Translation2d v, double yawRad) {
    double c = Math.cos(yawRad);
    double s = Math.sin(yawRad);
    return new Translation2d(c * v.getX() - s * v.getY(), s * v.getX() + c * v.getY());
  }
}
