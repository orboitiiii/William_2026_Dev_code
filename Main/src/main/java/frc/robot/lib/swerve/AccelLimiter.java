package frc.robot.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class AccelLimiter {
  private final SwerveConfig cfg;
  private double vxPrev;
  private double vyPrev;
  private double wPrev;

  public AccelLimiter(SwerveConfig cfg) {
    this.cfg = cfg;
    reset();
  }

  public void reset() {
    vxPrev = 0.0;
    vyPrev = 0.0;
    wPrev = 0.0;
  }

  public ChassisSpeeds limit(ChassisSpeeds target) {
    double dt = cfg.dt;
    double vx = target.vxMetersPerSecond;
    double vy = target.vyMetersPerSecond;

    double axWanted = (vx - vxPrev) / dt;
    double ayWanted = (vy - vyPrev) / dt;

    double dirX = Math.hypot(vx, vy) > 1e-4 ? vx : vxPrev;
    double dirY = Math.hypot(vx, vy) > 1e-4 ? vy : vyPrev;
    double dirN = Math.hypot(dirX, dirY);
    double fx = (dirN > 1e-9) ? dirX / dirN : 1.0;
    double fy = (dirN > 1e-9) ? dirY / dirN : 0.0;

    double aF = axWanted * fx + ayWanted * fy;
    double axF = aF * fx;
    double ayF = aF * fy;
    double axL = axWanted - axF;
    double ayL = ayWanted - ayF;

    double vMagPrev = Math.hypot(vxPrev, vyPrev);
    double aFmax =
        Math.max(0.0, cfg.aMaxForward0 * (1.0 - vMagPrev / Math.max(cfg.maxWheelSpeed, 1e-6)));
    // aF = SwerveUtil.clamp(aF, -aFmax, aFmax);
    // Limit forward (positive) acceleration as speed increases, but still allow braking so the
    // robot can decelerate quickly even when already at top speed.
    if (aF > aFmax) {
      aF = aFmax;
    } else if (aF < -cfg.aMaxSkid) {
      aF = -cfg.aMaxSkid;
    }
    axF = aF * fx;
    ayF = aF * fy;

    double aLmag = Math.hypot(axL, ayL);
    if (aLmag > cfg.aMaxLateral) {
      double s = cfg.aMaxLateral / aLmag;
      axL *= s;
      ayL *= s;
    }

    double ax = axF + axL;
    double ay = ayF + ayL;
    double amag = Math.hypot(ax, ay);
    if (amag > cfg.aMaxSkid) {
      double s = cfg.aMaxSkid / amag;
      ax *= s;
      ay *= s;
    }

    vxPrev += ax * dt;
    vyPrev += ay * dt;

    double w = target.omegaRadiansPerSecond;
    double aw = (w - wPrev) / dt;
    aw = SwerveUtil.clamp(aw, -cfg.aMaxOmega, cfg.aMaxOmega);
    wPrev += aw * dt;

    return new ChassisSpeeds(vxPrev, vyPrev, wPrev);
  }
}
