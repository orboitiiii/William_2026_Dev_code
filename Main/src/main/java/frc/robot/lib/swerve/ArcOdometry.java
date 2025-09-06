package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class ArcOdometry {
  public Translation2d updateRobotFrameDelta(SwerveModule[] modules) {
    Translation2d sum = new Translation2d();
    int cnt = 0;
    for (SwerveModule m : modules) {
      if (m.ignoreForOdom) {
        continue;
      }
      double th1 = m.lastSteerRad;
      double th2 = SwerveUtil.wrap(m.io.getSteerAngleRad());
      double dTh = SwerveUtil.wrap(th2 - th1);
      double dX = m.io.getDriveDeltaPosM();
      Translation2d d;
      if (Math.abs(dTh) > 1e-6) {
        double r = dX / dTh;
        Translation2d v1 = new Translation2d(r, new Rotation2d(th1 - Math.PI / 2.0));
        Translation2d v2 = new Translation2d(r, new Rotation2d(th2 - Math.PI / 2.0));
        d = v2.minus(v1);
      } else {
        d = new Translation2d(dX, new Rotation2d(th2));
      }
      sum = sum.plus(d);
      cnt++;
      m.lastSteerRad = th2;
    }
    return (cnt == 0) ? new Translation2d() : new Translation2d(sum.getX() / cnt, sum.getY() / cnt);
  }
}
