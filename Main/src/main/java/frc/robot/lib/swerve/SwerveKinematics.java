package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Kinematics and target optimization helpers. */
public final class SwerveKinematics {
  private SwerveKinematics() {}

  public static ModuleState[] inverseKinematics(
      SwerveConfig cfg, Translation2d[] r, ChassisSpeeds body) {
    ModuleState[] out = new ModuleState[4];
    double max = 0.0;
    for (int i = 0; i < 4; i++) {
      double vx = body.vxMetersPerSecond - body.omegaRadiansPerSecond * r[i].getY();
      double vy = body.vyMetersPerSecond + body.omegaRadiansPerSecond * r[i].getX();
      double sp = Math.hypot(vx, vy);
      Rotation2d ang = new Rotation2d(vx, vy);
      out[i] = new ModuleState(sp, ang);
      max = Math.max(max, sp);
    }
    if (max > cfg.maxWheelSpeed + 1e-9) {
      double s = cfg.maxWheelSpeed / max;
      for (ModuleState m : out) {
        m.speedMps *= s;
      }
    }
    return out;
  }

  public static ModuleState optimize(ModuleState desired, Rotation2d current) {
    double delta = desired.angle.minus(current).getRadians();
    if (Math.abs(delta) > Math.PI / 2.0) {
      return new ModuleState(
          -desired.speedMps, desired.angle.plus(Rotation2d.fromRadians(Math.PI)));
    }
    return desired;
  }
}
