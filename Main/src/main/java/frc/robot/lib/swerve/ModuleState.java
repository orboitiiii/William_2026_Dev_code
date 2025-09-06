package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** Simple state for module demand (speed + angle). */
public final class ModuleState {
  public double speedMps;
  public Rotation2d angle;

  public ModuleState(double speedMps, Rotation2d angle) {
    this.speedMps = speedMps;
    this.angle = angle;
  }
}
