package frc.robot.lib.swerve;

/** Container for module state and per-cycle caches. */
public final class SwerveModule {
  public final SwerveModuleIo io;
  public double lastSteerRad = 0.0;
  public double lastDriveMps = 0.0;
  public boolean ignoreForOdom = false;

  public SwerveModule(SwerveModuleIo io) {
    this.io = io;
  }
}
