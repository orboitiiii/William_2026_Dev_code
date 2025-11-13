package frc.robot.lib.swerve;

/** Container for module state and per-cycle caches. */
public final class SwerveModule {
  // Updated to use the renamed SwerveModuleIO interface
  public final SwerveModuleIO io;
  public double lastSteerRad = 0.0;
  public double lastDriveMps = 0.0;
  public boolean ignoreForOdom = false;

  public SwerveModule(SwerveModuleIO io) {
    this.io = io;
  }
}
