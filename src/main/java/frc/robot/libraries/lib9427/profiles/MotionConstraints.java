package frc.robot.libraries.lib9427.profiles;

/**
 * Motion profile constraints defining physical limits.
 *
 * <p>These constraints bound the motion profile generator to prevent commands that exceed mechanism
 * capabilities.
 *
 * <p><strong>Units</strong>: All values are in mechanism-specific units, typically meters and
 * seconds for linear systems, or radians and seconds for rotational systems.
 */
public class MotionConstraints {
  /** Maximum velocity (units/s). */
  public final double maxVelocity;

  /** Maximum acceleration (units/s²). */
  public final double maxAcceleration;

  /** Maximum jerk (units/s³). */
  public final double maxJerk;

  /**
   * Creates motion constraints.
   *
   * @param maxVelocity Maximum velocity.
   * @param maxAcceleration Maximum acceleration.
   * @param maxJerk Maximum jerk (rate of change of acceleration).
   */
  public MotionConstraints(double maxVelocity, double maxAcceleration, double maxJerk) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxJerk = maxJerk;
  }
}
