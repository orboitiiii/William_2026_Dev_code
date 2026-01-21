package frc.robot.libraries.lib9427.profiles;

/**
 * Instantaneous motion state at a given time.
 *
 * <p>Represents a complete kinematic snapshot: position, velocity, and acceleration. Used as the
 * output of motion profile generators.
 */
public class MotionState {
  /** Time (seconds) from profile start. */
  public final double t;

  /** Position (mechanism units). */
  public final double pos;

  /** Velocity (mechanism units/s). */
  public final double vel;

  /** Acceleration (mechanism units/s²). */
  public final double acc;

  /**
   * Creates a motion state.
   *
   * @param t Time.
   * @param pos Position.
   * @param vel Velocity.
   * @param acc Acceleration.
   */
  public MotionState(double t, double pos, double vel, double acc) {
    this.t = t;
    this.pos = pos;
    this.vel = vel;
    this.acc = acc;
  }

  @Override
  public String toString() {
    return String.format("t=%.3f, p=%.3f, v=%.3f, a=%.3f", t, pos, vel, acc);
  }
}
