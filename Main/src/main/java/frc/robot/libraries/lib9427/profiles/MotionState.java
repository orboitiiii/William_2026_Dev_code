package frc.robot.libraries.lib9427.profiles;

/** Represents the state of motion at a specific point in time. */
public class MotionState {
  public final double t;
  public final double pos;
  public final double vel;
  public final double acc;

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
