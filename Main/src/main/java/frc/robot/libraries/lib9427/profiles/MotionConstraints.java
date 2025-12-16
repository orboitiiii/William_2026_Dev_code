package frc.robot.libraries.lib9427.profiles;

/** Constraints for motion profiling. */
public class MotionConstraints {
  public final double maxVelocity;
  public final double maxAcceleration;
  public final double maxJerk;

  public MotionConstraints(double maxVelocity, double maxAcceleration, double maxJerk) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxJerk = maxJerk;
  }
}
