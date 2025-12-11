package frc.robot.framework;

/**
 * Interface for loops, which are routines that run periodically in the robot code (such as periodic
 * gyroscope calibration, etc.)
 *
 * <p>Based on Team 254's Looper architecture.
 *
 * @author Team 254
 */
public interface ILoop {
  void onStart(double timestamp);

  void onLoop(double timestamp);

  void onStop(double timestamp);
}
