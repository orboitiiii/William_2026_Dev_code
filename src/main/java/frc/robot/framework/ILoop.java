package frc.robot.framework;

/**
 * Interface for periodic control loops.
 *
 * <p>Loops are registered with a {@link Looper} and executed at a fixed frequency (typically
 * 100Hz). This pattern decouples control logic from the WPILib TimedRobot's 50Hz periodic
 * callbacks.
 *
 * <p><strong>Lifecycle</strong>:
 *
 * <ol>
 *   <li>{@link #onStart(double)}: Called once when the looper starts.
 *   <li>{@link #onLoop(double)}: Called every loop period (~10ms).
 *   <li>{@link #onStop(double)}: Called once when the looper stops.
 * </ol>
 *
 * <p><strong>Attribution</strong>: Based on Team 254's Looper architecture.
 *
 * @see Looper
 */
public interface ILoop {
  /**
   * Called when the looper starts.
   *
   * @param timestamp FPGA timestamp at start.
   */
  void onStart(double timestamp);

  /**
   * Called every loop period.
   *
   * @param timestamp FPGA timestamp at this iteration.
   */
  void onLoop(double timestamp);

  /**
   * Called when the looper stops.
   *
   * @param timestamp FPGA timestamp at stop.
   */
  void onStop(double timestamp);
}
