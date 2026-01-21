package frc.robot.auto.actions;

/**
 * Interface for autonomous actions.
 *
 * <p>An Action represents a discrete, iterative task executed within an autonomous routine. Actions
 * follow a strict lifecycle managed by {@link frc.robot.auto.AutoModeBase#runAction}.
 *
 * <p><strong>Lifecycle</strong>:
 *
 * <pre>
 * +-----------------------------------------------------------+
 * |  start()                                                   |
 * |    +---> Called once when action begins                    |
 * +-----------------------------------------------------------+
 * |  update()                                                  |
 * |    +---> Called every 20ms while action is running         |
 * +-----------------------------------------------------------+
 * |  isFinished()                                              |
 * |    +---> Checked after each update() to determine if       |
 * |         the action should terminate                        |
 * +-----------------------------------------------------------+
 * |  done()                                                    |
 * |    +---> Called once when action ends (normal or abort)    |
 * +-----------------------------------------------------------+
 * </pre>
 *
 * <p><strong>Design Principles (First Principles)</strong>:
 *
 * <ul>
 *   <li><strong>Single Responsibility</strong>: Each action does one thing.
 *   <li><strong>Clear Lifecycle</strong>: start → update → done is a clean state machine with
 *       guaranteed entry and exit points.
 *   <li><strong>Composability</strong>: Actions can be combined using {@link SeriesAction} and
 *       {@link ParallelAction}.
 * </ul>
 *
 * <p><strong>Attribution</strong>: Based on Team 254's autonomous framework.
 *
 * @see frc.robot.auto.AutoModeBase
 */
public interface Action {
  /**
   * Called once when the action starts.
   *
   * <p>Use this to initialize state, record timestamps, reset controllers, etc.
   */
  void start();

  /**
   * Called iteratively until {@link #isFinished()} returns true.
   *
   * <p>Core control logic belongs here. Called every ~20ms.
   */
  void update();

  /**
   * Determines whether the action has completed.
   *
   * <p>Checked after each {@link #update()} call. Return true to terminate.
   *
   * @return True if the action is complete.
   */
  boolean isFinished();

  /**
   * Called once when the action ends.
   *
   * <p>This is guaranteed to be called whether the action completes normally or is interrupted. Use
   * for cleanup: stopping motors, logging, releasing resources.
   */
  void done();
}
