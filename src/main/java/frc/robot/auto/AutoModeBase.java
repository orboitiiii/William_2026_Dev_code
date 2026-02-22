package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto.actions.Action;

/**
 * Abstract base class for all autonomous routines.
 *
 * <p>This class defines the Team 254-style autonomous framework where each autonomous routine is
 * defined by implementing the {@link #routine()} method. Actions are executed sequentially using
 * {@link #runAction(Action)}.
 *
 * <p><strong>Architecture</strong>:
 *
 * <ul>
 *   <li><strong>Independent Thread</strong>: Autonomous runs in a separate thread, not blocking the
 *       main robot loop.
 *   <li><strong>Interruptible Design</strong>: Supports safe stop at any time via {@link #stop()}
 *       or interrupt/resume for operator override.
 *   <li><strong>Action Chaining</strong>: Use {@link #runAction(Action)} to execute actions
 *       sequentially within the routine.
 * </ul>
 *
 * <p><strong>Usage Example</strong>:
 *
 * <pre>{@code
 * public class MyAutoMode extends AutoModeBase {
 *     @Override
 *     protected void routine() throws AutoModeEndedException {
 *         runAction(new DriveTrajectoryAction(trajectory));
 *         runAction(new WaitAction(0.5));
 *         runAction(new ShootAction());
 *     }
 * }
 * }</pre>
 *
 * <p><strong>Attribution</strong>: Based on Team 254's autonomous framework.
 *
 * @see AutoModeExecutor
 * @see Action
 */
public abstract class AutoModeBase {
  /** Update rate for action loop: 50Hz (20ms period). */
  protected static final double kUpdateRate = 1.0 / 50.0;

  /** True if the autonomous routine is currently active. */
  protected volatile boolean mActive = false;

  /** True if the autonomous routine has been paused by the operator. */
  protected volatile boolean mIsInterrupted = false;

  /**
   * Defines the autonomous routine's action sequence.
   *
   * <p>Subclasses implement this method to define their behavior. Use {@link #runAction(Action)} to
   * execute actions in sequence.
   *
   * @throws AutoModeEndedException When the routine is stopped early.
   */
  protected abstract void routine() throws AutoModeEndedException;

  /**
   * Executes the autonomous routine.
   *
   * <p>Called by {@link AutoModeExecutor} in a separate thread. This method wraps {@link
   * #routine()} with exception handling.
   */
  public void run() {
    mActive = true;

    try {
      routine();
    } catch (AutoModeEndedException e) {
      DriverStation.reportError("AUTO MODE ENDED EARLY", false);
      return;
    }

    done();
  }

  /**
   * Called when the routine completes normally.
   *
   * <p>Override in subclasses to perform cleanup or logging.
   */
  public void done() {}

  /**
   * Stops the autonomous routine.
   *
   * <p>This sets the active flag to false, causing the next {@link #isActiveWithThrow()} check to
   * throw an exception.
   */
  public void stop() {
    mActive = false;
  }

  /**
   * Returns whether the routine is currently active.
   *
   * @return True if the routine is running.
   */
  public boolean isActive() {
    return mActive;
  }

  /**
   * Checks if active and throws if not.
   *
   * <p>This is used within the action loop to enable early termination.
   *
   * @return True if active.
   * @throws AutoModeEndedException If the routine has been stopped.
   */
  public boolean isActiveWithThrow() throws AutoModeEndedException {
    if (!isActive()) {
      throw new AutoModeEndedException();
    }
    return true;
  }

  /**
   * Pauses the autonomous routine.
   *
   * <p>The routine will resume from where it was interrupted when {@link #resume()} is called.
   */
  public void interrupt() {
    mIsInterrupted = true;
  }

  /** Resumes a previously interrupted routine. */
  public void resume() {
    mIsInterrupted = false;
  }

  /**
   * Returns whether the routine is currently paused.
   *
   * @return True if interrupted.
   */
  public boolean getIsInterrupted() {
    return mIsInterrupted;
  }

  /**
   * Executes an action, blocking until it completes or the routine stops.
   *
   * <p><strong>Control Flow</strong>:
   *
   * <ol>
   *   <li>Wait for any active interrupt to clear
   *   <li>Call {@code action.start()}
   *   <li>Loop: call {@code action.update()} every 20ms until {@code action.isFinished()} returns
   *       true
   *   <li>Call {@code action.done()}
   * </ol>
   *
   * @param action The action to execute.
   * @throws AutoModeEndedException If the routine is stopped during execution.
   */
  public void runAction(Action action) throws AutoModeEndedException {
    isActiveWithThrow();

    long waitTimeMs = (long) (kUpdateRate * 1000.0);

    // Wait for interrupt state to clear
    while (isActiveWithThrow() && mIsInterrupted) {
      try {
        Thread.sleep(waitTimeMs);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }

    // Start the action
    action.start();

    // Action update loop
    while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
      action.update();

      try {
        Thread.sleep(waitTimeMs);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }

    // Cleanup
    action.done();
  }
}
