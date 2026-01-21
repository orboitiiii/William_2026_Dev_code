package frc.robot.auto;

/**
 * Manages the lifecycle of autonomous routines.
 *
 * <p>This singleton class handles selection, execution, and termination of autonomous modes. The
 * autonomous routine runs in a separate thread to avoid blocking the main robot loop.
 *
 * <p><strong>Usage Pattern</strong>:
 *
 * <pre>{@code
 * // In autonomousInit():
 * AutoModeExecutor.getInstance().setAutoMode(new MyAutoMode());
 * AutoModeExecutor.getInstance().start();
 *
 * // In disabledInit():
 * AutoModeExecutor.getInstance().stop();
 * }</pre>
 *
 * <p><strong>Thread Safety</strong>: The executor creates a new thread for each autonomous run.
 * Ensure subsystem access is properly synchronized.
 *
 * <p><strong>Attribution</strong>: Based on Team 254's autonomous framework.
 *
 * @see AutoModeBase
 */
public class AutoModeExecutor {
  private static AutoModeExecutor mInstance = null;

  private AutoModeBase mAutoMode = null;
  private Thread mThread = null;

  private AutoModeExecutor() {}

  /**
   * Returns the singleton instance.
   *
   * @return The global AutoModeExecutor instance.
   */
  public static AutoModeExecutor getInstance() {
    if (mInstance == null) {
      mInstance = new AutoModeExecutor();
    }
    return mInstance;
  }

  /**
   * Sets the autonomous routine to execute.
   *
   * <p>Creates a new thread for the routine. The previous routine (if any) should be stopped before
   * calling this method.
   *
   * @param newAutoMode The autonomous routine to run.
   */
  public void setAutoMode(AutoModeBase newAutoMode) {
    mAutoMode = newAutoMode;
    mThread =
        new Thread(
            () -> {
              if (mAutoMode != null) {
                try {
                  mAutoMode.run();
                } catch (Exception e) {
                  System.err.println("[AutoModeExecutor] Auto mode crashed: " + e.getMessage());
                  e.printStackTrace();
                }
              }
            },
            "AutoModeThread");
  }

  /**
   * Starts the autonomous routine thread.
   *
   * <p>Must be called after {@link #setAutoMode(AutoModeBase)}.
   */
  public void start() {
    if (mThread != null) {
      mThread.start();
    }
  }

  /**
   * Returns whether the autonomous routine is currently running.
   *
   * @return True if the routine is active and the thread is alive.
   */
  public boolean isStarted() {
    return mAutoMode != null && mAutoMode.isActive() && mThread != null && mThread.isAlive();
  }

  /** Resets the executor, stopping any active routine. */
  public void reset() {
    if (isStarted()) {
      stop();
    }
    mAutoMode = null;
  }

  /**
   * Stops the current autonomous routine.
   *
   * <p>This triggers {@link AutoModeBase#stop()}, which will cause the routine thread to exit
   * cleanly.
   */
  public void stop() {
    if (mAutoMode != null) {
      mAutoMode.stop();
    }
    mThread = null;
  }

  /**
   * Returns the current autonomous routine.
   *
   * @return The active AutoModeBase, or null if none is set.
   */
  public AutoModeBase getAutoMode() {
    return mAutoMode;
  }

  /**
   * Returns whether the routine is currently paused.
   *
   * @return True if interrupted.
   */
  public boolean isInterrupted() {
    if (mAutoMode == null) {
      return false;
    }
    return mAutoMode.getIsInterrupted();
  }

  /** Pauses the current autonomous routine. */
  public void interrupt() {
    if (mAutoMode != null) {
      mAutoMode.interrupt();
    }
  }

  /** Resumes a paused autonomous routine. */
  public void resume() {
    if (mAutoMode != null) {
      mAutoMode.resume();
    }
  }
}
