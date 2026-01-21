package frc.robot.auto;

/**
 * Exception thrown when an autonomous routine is stopped early.
 *
 * <p>This is a control-flow mechanism, not an error condition. It allows the {@link
 * AutoModeBase#runAction} method to unwind cleanly when the routine is cancelled.
 *
 * @see AutoModeBase#isActiveWithThrow()
 */
public class AutoModeEndedException extends Exception {
  private static final long serialVersionUID = 1L;

  /** Creates a new AutoModeEndedException with a default message. */
  public AutoModeEndedException() {
    super("Auto mode ended early");
  }
}
