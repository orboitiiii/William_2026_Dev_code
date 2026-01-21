package frc.robot.framework.requests;

/**
 * Interface for asynchronous subsystem requests.
 *
 * <p>A Request represents a discrete command that can be queued or executed by a subsystem. Unlike
 * direct method calls, Requests can be:
 *
 * <ul>
 *   <li>Queued for later execution
 *   <li>Polled for completion status
 *   <li>Composed into sequences
 * </ul>
 *
 * <p><strong>Attribution</strong>: Based on Team 1678's Request architecture.
 *
 * @see frc.robot.framework.Subsystem
 */
public interface Request {
  /**
   * Executes the request action.
   *
   * <p>Called when the request is ready to be processed.
   */
  void act();

  /**
   * Returns whether the request has completed.
   *
   * @return True if the request is finished.
   */
  boolean isFinished();
}
