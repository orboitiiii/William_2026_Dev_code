package frc.robot.framework.requests;

/**
 * Interface for subsystem requests. This allows for complex actions to be queued or executed by
 * subsystems.
 *
 * <p>Based on Team 1678's Request architecture.
 *
 * @author Team 1678
 */
public interface Request {
  /** executing the request */
  void act();

  /**
   * @return true if the request is finished
   */
  boolean isFinished();
}
