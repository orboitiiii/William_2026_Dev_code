package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action that waits for a specified duration.
 *
 * <p>Simple but essential for sequencing autonomous actions with timing gaps.
 *
 * <p><strong>Use Case</strong>: Delay between actions, settling time for mechanisms, or
 * synchronization with field events.
 */
public class WaitAction implements Action {
  private final double mDurationSeconds;
  private double mStartTime;

  /**
   * Creates a wait action.
   *
   * @param durationSeconds Time to wait in seconds.
   */
  public WaitAction(double durationSeconds) {
    this.mDurationSeconds = durationSeconds;
  }

  @Override
  public void start() {
    mStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {
    // No-op: just waiting
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - mStartTime >= mDurationSeconds;
  }

  @Override
  public void done() {
    // No-op: nothing to clean up
  }
}
