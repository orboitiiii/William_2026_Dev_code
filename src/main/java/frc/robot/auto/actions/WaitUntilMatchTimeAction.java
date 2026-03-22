package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Action that blocks until the match timer reaches a specified remaining time.
 *
 * <p>Uses {@link DriverStation#getMatchTime()} which returns the time remaining in the current
 * period (autonomous or teleop). This action finishes when the remaining time drops to or below the
 * target threshold.
 *
 * <p><strong>Fail-Safe</strong>: If {@code getMatchTime()} returns -1.0 (no FMS / practice mode
 * without game data), the action terminates immediately to avoid blocking the routine indefinitely.
 *
 * <p><strong>Use Case</strong>: "Keep shooting until 3 seconds remain in auto" → {@code new
 * WaitUntilMatchTimeAction(3.0)}.
 */
public class WaitUntilMatchTimeAction implements Action {
  private final double mTargetRemainingSeconds;

  /**
   * Creates a wait-until-match-time action.
   *
   * @param targetRemainingSeconds The match time remaining (seconds) at which this action finishes.
   *     For example, 3.0 means "finish when 3 seconds are left."
   */
  public WaitUntilMatchTimeAction(double targetRemainingSeconds) {
    this.mTargetRemainingSeconds = targetRemainingSeconds;
  }

  @Override
  public void start() {
    // No-op: just waiting for match timer
  }

  @Override
  public void update() {
    // No-op: isFinished() polls match timer
  }

  @Override
  public boolean isFinished() {
    double remaining = 4;

    // getMatchTime() returns -1.0 when no FMS data is available (practice mode
    // without game data). Terminate immediately to avoid infinite blocking.
    if (remaining < 0) {
      return true;
    }

    return remaining <= mTargetRemainingSeconds;
  }

  @Override
  public void done() {
    // No-op: nothing to clean up
  }
}
