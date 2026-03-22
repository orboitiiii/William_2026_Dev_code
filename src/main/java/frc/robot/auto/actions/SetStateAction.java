package frc.robot.auto.actions;

/**
 * Action that executes a state change immediately and finishes.
 *
 * <p>Used to set {@link frc.robot.GlobalData} fields during autonomous routines. The state change
 * runs in {@link #start()} and the action completes immediately.
 *
 * <p><strong>Usage</strong>:
 *
 * <pre>{@code
 * runAction(new SetStateAction(() -> {
 *     GlobalData.robotState = RobotState.SCORE;
 *     GlobalData.hasGamePiece = true;
 * }));
 * }</pre>
 *
 * @see frc.robot.GlobalData
 */
public class SetStateAction implements Action {
  private final Runnable mStateChange;

  /**
   * Creates a state-setting action.
   *
   * @param stateChange Runnable that modifies GlobalData fields.
   */
  public SetStateAction(Runnable stateChange) {
    mStateChange = stateChange;
  }

  @Override
  public void start() {
    mStateChange.run();
  }

  @Override
  public void update() {}

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void done() {}
}
