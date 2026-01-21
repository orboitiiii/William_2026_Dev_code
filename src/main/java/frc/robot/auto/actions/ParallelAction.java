package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Composite action that runs multiple actions simultaneously.
 *
 * <p>All child actions start together and the ParallelAction completes when ALL children have
 * finished.
 *
 * <p><strong>Use Case</strong>: Running independent subsystems concurrently, such as driving while
 * spinning up a shooter.
 *
 * <p><strong>Timing Behavior</strong>:
 *
 * <pre>
 * Action A: ──────────────────────▶ (3 seconds)
 * Action B: ──────────▶             (1 second)
 * Action C: ─────────────▶          (1.5 seconds)
 *           ↑                      ↑
 *         start()               done() (after 3 seconds)
 * </pre>
 *
 * @see SeriesAction
 */
public class ParallelAction implements Action {
  private final List<Action> mActions;

  /**
   * Creates a parallel action from a list.
   *
   * @param actions List of actions to run in parallel.
   */
  public ParallelAction(List<Action> actions) {
    mActions = new ArrayList<>(actions);
  }

  /**
   * Creates a parallel action from varargs.
   *
   * @param actions Actions to run in parallel.
   */
  public ParallelAction(Action... actions) {
    this(Arrays.asList(actions));
  }

  @Override
  public void start() {
    for (Action action : mActions) {
      action.start();
    }
  }

  @Override
  public void update() {
    for (Action action : mActions) {
      if (!action.isFinished()) {
        action.update();
      }
    }
  }

  @Override
  public boolean isFinished() {
    for (Action action : mActions) {
      if (!action.isFinished()) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void done() {
    for (Action action : mActions) {
      action.done();
    }
  }
}
