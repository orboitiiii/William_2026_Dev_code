package frc.robot.auto.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Composite action that runs multiple actions sequentially.
 *
 * <p>Each child action completes before the next one starts. The SeriesAction completes when the
 * last child finishes.
 *
 * <p><strong>Use Case</strong>: Chaining dependent actions where order matters, such as: position
 * arm → shoot → retract arm.
 *
 * <p><strong>Timing Behavior</strong>:
 *
 * <pre>
 * Action A: ──────────▶
 *                     Action B: ──────────▶
 *                                         Action C: ──────────▶
 * ↑                                                           ↑
 * start()                                                   done()
 * </pre>
 *
 * @see ParallelAction
 */
public class SeriesAction implements Action {
  private final List<Action> mActions;
  private int mCurrentIndex;
  private Action mCurrentAction;

  /**
   * Creates a series action from a list.
   *
   * @param actions List of actions to run sequentially.
   */
  public SeriesAction(List<Action> actions) {
    mActions = new ArrayList<>(actions);
    mCurrentIndex = 0;
    mCurrentAction = null;
  }

  /**
   * Creates a series action from varargs.
   *
   * @param actions Actions to run sequentially.
   */
  public SeriesAction(Action... actions) {
    this(Arrays.asList(actions));
  }

  @Override
  public void start() {
    if (!mActions.isEmpty()) {
      mCurrentAction = mActions.get(0);
      mCurrentAction.start();
    }
  }

  @Override
  public void update() {
    if (mCurrentAction == null) {
      return;
    }

    mCurrentAction.update();

    // Advance to next action if current is finished
    if (mCurrentAction.isFinished()) {
      mCurrentAction.done();
      mCurrentIndex++;

      if (mCurrentIndex < mActions.size()) {
        mCurrentAction = mActions.get(mCurrentIndex);
        mCurrentAction.start();
      } else {
        mCurrentAction = null;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return mCurrentIndex >= mActions.size();
  }

  @Override
  public void done() {
    if (mCurrentAction != null) {
      mCurrentAction.done();
    }
  }
}
