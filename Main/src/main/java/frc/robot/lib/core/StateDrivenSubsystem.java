package frc.robot.lib.core;

/**
 * An abstract class representing a subsystem that is driven by the global RobotIntentState. This is
 * the core of the 1690-style architecture.
 *
 * @param <S> The local state enum for this specific subsystem.
 */
public abstract class StateDrivenSubsystem<S extends Enum<S>> {
  private S systemState;
  private boolean initialized;

  /**
   * The main entry point for the subsystem. This method is called by SubsystemManager every cycle.
   * It handles state transitions and calls the 'handle' method.
   *
   * @param desired The global RobotIntentState.
   * @param ctx The current RobotOperatingContext (sensed data, processed inputs).
   */
  public final void operate(RobotIntentState desired, RobotOperatingContext ctx) {
    // Determine the *local* state based on the *global* intent
    S next = transition(desired, ctx);

    if (!initialized) {
      // First cycle initialization
      systemState = next;
      onEnter(null, systemState);
      initialized = true;
    } else if (next != systemState) {
      // A state transition has occurred
      onExit(systemState, next);
      S prev = systemState;
      systemState = next;
      onEnter(prev, next);
    }

    // Execute the logic for the current local state
    handle(systemState, ctx);
  }

  /**
   * The state-transition logic. This method translates the global RobotIntentState into the
   * subsystem's local state.
   *
   * @param desired The global RobotIntentState.
   * @param ctx The current RobotOperatingContext.
   * @return The *local* state (S) this subsystem should be in.
   */
  protected abstract S transition(RobotIntentState desired, RobotOperatingContext ctx);

  /**
   * The state-handling logic. This method executes the behavior for the *current local state*.
   *
   * @param state The current local state (S) of this subsystem.
   * @param ctx The current RobotOperatingContext (contains pose, driver speeds, etc.)
   */
  protected abstract void handle(S state, RobotOperatingContext ctx);

  /**
   * Optional: Called once when entering a new state.
   *
   * @param prev The state being exited (null on first run).
   * @param next The state being entered.
   */
  protected void onEnter(S prev, S next) {}

  /**
   * Optional: Called once when exiting a state.
   *
   * @param prev The state being exited.
   * @param next The state being entered.
   */
  protected void onExit(S prev, S next) {}
}
