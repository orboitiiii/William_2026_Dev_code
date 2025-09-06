package frc.robot.lib.core;

public abstract class SubsystemNode<S extends Enum<S>> {
  private S systemState;
  private boolean initialized;

  public final void operate(RobotIntentState desired, RobotContext ctx) {
    S next = transition(desired, ctx);
    if (!initialized) {
      systemState = next;
      onEnter(null, systemState);
      initialized = true;
    } else if (next != systemState) {
      onExit(systemState, next);
      S prev = systemState;
      systemState = next;
      onEnter(prev, next);
    }
    handle(systemState, ctx);
  }

  protected abstract S transition(RobotIntentState desired, RobotContext ctx);

  protected abstract void handle(S state, RobotContext ctx);

  protected void onEnter(S prev, S next) {}

  protected void onExit(S prev, S next) {}
}
