package frc.robot.subsystems;

import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;

/**
 * Superstructure Subsystem - Central Coordinator for Mechanisms.
 *
 * <p>This subsystem acts as the "brain" that orchestrates all upper mechanisms (Shooter, Intake,
 * Elevator, Climber, etc.) using a strict State Machine architecture inspired by FRC Teams 254 and
 * 1678.
 *
 * <p><strong>Architecture Overview</strong>:
 *
 * <pre>
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    Operator / Auto                          │
 * │                         │                                   │
 * │                    setWantedState()                         │
 * │                         ▼                                   │
 * │  ┌─────────────────────────────────────────────────────┐    │
 * │  │              WantedState (Desired)                  │    │
 * │  └─────────────────────────────────────────────────────┘    │
 * │                         │                                   │
 * │                  State Machine Logic                        │
 * │                         ▼                                   │
 * │  ┌─────────────────────────────────────────────────────┐    │
 * │  │              SystemState (Actual)                   │    │
 * │  └─────────────────────────────────────────────────────┘    │
 * │                         │                                   │
 * │                writePeriodicOutputs()                       │
 * │                         ▼                                   │
 * │                   Hardware Outputs                          │
 * └─────────────────────────────────────────────────────────────┘
 * </pre>
 *
 * <p><strong>Key Concepts</strong>:
 *
 * <ul>
 *   <li><strong>SystemState</strong>: The ACTUAL physical state of the mechanisms.
 *   <li><strong>WantedState</strong>: The DESIRED state requested by operator or auto.
 *   <li><strong>State Transitions</strong>: Logic in handle*() methods determines if/when to
 *       transition between states.
 * </ul>
 *
 * <p><strong>Thread Safety</strong>: All state access is synchronized as the Looper runs on a
 * separate thread from the main robot loop.
 */
public class Superstructure extends Subsystem {
  private static Superstructure mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global Superstructure instance.
   */
  public static Superstructure getInstance() {
    if (mInstance == null) {
      mInstance = new Superstructure();
    }
    return mInstance;
  }

  /**
   * Physical states of the superstructure mechanisms.
   *
   * <p>Each state represents a distinct mechanical configuration with specific motor outputs and
   * sensor expectations.
   */
  public enum SystemState {
    /** All mechanisms idle, minimal power consumption. */
    IDLE,
    /** Preparing to shoot: spooling flywheel, calculating aim. */
    PROCESSING_SHOT,
    /** Actively shooting: running indexer to feed game pieces. */
    SHOOTING,
    /** Intake deployed, rollers running to acquire game pieces. */
    INTAKING,
    /** Climb sequence active: extending/retracting hooks. */
    CLIMBING
  }

  /**
   * Operator-requested states.
   *
   * <p>The state machine maps these high-level intents to specific SystemStates.
   */
  public enum WantedState {
    /** Request idle state. */
    IDLE,
    /** Request shooting sequence. */
    SHOOT,
    /** Request intake operation. */
    INTAKE,
    /** Request climb sequence. */
    CLIMB
  }

  private SystemState mSystemState = SystemState.IDLE;
  private WantedState mWantedState = WantedState.IDLE;

  /** True during the first cycle after a state change. */
  private boolean mStateChanged;

  private Superstructure() {
    // Acquire child subsystem instances here if needed
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
              mSystemState = SystemState.IDLE;
              mWantedState = WantedState.IDLE;
              mStateChanged = true;
            }
          }

          @Override
          public void onLoop(double timestamp) {
            synchronized (Superstructure.this) {
              SystemState newState = mSystemState;

              switch (mSystemState) {
                case IDLE:
                  newState = handleIdle();
                  break;
                case PROCESSING_SHOT:
                  newState = handleProcessingShot();
                  break;
                case SHOOTING:
                  newState = handleShooting();
                  break;
                case INTAKING:
                  newState = handleIntaking();
                  break;
                case CLIMBING:
                  newState = handleClimbing();
                  break;
                default:
                  newState = SystemState.IDLE;
              }

              if (newState != mSystemState) {
                System.out.println(
                    "Superstructure State Changed: " + mSystemState + " -> " + newState);
                mSystemState = newState;
                mStateChanged = true;
              } else {
                mStateChanged = false;
              }
            }
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  // --- State Transition Logic ---

  /**
   * Handles the IDLE state.
   *
   * @return The next state based on wanted state requests.
   */
  private SystemState handleIdle() {
    switch (mWantedState) {
      case SHOOT:
        return SystemState.PROCESSING_SHOT;
      case INTAKE:
        return SystemState.INTAKING;
      case CLIMB:
        return SystemState.CLIMBING;
      default:
        return SystemState.IDLE;
    }
  }

  /**
   * Handles shot preparation state.
   *
   * <p>Transitions to SHOOTING when flywheel is at speed and aim is locked.
   *
   * @return The next state.
   */
  private SystemState handleProcessingShot() {
    if (mWantedState != WantedState.SHOOT) {
      return SystemState.IDLE;
    }
    // TODO: Check flywheel ready && aimed conditions
    return SystemState.PROCESSING_SHOT;
  }

  /**
   * Handles active shooting state.
   *
   * @return The next state.
   */
  private SystemState handleShooting() {
    if (mWantedState != WantedState.SHOOT) {
      return SystemState.IDLE;
    }
    return SystemState.SHOOTING;
  }

  /**
   * Handles intake operation state.
   *
   * @return The next state.
   */
  private SystemState handleIntaking() {
    if (mWantedState != WantedState.INTAKE) {
      return SystemState.IDLE;
    }
    return SystemState.INTAKING;
  }

  /**
   * Handles climbing state.
   *
   * @return The next state.
   */
  private SystemState handleClimbing() {
    // Climbing typically continues until explicitly cancelled or sequence completes
    return SystemState.CLIMBING;
  }

  // --- Subsystem Interface ---

  @Override
  public void readPeriodicInputs() {
    // Read sensors relevant to superstructure coordination
  }

  @Override
  public void writePeriodicOutputs() {
    // Apply outputs based on mSystemState
  }

  @Override
  public boolean checkConnectionActive() {
    return true; // Virtual subsystem, no direct hardware
  }

  @Override
  public boolean checkConnectionPassive() {
    return true;
  }

  @Override
  public boolean checkSanityPassive() {
    return true;
  }

  @Override
  public void outputTelemetry() {}

  @Override
  public void stop() {
    mWantedState = WantedState.IDLE;
    mSystemState = SystemState.IDLE;
  }

  @Override
  public void zeroSensors() {}

  // --- Public API ---

  /**
   * Sets the desired superstructure state.
   *
   * <p>The state machine will transition to the appropriate SystemState based on mechanism
   * readiness and safety interlocks.
   *
   * @param wantedState The desired operating mode.
   */
  public synchronized void setWantedState(WantedState wantedState) {
    mWantedState = wantedState;
  }
}
