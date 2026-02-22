package frc.robot.framework;

import frc.robot.ControlBoard;
import frc.robot.GlobalData;

/**
 * Abstract base class for all robot subsystems.
 *
 * <p>Defines the lifecycle contract that all subsystems must implement. The {@link
 * SubsystemManager} orchestrates these methods across all registered subsystems.
 *
 * <p><strong>Lifecycle Ordering</strong> (per robot periodic cycle):
 *
 * <ol>
 *   <li>{@link #readPeriodicInputs()}: Read all sensors
 *   <li>{@link #operate()}: State machine dispatch based on {@link GlobalData#robotState}
 *   <li>{@link #writePeriodicOutputs()}: Apply actuator commands
 *   <li>{@link #outputTelemetry()}: Publish to dashboard
 * </ol>
 *
 * <p><strong>Architecture (Orbit 1690 Pattern)</strong>: Each subsystem overrides per-state operate
 * methods ({@code travelOperate()}, {@code scoreOperate()}, etc.) to define its behavior in each
 * robot state. The base class {@link #operate()} method dispatches to the correct method based on
 * {@link GlobalData#robotState}.
 *
 * @see SubsystemManager
 * @see GlobalData
 */
public abstract class Subsystem {
  /**
   * Reads all sensor inputs.
   *
   * <p>Called at the beginning of every robot periodic. Should be fast and non-blocking.
   */
  public abstract void readPeriodicInputs();

  /**
   * Writes all actuator outputs.
   *
   * <p>Called after state machine logic has computed the desired commands.
   */
  public abstract void writePeriodicOutputs();

  /**
   * Active Connection Check - Called ONCE at robot initialization.
   *
   * <p>Performs heavy CAN queries: reads Firmware version, verifies config. High CAN bandwidth,
   * high latency. Safe to run in disabled mode.
   *
   * @return true if all connections are healthy, false otherwise.
   */
  public abstract boolean checkConnectionActive();

  /**
   * Passive Connection Check - Called periodically (1Hz) by HealthCheckLooper.
   *
   * <p>Lightweight: reads cached status flags (LastError, Timestamp). Zero CAN bandwidth (cache
   * read), minimal CPU.
   *
   * @return true if all connections are healthy, false otherwise.
   */
  public abstract boolean checkConnectionPassive();

  /**
   * Sanity Check - Called periodically (1Hz) by HealthCheckLooper.
   *
   * <p>Verifies physics models (e.g., V_applied vs ω_measured using kV). Skipped at init since
   * motor is not moving. Zero CAN bandwidth, minimal CPU (math only).
   *
   * @return true if the system's state is sane, false otherwise.
   */
  public abstract boolean checkSanityPassive();

  /**
   * Publishes telemetry data to the dashboard.
   *
   * <p>Called every cycle. Keep this lightweight.
   */
  public abstract void outputTelemetry();

  /**
   * Stops all subsystem outputs.
   *
   * <p>Called during disable and emergency stop. Must be safe to call at any time.
   */
  public abstract void stop();

  /**
   * Resets all sensors to their zero/home position.
   *
   * <p>Called during robot initialization.
   */
  public abstract void zeroSensors();

  /**
   * Registers this subsystem's loops with the enabled looper.
   *
   * @param enabledLooper The looper to register with.
   */
  public abstract void registerEnabledLoops(Looper enabledLooper);

  // ============================================================
  // PER-STATE OPERATE METHODS (Orbit 1690 Pattern)
  // ============================================================
  // Override in subclasses to define behavior for each RobotState.
  // Default: no-op (mechanism does nothing in that state).

  /** Called when robotState == TRAVEL. Default: no-op. */
  public void travelOperate() {}

  /** Called when robotState == INTAKE. Default: no-op. */
  public void intakeOperate() {}

  /** Called when robotState == SCORE. Default: no-op. */
  public void scoreOperate() {}

  /** Called when robotState == PASS. Default: no-op. */
  public void passOperate() {}

  /** Called when robotState == CLIMB. Default: no-op. */
  public void climbOperate() {}

  /** Called when robotState == CALIBRATE. Default: no-op. */
  public void calibrateOperate() {}

  /** Called when robotState == SCORE_TEST. Default: no-op. */
  public void scoreTestOperate() {}

  /** Called when robotState == DISABLED. Default: calls {@link #stop()}. */
  public void disabledOperate() {
    stop();
  }

  /**
   * Dispatches to the appropriate per-state operate method based on {@link GlobalData#robotState}.
   *
   * <p>This method is {@code final} — subclasses must NOT override it. Instead, override the
   * individual per-state methods ({@code travelOperate()}, {@code scoreOperate()}, etc.).
   *
   * <p>Called by {@link SubsystemManager} between {@code readPeriodicInputs()} and {@code
   * writePeriodicOutputs()} every control cycle.
   */
  public final void operate() {
    switch (GlobalData.robotState) {
      case TRAVEL -> travelOperate();
      case INTAKE -> intakeOperate();
      case SCORE -> scoreOperate();
      case PASS -> passOperate();
      case CLIMB -> climbOperate();
      case CALIBRATE -> calibrateOperate();
      case SCORE_TEST -> scoreTestOperate();
      case DISABLED -> disabledOperate();
    }
  }

  // ============================================================
  // TEST MODE (Distributed Pattern)
  // ============================================================

  /**
   * Handles subsystem-specific test mode logic.
   *
   * <p>Override in subclasses to implement SysId, voltage tests, PID tuning, etc. Called by the
   * test mode handler instead of {@link #operate()}.
   *
   * @param control The ControlBoard instance for reading button inputs.
   */
  public void handleTestMode(ControlBoard control) {
    // Default: no-op. Subclasses override for test routines.
  }

  // --- Logging Infrastructure ---

  /** CSV logger instance for subsystem data. */
  protected CSVLogWriter mLogger;

  /**
   * Starts CSV logging for this subsystem.
   *
   * <p>Override to initialize and configure the logger.
   */
  public void startLogging() {
    // Default: no-op. Subsystems override if they want logging.
  }
}
