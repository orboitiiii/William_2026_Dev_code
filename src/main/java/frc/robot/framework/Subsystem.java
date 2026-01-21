package frc.robot.framework;

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
 *   <li>State Machine Logic (in Looper)
 *   <li>{@link #writePeriodicOutputs()}: Apply actuator commands
 *   <li>{@link #outputTelemetry()}: Publish to dashboard
 * </ol>
 *
 * <p><strong>Attribution</strong>: Based on Team 254's Subsystem architecture.
 *
 * @see SubsystemManager
 */
public abstract class Subsystem {
  /**
   * Writes diagnostic information to log files.
   *
   * <p>Override to implement subsystem-specific logging.
   */
  public void writeToLog() {}

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
