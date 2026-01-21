package frc.robot.framework;

import java.util.ArrayList;
import java.util.List;

/**
 * Central manager for all robot subsystems.
 *
 * <p>This singleton orchestrates lifecycle callbacks across all registered subsystems, ensuring
 * consistent ordering of sensor reads, output writes, and telemetry.
 *
 * <p><strong>Usage</strong>:
 *
 * <pre>{@code
 * SubsystemManager.getInstance().setSubsystems(drive, intake, shooter);
 * SubsystemManager.getInstance().registerEnabledLoops(mEnabledLooper);
 * }</pre>
 *
 * <p><strong>Attribution</strong>: Based on Team 254's SubsystemManager.
 *
 * @see Subsystem
 */
public class SubsystemManager implements ILoop {
  private static SubsystemManager mInstance = null;

  /**
   * Returns the singleton instance.
   *
   * @return The global SubsystemManager instance.
   */
  public static SubsystemManager getInstance() {
    if (mInstance == null) {
      mInstance = new SubsystemManager();
    }
    return mInstance;
  }

  private final List<Subsystem> mAllSubsystems;

  private SubsystemManager() {
    mAllSubsystems = new ArrayList<>();
  }

  /** Calls {@code outputTelemetry()} on all subsystems. */
  public void outputTelemetry() {
    mAllSubsystems.forEach((s) -> s.outputTelemetry());
  }

  /** Calls {@code writeToLog()} on all subsystems. */
  public void writeToLog() {
    mAllSubsystems.forEach((s) -> s.writeToLog());
  }

  /** Stops all subsystems. */
  public void stop() {
    mAllSubsystems.forEach((s) -> s.stop());
  }

  /**
   * Returns the list of registered subsystems.
   *
   * @return The subsystem list.
   */
  public List<Subsystem> getSubsystems() {
    return mAllSubsystems;
  }

  /**
   * Registers all subsystems with the manager.
   *
   * <p>Clears any previously registered subsystems.
   *
   * @param allSubsystems Subsystems to manage.
   */
  public void setSubsystems(Subsystem... allSubsystems) {
    mAllSubsystems.clear();
    for (Subsystem s : allSubsystems) {
      mAllSubsystems.add(s);
    }
  }

  /**
   * Registers all subsystems' loops with the enabled looper.
   *
   * @param enabledLooper The looper for enabled-mode loops.
   */
  public void registerEnabledLoops(Looper enabledLooper) {
    mAllSubsystems.forEach((s) -> s.registerEnabledLoops(enabledLooper));
  }

  /** Calls {@code readPeriodicInputs()} on all subsystems. */
  public void readPeriodicInputs() {
    mAllSubsystems.forEach((s) -> s.readPeriodicInputs());
  }

  /** Calls {@code writePeriodicOutputs()} on all subsystems. */
  public void writePeriodicOutputs() {
    mAllSubsystems.forEach((s) -> s.writePeriodicOutputs());
  }

  @Override
  public void onStart(double timestamp) {}

  @Override
  public void onLoop(double timestamp) {}

  @Override
  public void onStop(double timestamp) {
    stop();
  }
}
