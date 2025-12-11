package frc.robot.framework;

import java.util.ArrayList;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 *
 * <p>Based on Team 254's SubsystemManager.
 *
 * @author Team 254
 */
public class SubsystemManager implements ILoop {
  private static SubsystemManager mInstance = null;

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

  public void outputTelemetry() {
    mAllSubsystems.forEach((s) -> s.outputTelemetry());
  }

  public void writeToLog() {
    mAllSubsystems.forEach((s) -> s.writeToLog());
  }

  public void stop() {
    mAllSubsystems.forEach((s) -> s.stop());
  }

  public List<Subsystem> getSubsystems() {
    return mAllSubsystems;
  }

  public void setSubsystems(Subsystem... allSubsystems) {
    mAllSubsystems.clear();
    for (Subsystem s : allSubsystems) {
      mAllSubsystems.add(s);
    }
  }

  public void registerEnabledLoops(Looper enabledLooper) {
    mAllSubsystems.forEach((s) -> s.registerEnabledLoops(enabledLooper));
  }

  public void readPeriodicInputs() {
    mAllSubsystems.forEach((s) -> s.readPeriodicInputs());
  }

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
