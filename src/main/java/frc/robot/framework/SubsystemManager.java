package frc.robot.framework;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  /**
   * Dashboard chooser for selecting which subsystem to test in Test Mode.
   *
   * <p>"ALL" runs every subsystem's handleTestMode(); selecting a specific name gates the call to
   * that subsystem only and stop()s the rest.
   */
  private final SendableChooser<String> mTestChooser = new SendableChooser<>();

  private SubsystemManager() {
    mAllSubsystems = new ArrayList<>();
  }

  /** Calls {@code outputTelemetry()} on all subsystems. */
  public void outputTelemetry() {
    mAllSubsystems.forEach((s) -> s.outputTelemetry());
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
    buildTestChooser();
  }

  /**
   * Populates the test-mode SendableChooser with all registered subsystem names.
   *
   * <p>Published to SmartDashboard under "Test/Subsystem". The default option "ALL" preserves
   * backward-compatible behavior (all subsystems receive handleTestMode). Selecting a specific
   * subsystem gates the call to that subsystem only.
   */
  private void buildTestChooser() {
    // Pre-allocated at init — no allocation in periodic loops
    mTestChooser.setDefaultOption("ALL", "ALL");
    for (Subsystem s : mAllSubsystems) {
      String name = s.getClass().getSimpleName();
      mTestChooser.addOption(name, name);
    }
    SmartDashboard.putData("Test/Subsystem", mTestChooser);
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

  /** Calls {@code operate()} on all subsystems (state machine dispatch). */
  public void operate() {
    mAllSubsystems.forEach((s) -> s.operate());
  }

  /** Calls {@code handleTestMode()} on all subsystems. */
  public void handleTestMode(frc.robot.ControlBoard control) {
    mAllSubsystems.forEach((s) -> s.handleTestMode(control));
  }

  @Override
  public void onLoop(double timestamp) {
    // 1. Read Inputs (50Hz)
    for (Subsystem s : mAllSubsystems) {
      s.readPeriodicInputs();
    }

    // 2. Operate - State Machine Dispatch (Orbit 1690 Pattern)
    // Each subsystem reads GlobalData.robotState and acts accordingly
    if (frc.robot.GlobalData.isTestMode) {
      // Test mode: only the selected subsystem receives handleTestMode().
      // Unselected subsystems are stop()-ed to prevent zombie outputs.
      var control = frc.robot.ControlBoard.getInstance();
      String selected = mTestChooser.getSelected();
      SmartDashboard.putString("Test/Selected", selected == null ? "NULL" : selected);
      boolean runAll = "ALL".equals(selected);
      for (Subsystem s : mAllSubsystems) {
        if (runAll || s.getClass().getSimpleName().equals(selected)) {
          s.handleTestMode(control);
        } else {
          s.stop();
        }
      }
    } else {
      for (Subsystem s : mAllSubsystems) {
        s.operate();
      }
    }

    // 3. Write Outputs (50Hz)
    for (Subsystem s : mAllSubsystems) {
      s.writePeriodicOutputs();
    }
  }

  @Override
  public void onStop(double timestamp) {
    stop();
  }
}
