package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;

/**
 * Intake Wheel Subsystem - Single-motor roller mechanism.
 *
 * <p>This subsystem controls the intake roller used to acquire game pieces. It implements a simple
 * toggle-based state machine: the roller is either running or stopped.
 *
 * <p><strong>Control Strategy</strong>: Uses open-loop voltage control for consistent torque output
 * regardless of battery state. This is appropriate for intake mechanisms where grip force matters
 * more than precise speed control.
 *
 * <p><strong>Thread Safety</strong>: All state access is synchronized as the Looper runs on a
 * separate thread from the main robot loop.
 *
 * @see IntakeWheelIO
 */
public class IntakeWheel extends Subsystem {
  private static IntakeWheel mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global IntakeWheel instance.
   */
  public static IntakeWheel getInstance() {
    if (mInstance == null) {
      mInstance = new IntakeWheel();
    }
    return mInstance;
  }

  private final IntakeWheelIO mIO;
  private final IntakeWheelIO.IntakeWheelIOInputs mInputs = new IntakeWheelIO.IntakeWheelIOInputs();

  /**
   * Toggle state: true = roller running, false = stopped.
   *
   * <p>Synchronized access required as state changes may originate from the main robot thread while
   * the Looper reads state from a separate thread.
   */
  private boolean mIsRunning = false;

  private IntakeWheel() {
    // Use real IO implementation; swap to simulation IO if needed
    mIO = new IntakeWheelIOReal();
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (IntakeWheel.this) {
              mIsRunning = false;
            }
          }

          @Override
          public void onLoop(double timestamp) {
            // State machine logic runs here if needed
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  // --- Public API ---

  /**
   * Sets the running state of the intake roller.
   *
   * @param running True to run the roller, false to stop.
   */
  public synchronized void setRunning(boolean running) {
    mIsRunning = running;
  }

  /**
   * Toggles the intake roller state.
   *
   * <p>If currently running, stops the roller. If stopped, starts the roller.
   */
  public synchronized void toggle() {
    mIsRunning = !mIsRunning;
    System.out.println("[IntakeWheel] Toggle -> " + (mIsRunning ? "RUNNING" : "STOPPED"));
  }

  /**
   * Returns the current running state.
   *
   * @return True if the roller is running.
   */
  public synchronized boolean isRunning() {
    return mIsRunning;
  }

  // --- Subsystem Interface ---

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);
  }

  @Override
  public void writePeriodicOutputs() {
    synchronized (this) {
      if (mIsRunning) {
        mIO.setVoltage(Constants.Intake.kIntakeVoltage);
      } else {
        mIO.stop();
      }
    }
  }

  @Override
  public boolean checkConnectionActive() {
    // Active Query: Read Firmware Version from motor
    System.out.println("[IntakeWheel] Running Active Connection Check...");

    int fw = mIO.getMotorFirmwareVersion();
    if (fw == 0) {
      System.err.println("[IntakeWheel] Motor Firmware Read Failed!");
      return false;
    } else {
      System.out.println("[IntakeWheel] Motor FW: " + fw);
      return true;
    }
  }

  @Override
  public boolean checkConnectionPassive() {
    // Passive: Check cached status from IO layer
    return mInputs.motorConnected;
  }

  @Override
  public boolean checkSanityPassive() {
    // Check for current spike when running (stall detection)
    if (mInputs.currentAmps > 40.0
        || (mIsRunning && mInputs.velocityRotationsPerSec < 1 && mInputs.appliedVolts > 6)) {
      System.err.println("[IntakeWheel] Current Spike (Stall): " + mInputs.currentAmps + "A");
      return false;
    }
    return true;
  }

  @Override
  public void outputTelemetry() {
    // TODO: Publish intake state to NetworkTables if needed
  }

  @Override
  public void stop() {
    synchronized (this) {
      mIsRunning = false;
    }
    mIO.stop();
  }

  @Override
  public void zeroSensors() {
    // No sensors to zero for this simple mechanism
  }
}
