package frc.robot.framework;

/**
 * @author Team 254
 */
public abstract class Subsystem {
  public void writeToLog() {}

  // Optional: Only used if using WPILib commands alongside this framework
  // public abstract void periodic();

  public abstract void readPeriodicInputs();

  public abstract void writePeriodicOutputs();

  public abstract void checkSystem();

  public abstract void outputTelemetry();

  public abstract void stop();

  public abstract void zeroSensors();

  public abstract void registerEnabledLoops(Looper enabledLooper);

  // Logging Infrastructure
  protected CSVLogWriter mLogger;

  public void startLogging() {
    // Default: do nothing. Specific subsystems override this if they want to log.
  }
}
