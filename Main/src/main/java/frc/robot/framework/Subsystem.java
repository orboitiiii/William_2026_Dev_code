package frc.robot.framework;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each
 * subsystem outputs commands to SmartDashboard, has a stop routine (for after each match), and a
 * routine to zero all sensors, which helps with calibration.
 *
 * <p>All Subsystems only have one instance (after all, one robot does not have two drivetrains),
 * and functions get the instance of the subsystem and act accordingly. Subsystems are also a state
 * machine with a desired state and actual state; the robot code will try to match the two states
 * with actions. Each Subsystem also is responsible for initializing all Looper requests and
 * updating the SmartDashboard.
 *
 * <p>Based on Team 254 and Orbit 1690 architectures.
 *
 * @author Team 254
 * @author Orbit 1690
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
}
