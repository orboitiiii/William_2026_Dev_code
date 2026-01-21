package frc.robot.framework;

import edu.wpi.first.wpilibj.Notifier;

/**
 * Health Check Looper - 1Hz Independent Thread for System Diagnostics.
 *
 * <p>Runs passive connection and sanity checks on all registered subsystems at 1Hz (once per
 * second) to minimize overhead while providing timely fault detection.
 *
 * <p><strong>Thread Safety</strong>: Subsystem check methods must be synchronized as this runs in a
 * separate thread from the main robot loop.
 *
 * @see Subsystem#checkConnectionPassive()
 * @see Subsystem#checkSanityPassive()
 */
public class HealthCheckLooper {
  /** Check period in seconds (1Hz = 1.0s). */
  public static final double kPeriod = 1.0;

  private static HealthCheckLooper mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global HealthCheckLooper instance.
   */
  public static synchronized HealthCheckLooper getInstance() {
    if (mInstance == null) {
      mInstance = new HealthCheckLooper();
    }
    return mInstance;
  }

  private final Notifier mNotifier;
  private final Object mLock = new Object();
  private boolean mRunning = false;
  private boolean mActiveCheckCompleted = false;

  private HealthCheckLooper() {
    mNotifier = new Notifier(this::runChecks);
  }

  /**
   * Runs the active connection check once at initialization.
   *
   * <p>This performs heavy CAN queries (Firmware version, etc.) and should only be called once
   * after robot connection is established.
   */
  public void runActiveChecks() {
    synchronized (mLock) {
      if (mActiveCheckCompleted) {
        return;
      }

      System.out.println("[HealthCheck] Running Active Connection Checks...");
      for (Subsystem s : SubsystemManager.getInstance().getSubsystems()) {
        boolean result = s.checkConnectionActive();
        if (!result) {
          System.err.println(
              "[HealthCheck] Active check failed for: " + s.getClass().getSimpleName());
        }
      }
      mActiveCheckCompleted = true;
      System.out.println("[HealthCheck] Active Connection Checks Complete.");
    }
  }

  /**
   * Starts the 1Hz passive check loop.
   *
   * <p>Should be called after robot initialization.
   */
  public void start() {
    synchronized (mLock) {
      if (!mRunning) {
        System.out.println("[HealthCheck] Starting 1Hz Passive Checks...");
        mRunning = true;
        mNotifier.startPeriodic(kPeriod);
      }
    }
  }

  /** Stops the passive check loop. */
  public void stop() {
    synchronized (mLock) {
      if (mRunning) {
        System.out.println("[HealthCheck] Stopping Passive Checks.");
        mNotifier.stop();
        mRunning = false;
      }
    }
  }

  /** Periodic runnable executed by the Notifier. */
  private void runChecks() {
    frc.robot.DashboardState dashboard = frc.robot.DashboardState.getInstance();

    // Drive Subsystem Check
    frc.robot.subsystems.drive.Drive drive = frc.robot.subsystems.drive.Drive.getInstance();
    boolean driveOk = drive.checkConnectionPassive() && drive.checkSanityPassive();
    dashboard.driveHealth = driveOk ? (byte) 1 : (byte) 0;

    // IntakeWheel Subsystem Check
    frc.robot.subsystems.intake.IntakeWheel intake =
        frc.robot.subsystems.intake.IntakeWheel.getInstance();
    boolean intakeOk = intake.checkConnectionPassive() && intake.checkSanityPassive();
    dashboard.intakeHealth = intakeOk ? (byte) 1 : (byte) 0;
  }
}
