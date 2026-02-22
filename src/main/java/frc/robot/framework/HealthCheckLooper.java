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

      for (Subsystem s : SubsystemManager.getInstance().getSubsystems()) {
        s.checkConnectionActive();
      }
      mActiveCheckCompleted = true;
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
        mRunning = true;
        mNotifier.startPeriodic(kPeriod);
      }
    }
  }

  /** Stops the passive check loop. */
  public void stop() {
    synchronized (mLock) {
      if (mRunning) {
        mNotifier.stop();
        mRunning = false;
      }
    }
  }

  /** Periodic runnable executed by the Notifier. */
  private void runChecks() {
    frc.robot.DashboardState dashboard = frc.robot.DashboardState.getInstance();

    // Drive Subsystem Check
    var drive = frc.robot.subsystems.drive.Drive.getInstance();
    dashboard.driveOK = drive.checkConnectionPassive() && drive.checkSanityPassive();

    // Turret Subsystem Check
    if (frc.robot.Constants.kHasTurret) {
      var turret = frc.robot.subsystems.turret.Turret.getInstance();
      dashboard.turretOK = turret.checkConnectionPassive() && turret.checkSanityPassive();
    }

    // Hood Subsystem Check
    if (frc.robot.Constants.kHasHood) {
      var hood = frc.robot.subsystems.hood.Hood.getInstance();
      dashboard.hoodOK = hood.checkConnectionPassive() && hood.checkSanityPassive();
    }

    // Shooter Subsystem Check
    var shooter = frc.robot.subsystems.shooter.Shooter.getInstance();
    dashboard.shooterOK = shooter.checkConnectionPassive() && shooter.checkSanityPassive();

    // Intake Wheels Subsystem Check
    var intakeWheel = frc.robot.subsystems.intake.IntakeWheel.getInstance();
    dashboard.intakeWheelsOK =
        intakeWheel.checkConnectionPassive() && intakeWheel.checkSanityPassive();

    // Intake Pivot Subsystem Check
    var intakePivot = frc.robot.subsystems.intake.IntakePivot.getInstance();
    dashboard.intakePivotOK =
        intakePivot.checkConnectionPassive() && intakePivot.checkSanityPassive();

    // Indexer Subsystem Check
    var indexer = frc.robot.subsystems.indexer.Indexer.getInstance();
    dashboard.indexerOK = indexer.checkConnectionPassive() && indexer.checkSanityPassive();

    // Vision Subsystem Check (Front Limelight)
    var vision = frc.robot.subsystems.vision.VisionSubsystem.getInstance();
    dashboard.frontLLOK = vision.checkConnectionPassive() && vision.checkSanityPassive();
  }
}
