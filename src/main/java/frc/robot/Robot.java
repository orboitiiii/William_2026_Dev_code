package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.TestTrajectoryMode;
import frc.robot.framework.HealthCheckLooper;
import frc.robot.framework.Looper;
import frc.robot.framework.SubsystemManager;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeWheel;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Main Robot class extending WPILib's TimedRobot framework.
 *
 * <p>This class serves as the entry point for all robot operations. It implements a Team 254-style
 * architecture with the following key components:
 *
 * <ul>
 *   <li><strong>Looper</strong>: High-frequency (100Hz) control loop executor using WPILib's
 *       Notifier for deterministic timing.
 *   <li><strong>SubsystemManager</strong>: Orchestrates lifecycle calls (readInputs, writeOutputs,
 *       telemetry) across all subsystems.
 *   <li><strong>AutoModeExecutor</strong>: Manages autonomous routines in a separate thread to
 *       avoid blocking the main loop.
 * </ul>
 *
 * <p><strong>Lifecycle Order</strong>:
 *
 * <pre>
 * robotInit() -> [Mode]Init() -> robotPeriodic() + [Mode]Periodic() -> disabledInit()
 * </pre>
 *
 * @see Looper
 * @see SubsystemManager
 */
public class Robot extends TimedRobot {
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  private final Drive mDrive = Drive.getInstance();
  private final Superstructure mSuperstructure = Superstructure.getInstance();
  private final IntakeWheel mIntakeWheel = IntakeWheel.getInstance();
  private final IntakePivot mIntakePivot = IntakePivot.getInstance();
  private final Indexer mIndexer = Indexer.getInstance();
  private final Shooter mShooter = Shooter.getInstance();

  private final DashboardState mDashboard = DashboardState.getInstance();

  /** Operator control interface. */
  public final ControlBoard control = ControlBoard.getInstance();

  private final AutoModeExecutor mAutoModeExecutor = AutoModeExecutor.getInstance();

  /**
   * Flag indicating Test Mode is active.
   *
   * <p>When true, prevents {@link #robotPeriodic()} from calling writePeriodicOutputs, allowing
   * direct motor control for calibration without state machine interference.
   */
  private boolean mIsInTestMode = false;

  /**
   * Constructs the Robot and registers all subsystems with the framework.
   *
   * <p>This is called once when the robot code is first loaded. All subsystem instances should be
   * acquired here to ensure consistent initialization order.
   */
  public Robot() {
    mSubsystemManager.setSubsystems(
        mDrive, mSuperstructure, mIntakeWheel, mIntakePivot, mIndexer, mShooter);
    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    mEnabledLooper.register(mSubsystemManager);
  }

  /**
   * Called once when robot code first starts.
   *
   * <p>Performs JIT warmup to prevent first-loop latency spikes and zeros all subsystem sensors to
   * establish a known initial state.
   */
  @Override
  public void robotInit() {
    SystemWarmup.warmup();
    mSubsystemManager.getSubsystems().forEach(s -> s.zeroSensors());

    // Run Active Connection Checks (Firmware) once at startup
    HealthCheckLooper.getInstance().runActiveChecks();

    // Start 1Hz Passive Health Check loop
    HealthCheckLooper.getInstance().start();
  }

  /**
   * Called every 20ms regardless of robot mode.
   *
   * <p>Handles:
   *
   * <ul>
   *   <li>Sensor reading (always)
   *   <li>Telemetry output (always)
   *   <li>Actuator output (disabled in Test Mode)
   *   <li>Dashboard state publication
   * </ul>
   */
  @Override
  public void robotPeriodic() {
    mSubsystemManager.readPeriodicInputs();
    mSubsystemManager.outputTelemetry();

    // Test Mode requires direct motor control; skip state machine outputs
    if (!mIsInTestMode) {
      mSubsystemManager.writePeriodicOutputs();
    }

    mDashboard.matchTime = Timer.getMatchTime();

    // Read game-specific data from FMS and encode as byte (0=unknown, 1=Red,
    // 2=Blue)
    // Data becomes available ~3s after Auto ends; until then it's an empty string.
    String rawGameData = DriverStation.getGameSpecificMessage();
    if (rawGameData != null && rawGameData.length() > 0) {
      switch (rawGameData.charAt(0)) {
        case 'R':
          mDashboard.gameData = 1;
          break;
        case 'B':
          mDashboard.gameData = 2;
          break;
        default:
          // Corrupt data received - treat as unknown
          mDashboard.gameData = 0;
          break;
      }
    } else {
      // No data yet or null - send unknown
      mDashboard.gameData = 0;
    }

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      mDashboard.isRedAlliance = alliance.get() == DriverStation.Alliance.Red;
    } else {
      mDashboard.isRedAlliance = false;
    }

    mDashboard.publish();
  }

  /**
   * Called once when Teleop mode is enabled.
   *
   * <p>Starts the enabled looper and stops the disabled looper.
   */
  @Override
  public void teleopInit() {
    mIsInTestMode = false;
    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStart(Timer.getFPGATimestamp());
  }

  /**
   * Called every 20ms during Teleop mode.
   *
   * <p>Processes operator inputs and forwards them to the Drive subsystem.
   */
  @Override
  public void teleopPeriodic() {
    if (control.getZeroGyro()) {
      mDrive.zeroSensors();
    }

    mDrive.setTeleopInputs(control.getTranslation(), control.getRotation(), true);

    // Intake Toggle: R1 button toggles roller on/off
    if (control.getIntakeToggle()) {
      mIntakeWheel.toggle();
    }

    // Intake Pivot: R1 = forward, L1 = reverse, neither = stop
    if (control.getPivotForward()) {
      mIntakePivot.setOpenLoopVoltage(Constants.IntakePivot.kManualVoltage);
    } else if (control.getPivotReverse()) {
      mIntakePivot.setOpenLoopVoltage(-Constants.IntakePivot.kManualVoltage);
    } else {
      // Neither button pressed - hold current position (closed-loop)
      mIntakePivot.setOpenLoopVoltage(0);
    }

    // Indexer: Square button held = run both rollers
    mIndexer.setRunning(control.getIndexerButton());

    // Shooter: Cross button held = run shooter
    if (control.getShooterButton()) {
      mShooter.runVelocityControl();
    } else {
      mShooter.stopVelocityControl();
    }
  }

  /**
   * Called once when Autonomous mode is enabled.
   *
   * <p>Starts the looper infrastructure and launches the autonomous routine in a separate thread
   * via {@link AutoModeExecutor}.
   */
  @Override
  public void autonomousInit() {
    mIsInTestMode = false;
    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStart(Timer.getFPGATimestamp());

    mAutoModeExecutor.setAutoMode(new TestTrajectoryMode("HubToLeftGround.csv"));
    mAutoModeExecutor.start();

    System.out.println("[Robot] Autonomous mode started");
  }

  /** Called every 20ms during Autonomous mode. Currently no-op; logic runs in Looper. */
  @Override
  public void autonomousPeriodic() {}

  /**
   * Called once when the robot is disabled.
   *
   * <p>Stops all loopers and the autonomous executor to ensure safe shutdown.
   */
  @Override
  public void disabledInit() {
    mIsInTestMode = false;
    mEnabledLooper.onStop(Timer.getFPGATimestamp());
    mDisabledLooper.onStart(Timer.getFPGATimestamp());

    mAutoModeExecutor.stop();
    mSubsystemManager.stop();
  }

  /** Called every 20ms while disabled. Currently no-op. */
  @Override
  public void disabledPeriodic() {}

  /**
   * Called once when Test mode is enabled.
   *
   * <p><strong>Design Rationale</strong>: Test Mode is now dedicated to Shooter SysId
   * characterization. Other Test Mode features (Swerve calibration, Odometry characterizer) are
   * temporarily disabled to avoid button conflicts.
   */
  @Override
  public void testInit() {
    mIsInTestMode = true;

    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStop(Timer.getFPGATimestamp());

    mDrive.stop();
    mShooter.stop();

    System.out.println("========================================");
    System.out.println("[Robot] Test Mode Enabled - SHOOTER SYSID");
    System.out.println("Hold button to run test, release to stop & save:");
    System.out.println("  Triangle: Quasistatic Forward");
    System.out.println("  Circle:   Quasistatic Reverse");
    System.out.println("  Square:   Dynamic Forward");
    System.out.println("  Cross:    VELOCITY CONTROL (35 rot/s)");
    System.out.println("========================================");
  }

  // Track which SysId test was started to detect button release
  private boolean mSysIdButtonWasPressed = false;

  /**
   * Called every 20ms during Test mode.
   *
   * <p>Dedicated to Shooter SysId characterization. All other Test Mode features are disabled.
   */
  @Override
  public void testPeriodic() {
    // Read inputs for Shooter telemetry
    mShooter.readPeriodicInputs();

    // Determine if any SysId button is currently pressed (excluding Cross for
    // velocity control)
    boolean anySysIdButtonPressed =
        control.getSysIdQuasistaticForward()
            || control.getSysIdQuasistaticReverse()
            || control.getSysIdDynamicForward();

    // Cross button = Velocity Control Mode
    boolean crossPressed = control.getSysIdDynamicReverse();

    // Start SysId test on button press (only if not already running)
    if (!mShooter.isSysIdActive() && !mShooter.isVelocityControlActive()) {
      if (control.getSysIdQuasistaticForward()) {
        mShooter.startSysId(
            frc.robot.framework.SysIdRoutine.TestType.QUASISTATIC,
            frc.robot.framework.SysIdRoutine.Direction.FORWARD);
      } else if (control.getSysIdQuasistaticReverse()) {
        mShooter.startSysId(
            frc.robot.framework.SysIdRoutine.TestType.QUASISTATIC,
            frc.robot.framework.SysIdRoutine.Direction.REVERSE);
      } else if (control.getSysIdDynamicForward()) {
        mShooter.startSysId(
            frc.robot.framework.SysIdRoutine.TestType.DYNAMIC,
            frc.robot.framework.SysIdRoutine.Direction.FORWARD);
      }
    }

    // Update SysId (calculates voltage and applies it)
    mShooter.updateSysId();

    // Velocity Control: Cross button held = run at target velocity
    if (crossPressed) {
      mShooter.runVelocityControl();
    } else {
      mShooter.stopVelocityControl();
    }

    // Stop SysId test when button is released
    if (mSysIdButtonWasPressed && !anySysIdButtonPressed) {
      mShooter.stopSysId();
    }

    mSysIdButtonWasPressed = anySysIdButtonPressed;

    // Publish Shooter velocity to SmartDashboard for monitoring
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Shooter/Velocity_RPS", mShooter.getAverageVelocity());

    // --- DISABLED: Other Test Mode Features ---
    // Uncomment these to re-enable after SysId is complete.

    /*
     * // Calibration: Press A to calibrate swerve offsets
     * if (control.getCalibrateButton()) {
     * System.out.
     * println("[Robot] Calibration button pressed, starting calibration...");
     * mDrive.executeCalibrationDirect();
     * }
     *
     * // Diagnostic drive: Use joysticks to drive and monitor errors
     * double vx = control.getTranslation().getX() *
     * Constants.Swerve.kMaxDriveVelocity;
     * double vy = control.getTranslation().getY() *
     * Constants.Swerve.kMaxDriveVelocity;
     * double omega = control.getRotation() * Constants.Swerve.kMaxAngularVelocity;
     * mDrive.runDiagnosticDrive(vx, vy, omega);
     * mDrive.publishTestDiagnostics();
     *
     * // Legacy: Y button for simple drive motor spin test
     * if (control.getTestDriveButton()) {
     * mDrive.setAllDriveMotorsDutyCycle(0.05);
     * }
     *
     * // --- Odometry Characterization ---
     * OdometryCharacterizer odoChar = OdometryCharacterizer.getInstance();
     * if (control.getRecordingToggle()) {
     * if (odoChar.isRecording()) {
     * odoChar.cancelRecording();
     * System.out.println("[OdoChar] Recording cancelled");
     * } else {
     * odoChar.startRecording();
     * System.out.
     * println("[OdoChar] Recording started - drive robot then return to origin");
     * }
     * }
     * if (odoChar.isRecording()) {
     * odoChar.update(mDrive.getInputs(), mDrive.getFusedPose());
     * }
     * if (control.getEndTrialButton()) {
     * if (odoChar.isRecording()) {
     * var error = odoChar.endTrialAndReset(mDrive.getFusedPose());
     * mDrive.zeroSensors();
     * System.out.printf(
     * "[OdoChar] Trial complete! Error: (%.3fm, %.3fm, %.1f°)%n",
     * error.dx(), error.dy(), Math.toDegrees(error.dTheta()));
     * }
     * }
     * if (control.getExportButton()) {
     * String filename = "odom_trials_" + System.currentTimeMillis();
     * odoChar.exportAllTrials(filename);
     * }
     */
  }

  /**
   * Called once when exiting Test mode.
   *
   * <p>Resets the Test Mode flag to restore normal operation.
   */
  @Override
  public void testExit() {
    mIsInTestMode = false;
    mShooter.stopSysId(); // Ensure SysId is stopped
    System.out.println("[Robot] Exiting Test Mode");
  }
}
