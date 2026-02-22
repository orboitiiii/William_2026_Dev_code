package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.framework.HealthCheckLooper;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.framework.SubsystemManager;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeWheel;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShotCalculator;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;

/**
 * Main Robot class extending WPILib's TimedRobot framework.
 *
 * <p>Architecture: Orbit 1690 Global State Machine Pattern.
 *
 * <p>Each control cycle follows this flow:
 *
 * <ol>
 *   <li>{@link StateResolver#getRobotState()} determines the global {@link RobotState}
 *   <li>Shared data is updated in {@link GlobalData} (e.g., ShotCalculator results)
 *   <li>SubsystemManager calls: readPeriodicInputs → operate → writePeriodicOutputs
 *   <li>Each subsystem's operate() dispatches to per-state methods (travelOperate, scoreOperate,
 *       etc.)
 * </ol>
 *
 * <p>Auto and Teleop share the same loop. The only difference is how {@link GlobalData#robotState}
 * is determined (buttons vs auto state machine).
 *
 * @see StateResolver
 * @see GlobalData
 * @see RobotState
 */
public class Robot extends TimedRobot {
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  private final Drive mDrive = Drive.getInstance();
  private final IntakeWheel mIntakeWheel = IntakeWheel.getInstance();
  private final IntakePivot mIntakePivot = IntakePivot.getInstance();
  private final Indexer mIndexer = Indexer.getInstance();
  private final Shooter mShooter = Shooter.getInstance();
  private final Hood mHood = Hood.getInstance();
  private final Turret mTurret = Turret.getInstance();
  private final VisionSubsystem mVisionSubsystem = VisionSubsystem.getInstance();
  private final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

  private final DashboardState mDashboard = DashboardState.getInstance();

  private AutoModeExecutor mAutoModeExecutor = AutoModeExecutor.getInstance();

  public Robot() {
    // Register all subsystems — conditionally include Hood/Turret based on feature
    // flags
    List<Subsystem> subsystems =
        new ArrayList<>(
            List.of(
                mDrive,
                mShooter,
                mIndexer,
                mRobotStateEstimator,
                mIntakeWheel,
                mIntakePivot,
                mVisionSubsystem,
                mHood,
                mTurret));
    // if (Constants.kHasHood)
    // subsystems.add(Hood.getInstance());
    // if (Constants.kHasTurret)
    // subsystems.add(Turret.getInstance());
    mSubsystemManager.setSubsystems(subsystems.toArray(new Subsystem[0]));
    mSubsystemManager.registerEnabledLoops(mEnabledLooper);

    // Register the state resolver loop BEFORE SubsystemManager
    // This ensures GlobalData.robotState is updated before subsystems operate
    mEnabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {}

          @Override
          public void onLoop(double timestamp) {
            if (GlobalData.isTestMode) {
              return;
            } else if (GlobalData.isAutonomous) {
              // Auto: state determined by auto state machine
              // (handled by AutoStateMachine when implemented)
            } else {
              // Teleop: state determined by joystick buttons
              GlobalData.robotState = StateResolver.getRobotState();
            }

            // 2. Update shared data based on current state
            if (GlobalData.robotState == RobotState.SCORE) {
              ShotCalculator.getInstance().clearCache();
              GlobalData.currentShotParams = ShotCalculator.getInstance().calculate();
            } else {
              GlobalData.currentShotParams = null;
            }
          }

          @Override
          public void onStop(double timestamp) {}
        });

    mEnabledLooper.register(mSubsystemManager);
    mDisabledLooper.register(mSubsystemManager);
  }

  @Override
  public void robotInit() {
    SignalLogger.enableAutoLogging(false);
    mSubsystemManager.getSubsystems().forEach(s -> s.zeroSensors());

    // Disabled active health checks to prevent 6s loop overrun in robotInit
    // Passive checks in HealthCheckLooper (1Hz) are sufficient.
    // HealthCheckLooper.getInstance().runActiveChecks();
    HealthCheckLooper.getInstance().start();

    // FORCE initialization of the disabled state.
    // WPILib's TimedRobot delays calling disabledInit() until the first iteration
    // or driver station connection, which prevents mDisabledLooper from starting.
    disabledInit();
  }

  @Override
  public void robotPeriodic() {
    mSubsystemManager.outputTelemetry();

    mDashboard.matchTime = Timer.getMatchTime();

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
          mDashboard.gameData = 0;
          break;
      }
    } else {
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

  @Override
  public void teleopInit() {
    GlobalData.isTestMode = false;
    GlobalData.isAutonomous = false;
    GlobalData.robotState = RobotState.TRAVEL;

    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStart(Timer.getFPGATimestamp());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void autonomousInit() {
    GlobalData.isTestMode = false;
    GlobalData.isAutonomous = true;
    GlobalData.robotState = RobotState.TRAVEL;

    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStart(Timer.getFPGATimestamp());

    // MPC Trajectory auto can be started here:
    // mAutoModeExecutor.setAutoMode(new
    // TestTrajectoryMode("path_points_1769604165709.csv"));
    // mAutoModeExecutor.start();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void disabledInit() {
    GlobalData.isTestMode = false;
    GlobalData.isAutonomous = false;
    GlobalData.robotState = RobotState.DISABLED;

    mEnabledLooper.onStop(Timer.getFPGATimestamp());
    mDisabledLooper.onStart(Timer.getFPGATimestamp());

    mSubsystemManager.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    GlobalData.isTestMode = true;
    GlobalData.isAutonomous = false;

    mDisabledLooper.onStop(Timer.getFPGATimestamp());

    // CRITICAL: Set OPEN_LOOP state BEFORE starting the Looper
    // This prevents any PID control from running in the first loop iteration
    mDrive.stopOpenLoop();
    mIntakePivot.stop();

    // NOW start the main loop
    // SubsystemManager will call handleTestMode() on each subsystem
    mEnabledLooper.onStart(Timer.getFPGATimestamp());
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    GlobalData.isTestMode = false;
  }
}
