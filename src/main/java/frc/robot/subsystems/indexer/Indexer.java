package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.ControlBoard;
import frc.robot.framework.CSVLogWriter;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.framework.SysIdRoutine;

/**
 * Indexer Subsystem - Dual-motor game piece transfer mechanism.
 *
 * <p>This subsystem controls the internal indexer used to transfer game pieces from the intake to
 * the shooter/scoring mechanism. It consists of two Krakon X44 motors:
 *
 * <ul>
 *   <li><strong>Side Roller</strong>: Transfers game pieces laterally.
 *   <li><strong>Straight Roller</strong>: Transfers game pieces in a linear path.
 * </ul>
 *
 * <p><strong>Control Strategy</strong>: Uses open-loop voltage control for consistent torque output
 * regardless of battery state. Both motors receive the same voltage command to ensure synchronized
 * operation.
 *
 * <p><strong>Thread Safety</strong>: All state access is synchronized as the Looper runs on a
 * separate thread from the main robot loop.
 *
 * @see IndexerIO
 */
public class Indexer extends Subsystem {
  private static Indexer mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global Indexer instance.
   */
  public static Indexer getInstance() {
    if (mInstance == null) {
      mInstance = new Indexer();
    }
    return mInstance;
  }

  private final IndexerIO mIO;
  private final IndexerIO.IndexerIOInputs mInputs = new IndexerIO.IndexerIOInputs();

  /**
   * Running state: true = rollers running, false = stopped.
   *
   * <p>Synchronized access required as state changes may originate from the main robot thread while
   * the Looper reads state from a separate thread.
   */
  private boolean mSideRollerRunning = false;

  private boolean mStraightRollerRunning = false;

  /**
   * Target spinner velocity [rot/s] for the side roller during scoring. Dynamically capped by
   * ShotTables.maxSpinnerRps(distance) to prevent mid-air ball collisions. Negative because the
   * roller direction is inverted. Preallocated to avoid GC in the control loop.
   */
  private double mSideRollerTargetRps = frc.robot.Constants.Indexer.kSideRollerTargetVelocity;

  // --- SysId Integration ---
  private final SysIdRoutine mSideSysIdRoutine;
  private final SysIdRoutine mStraightSysIdRoutine;
  private boolean mSideSysIdActive = false;
  private boolean mStraightSysIdActive = false;

  private Indexer() {
    // Use real IO implementation; swap to simulation IO if needed
    mIO = new IndexerIOReal();

    // Initialize SysId routines
    mSideSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config()
                .setSubsystemName("Indexer_Side")
                .setRampRate(Volts.of(1.0))
                .setStepVoltage(Volts.of(4.0))
                .setTimeout(Seconds.of(10.0))
                .setLogWriter(new CSVLogWriter("sysid_Indexer_Side")));

    mStraightSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config()
                .setSubsystemName("Indexer_Straight")
                .setRampRate(Volts.of(1.0))
                .setStepVoltage(Volts.of(4.0))
                .setTimeout(Seconds.of(10.0))
                .setLogWriter(new CSVLogWriter("sysid_Indexer_Straight")));
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Indexer.this) {
              mSideRollerRunning = false;
              mStraightRollerRunning = false;
              stopSysId();
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
   * Sets the running state of the indexer (both rollers).
   *
   * @param running True to run the indexer, false to stop.
   */
  public synchronized void setRunning(boolean running) {
    mSideRollerRunning = running;
    mStraightRollerRunning = running;
  }

  /**
   * Sets the running state of the Side Roller.
   *
   * @param running True to run the side roller, false to stop.
   */
  public synchronized void setSideRollerRunning(boolean running) {
    mSideRollerRunning = running;
  }

  /**
   * Sets the running state of the Straight Roller.
   *
   * @param running True to run the straight roller, false to stop.
   */
  public synchronized void setStraightRollerRunning(boolean running) {
    mStraightRollerRunning = running;
  }

  /**
   * Toggles the indexer state (both rollers).
   *
   * <p>If either is running, stops both. If neither is running, starts both.
   */
  public synchronized void toggle() {
    boolean anyRunning = mSideRollerRunning || mStraightRollerRunning;
    if (anyRunning) {
      mSideRollerRunning = false;
      mStraightRollerRunning = false;
    } else {
      mSideRollerRunning = true;
      mStraightRollerRunning = true;
    }
  }

  /**
   * Returns true if either roller is running.
   *
   * @return True if active.
   */
  public synchronized boolean isRunning() {
    return mSideRollerRunning || mStraightRollerRunning;
  }

  public synchronized boolean isSideRollerRunning() {
    return mSideRollerRunning;
  }

  public synchronized boolean isStraightRollerRunning() {
    return mStraightRollerRunning;
  }

  // ============================================================
  // PER-STATE OPERATE METHODS (Orbit 1690 Pattern)
  // ============================================================

  @Override
  public void travelOperate() {
    mStraightRollerRunning = false;
    mSideRollerRunning = false;
  }

  @Override
  public void intakeOperate() {
    // Indexer idle during intake (only intake wheel runs)
    mStraightRollerRunning = false;
    mSideRollerRunning = false;
  }

  @Override
  public void scoreOperate() {
    mStraightRollerRunning = true;
    // Side roller only when shooter, turret, and hood are all at target
    var shooter = frc.robot.subsystems.shooter.Shooter.getInstance();
    var turret = frc.robot.subsystems.turret.Turret.getInstance();
    var hood = frc.robot.subsystems.hood.Hood.getInstance();

    double currentVel = shooter.getAverageVelocity();
    var params = frc.robot.GlobalData.currentShotParams;

    if (params != null && params.isValid) {
      double error = Math.abs(params.flywheelSpeedRotPerSec - currentVel);

      // Strict conditions for feeding ball into the shooter:
      // 1. Flywheel within 5.0 RPS of target
      // 2. Turret is at goal (aimed correctly)
      // 3. Hood is at goal (elevated correctly)
      boolean speedLocked = (error < 5.0);
      boolean turretLocked = turret.isAtGoal();
      boolean hoodLocked = hood.isAtGoal();

      mSideRollerRunning = speedLocked && turretLocked && hoodLocked;

      // Anti-collision: cap spinner RPS based on distance to prevent
      // mid-air ball collisions at close range (high launch angle = low
      // horizontal velocity at apex = balls stack up).
      // Uses critical BPS from trajectory apex minimum speed analysis.
      // Reference: https://www.chiefdelphi.com/t/512598
      // double maxRps =
      // frc.robot.subsystems.shooter.ShotTables.maxSpinnerRps(params.effectiveDistanceMeters());
      // double baseRps = frc.robot.Constants.Indexer.kSideRollerTargetVelocity;
      // // Both are negative (inverted roller direction), so use max() to cap
      // magnitude
      // mSideRollerTargetRps = Math.max(baseRps, -maxRps);
      mSideRollerTargetRps = frc.robot.Constants.Indexer.kSideRollerTargetVelocity;
    } else {
      mSideRollerRunning = false;
      mSideRollerTargetRps = frc.robot.Constants.Indexer.kSideRollerTargetVelocity;
    }
  }

  @Override
  public void passOperate() {
    setRunning(true);
  }

  @Override
  public void climbOperate() {
    mStraightRollerRunning = false;
    mSideRollerRunning = false;
  }

  @Override
  public void calibrateOperate() {
    mStraightRollerRunning = false;
    mSideRollerRunning = false;
  }

  @Override
  public void scoreTestOperate() {
    mStraightRollerRunning = true;

    // Side roller only when shooter is at target speed
    var shooter = frc.robot.subsystems.shooter.Shooter.getInstance();
    double currentVel = shooter.getAverageVelocity();
    double targetVel = 44;

    // Check if within 1.0 RPS of target AND Hood is at goal
    var hood = frc.robot.subsystems.hood.Hood.getInstance();
    if (Math.abs(currentVel - targetVel) < 1.0 && hood.isAtGoal()) {
      mSideRollerRunning = true;
    } else {
      mSideRollerRunning = false;
    }
  }

  // --- SysId API (Test Mode Only) ---

  public synchronized void startSideSysId(
      SysIdRoutine.TestType type, SysIdRoutine.Direction direction) {
    mSideSysIdActive = true;
    mSideSysIdRoutine.start(type, direction);
  }

  public synchronized void startStraightSysId(
      SysIdRoutine.TestType type, SysIdRoutine.Direction direction) {
    mStraightSysIdActive = true;
    mStraightSysIdRoutine.start(type, direction);
  }

  public synchronized void stopSysId() {
    if (mSideSysIdActive) {
      mSideSysIdRoutine.stop();
      mSideSysIdActive = false;
      mIO.setSideRollerVoltage(0.0);
    }
    if (mStraightSysIdActive) {
      mStraightSysIdRoutine.stop();
      mStraightSysIdActive = false;
      mIO.setStraightRollerVoltage(0.0);
    }
  }

  public synchronized void updateSysId() {
    double timestamp = Timer.getFPGATimestamp();

    if (mSideSysIdActive) {
      mSideSysIdRoutine.update(
          timestamp, mInputs.sideRollerPositionRot, mInputs.sideRollerVelocityRotPerSec);
    }

    if (mStraightSysIdActive) {
      mStraightSysIdRoutine.update(
          timestamp, mInputs.straightRollerPositionRot, mInputs.straightRollerVelocityRotPerSec);
    }
  }

  public synchronized boolean isSysIdActive() {
    return mSideSysIdActive || mStraightSysIdActive;
  }

  // ============================================================
  // TEST MODE
  // ============================================================

  private boolean mSysIdButtonWasPressed = false;

  @Override
  public void handleTestMode(ControlBoard control) {
    boolean anyButton = control.getSquareButton() || control.getCircleButton();

    if (!isSysIdActive()) {
      if (control.getR1Button()) {
        // Side Roller Tests (Reverse Only)
        if (control.getSquareButton()) {
          startSideSysId(SysIdRoutine.TestType.QUASISTATIC, SysIdRoutine.Direction.REVERSE);
        } else if (control.getCircleButton()) {
          startSideSysId(SysIdRoutine.TestType.DYNAMIC, SysIdRoutine.Direction.REVERSE);
        }
      } else if (control.getL1Button()) {
        // Straight Roller Tests (Reverse Only)
        if (control.getSquareButton()) {
          startStraightSysId(SysIdRoutine.TestType.QUASISTATIC, SysIdRoutine.Direction.REVERSE);
        } else if (control.getCircleButton()) {
          startStraightSysId(SysIdRoutine.TestType.DYNAMIC, SysIdRoutine.Direction.REVERSE);
        }
      }
    }

    updateSysId();

    if (mSysIdButtonWasPressed && !anyButton) {
      stopSysId();
    }
    mSysIdButtonWasPressed = anyButton;
  }

  // --- Subsystem Interface ---

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);
  }

  @Override
  public void writePeriodicOutputs() {
    synchronized (this) {
      if (mSideSysIdActive) {
        mIO.setSideRollerVoltage(mSideSysIdRoutine.getOutputVoltage());
      } else if (mSideRollerRunning) {
        mIO.setSideRollerTargetVelocity(mSideRollerTargetRps);
      } else {
        mIO.setSideRollerVoltage(0.0);
      }

      if (mStraightSysIdActive) {
        mIO.setStraightRollerVoltage(mStraightSysIdRoutine.getOutputVoltage());
      } else if (mStraightRollerRunning) {
        mIO.setStraightRollerTargetVelocity(
            frc.robot.Constants.Indexer.kStraightRollerTargetVelocity);
      } else {
        mIO.setStraightRollerVoltage(0.0);
      }
    }
  }

  @Override
  public boolean checkConnectionActive() {
    int sideRollerFw = mIO.getSideRollerFirmwareVersion();
    int straightRollerFw = mIO.getStraightRollerFirmwareVersion();

    return sideRollerFw != 0 && straightRollerFw != 0;
  }

  @Override
  public boolean checkConnectionPassive() {
    // Passive: Check cached status from IO layer
    return mInputs.sideRollerConnected && mInputs.straightRollerConnected;
  }

  @Override
  public boolean checkSanityPassive() {
    // Check for current spike when running (stall detection)
    double totalCurrent = mInputs.sideRollerCurrentAmps + mInputs.straightRollerCurrentAmps;

    if (totalCurrent > 80.0) {
      return false;
    }

    // Check individual motor stall conditions
    if (mSideRollerRunning
        && mInputs.sideRollerVelocityRotPerSec < 1
        && mInputs.sideRollerAppliedVolts > 6) {
      return false;
    }

    if (mStraightRollerRunning
        && mInputs.straightRollerVelocityRotPerSec < 1
        && mInputs.straightRollerAppliedVolts > 6) {
      return false;
    }

    return true;
  }

  @Override
  public void outputTelemetry() {
    frc.robot.DashboardState.getInstance().indexerOK =
        checkConnectionPassive() && checkSanityPassive();
  }

  @Override
  public void stop() {
    synchronized (this) {
      mSideRollerRunning = false;
      mStraightRollerRunning = false;
    }
    stopSysId();
    mIO.stop();
  }

  @Override
  public void zeroSensors() {
    // No sensors to zero for this simple mechanism
  }
}
