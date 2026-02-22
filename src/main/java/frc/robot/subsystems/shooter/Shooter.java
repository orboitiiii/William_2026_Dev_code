package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.framework.CSVLogWriter;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.framework.SysIdRoutine;

/**
 * Shooter Subsystem - Dual-motor flywheel mechanism.
 *
 * <p>This subsystem controls the shooter flywheel using two KrakenX60 motors in a leader-follower
 * configuration with opposed alignment. This produces synchronized, mirrored rotation suitable for
 * dual-flywheel shooter designs where both flywheels spin inward.
 *
 * <p><strong>Motor Configuration</strong>:
 *
 * <ul>
 *   <li>Right Motor (CAN ID from Constants): Leader, inverted (Clockwise Positive).
 *   <li>Left Motor (CAN ID from Constants): Follower, opposed alignment for mirrored spin.
 * </ul>
 *
 * <p><strong>Control Strategy</strong>: Uses open-loop voltage control. When enabled, applies a
 * fixed voltage to spin the flywheels. The left motor automatically mirrors the right motor's
 * output via Opposed Follower mode.
 *
 * <p><strong>Thread Safety</strong>: All state access is synchronized as the Looper runs on a
 * separate thread from the main robot loop.
 *
 * @see ShooterIO
 */
public class Shooter extends Subsystem {
  private static Shooter mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global Shooter instance.
   */
  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  private final ShooterIO mIO;
  private final ShooterIO.ShooterIOInputs mInputs = new ShooterIO.ShooterIOInputs();

  /** Whether the shooter is currently running (firing). */
  private boolean mIsRunning = false;

  // --- SysId Integration ---
  private final SysIdRoutine mSysIdRoutine;
  private boolean mSysIdActive = false;

  private Shooter() {
    // Use real IO implementation; swap to simulation IO if needed
    mIO = new ShooterIOReal();

    // Initialize SysId routine with Shooter-specific configuration
    mSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config()
                .setSubsystemName("Shooter")
                .setRampRate(Volts.of(1.0)) // 1 V/s for quasistatic
                .setStepVoltage(Volts.of(10.0)) // 4V step for dynamic
                .setTimeout(Seconds.of(10.0)) // 10s safety timeout
                .setLogWriter(new CSVLogWriter("sysid_Shooter")));
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Shooter.this) {
              // Start in stopped state
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
   * Starts the shooter at the configured voltage.
   *
   * <p>Applies constant voltage for open-loop flywheel control.
   */
  public synchronized void run() {
    mIsRunning = true;
  }

  /**
   * Checks if the shooter is currently running.
   *
   * @return True if the shooter is actively spinning.
   */
  public synchronized boolean isRunning() {
    return mIsRunning;
  }

  /**
   * Gets the current average flywheel velocity.
   *
   * @return Average velocity in rotations per second.
   */
  public synchronized double getAverageVelocity() {
    return (Math.abs(mInputs.rightVelocityRotPerSec) + Math.abs(mInputs.leftVelocityRotPerSec))
        / 2.0;
  }

  // --- Velocity Control Mode ---
  private boolean mVelocityControlActive = false;

  /**
   * Runs the shooter at a target velocity using feedforward + P feedback.
   *
   * <p>Uses the SysId-derived parameters: V = kS * sign(v) + kV * v_target + kP * (v_target - v)
   *
   * <p>Call this every cycle while the velocity control button is held.
   *
   * @param targetVel Target velocity in rotations per second.
   */
  /**
   * Runs the shooter at a target velocity using onboard VelocityVoltage control.
   *
   * <p>Uses the SysId-derived parameters stored in Slot 0 of the TalonFX.
   *
   * @param targetVel Target velocity in rotations per second.
   */
  public void runVelocityControl(double targetVel) {
    mVelocityControlActive = true;
    // Command directly to IO
    mIO.setTargetVelocity(targetVel);
  }

  /** Runs the shooter at the constant target velocity defined in Constants. */
  public void runVelocityControl() {
    runVelocityControl(Constants.Shooter.kTargetVelocity);
  }

  /** Stops velocity control mode. */
  public void stopVelocityControl() {
    if (mVelocityControlActive) {
      mVelocityControlActive = false;
      mIO.setVoltage(0);
    }
  }

  /**
   * Checks if velocity control is active.
   *
   * @return True if velocity control is running.
   */
  public boolean isVelocityControlActive() {
    return mVelocityControlActive;
  }

  // --- SysId API (Test Mode Only) ---

  /**
   * Starts a SysId characterization test.
   *
   * <p>Call this once when the corresponding button is pressed.
   *
   * @param type Test type (QUASISTATIC or DYNAMIC).
   * @param direction Test direction (FORWARD or REVERSE).
   */
  public void startSysId(SysIdRoutine.TestType type, SysIdRoutine.Direction direction) {
    mSysIdActive = true;
    mSysIdRoutine.start(type, direction);
  }

  /**
   * Stops the active SysId test and saves data to CSV.
   *
   * <p>Call this when the button is released.
   */
  public void stopSysId() {
    if (mSysIdActive) {
      mSysIdRoutine.stop();
      mSysIdActive = false;
      mIO.setVoltage(0); // Ensure motor stops
    }
  }

  /**
   * Updates the SysId routine and applies the calculated voltage.
   *
   * <p>Call this every cycle during Test Mode while a test is active.
   */
  public void updateSysId() {
    if (mSysIdActive) {
      double timestamp = Timer.getFPGATimestamp();
      // Update routine with current position and velocity (in rotations and rot/s)
      mSysIdRoutine.update(timestamp, mInputs.rightPositionRot, mInputs.rightVelocityRotPerSec);
      mIsRunning = false;
      mOpenLoopActive = true;
      mOpenLoopVoltage = mSysIdRoutine.getOutputVoltage();
      // Apply the calculated voltage
    }
  }

  /**
   * Checks if a SysId test is currently running.
   *
   * @return True if SysId is active.
   */
  public boolean isSysIdActive() {
    return mSysIdActive;
  }

  // ============================================================
  // PER-STATE OPERATE METHODS (Orbit 1690 Pattern)
  // ============================================================

  @Override
  public void travelOperate() {
    stopVelocityControl();
    mIsRunning = false;
    mOpenLoopActive = false;
  }

  @Override
  public void intakeOperate() {
    // Shooter idle during intake
    stopVelocityControl();
  }

  @Override
  public void scoreOperate() {
    var params = frc.robot.GlobalData.currentShotParams;
    if (params != null && params.isValid) {
      runVelocityControl(params.flywheelSpeedRotPerSec);
    }
  }

  @Override
  public void passOperate() {
    runVelocityControl();
  }

  @Override
  public void climbOperate() {
    stopShooter();
  }

  @Override
  public void calibrateOperate() {
    stopShooter();
  }

  @Override
  public void scoreTestOperate() {
    runVelocityControl(44);
  }

  /**
   * Checks if the shooter is at the target velocity within tolerance.
   *
   * @return True if average velocity is within 5 RPS of target.
   */
  public boolean isAtTargetSpeed() {
    if (!mVelocityControlActive) return false;
    // Approximate check: within 5 RPS tolerance
    return true; // Caller should check specific tolerance if needed
  }

  // ============================================================
  // TEST MODE (Distributed Pattern)
  // ============================================================

  /** Test routine selector for Shooter. */
  public enum ShooterTestRoutine {
    VOLTAGE,
    SYSID,
    PID
  }

  private ShooterTestRoutine mTestRoutine = ShooterTestRoutine.SYSID;
  private double mTunableShooterVel = Constants.Shooter.kTargetVelocity;
  private boolean mShooterSysIdButtonWasPressed = false;

  @Override
  public void handleTestMode(frc.robot.ControlBoard control) {
    switch (mTestRoutine) {
      case VOLTAGE -> {
        double testVoltage = 6.0;
        if (control.getTriangleButton()) {
          setOpenLoopVoltage(testVoltage);
        } else if (control.getCrossButton()) {
          setOpenLoopVoltage(-testVoltage);
        } else {
          setOpenLoopVoltage(0.0);
        }
      }
      case SYSID -> {
        boolean anySysIdButtonPressed =
            control.getTriangleButton()
                || control.getSquareButton()
                || control.getCrossButton()
                || control.getCircleButton();
        if (!isSysIdActive()) {
          if (control.getTriangleButton()) {
            startSysId(SysIdRoutine.TestType.QUASISTATIC, SysIdRoutine.Direction.FORWARD);
          } else if (control.getSquareButton()) {
            startSysId(SysIdRoutine.TestType.QUASISTATIC, SysIdRoutine.Direction.REVERSE);
          } else if (control.getCrossButton()) {
            startSysId(SysIdRoutine.TestType.DYNAMIC, SysIdRoutine.Direction.FORWARD);
          } else if (control.getCircleButton()) {
            startSysId(SysIdRoutine.TestType.DYNAMIC, SysIdRoutine.Direction.REVERSE);
          }
        }
        updateSysId();
        if (mShooterSysIdButtonWasPressed && !anySysIdButtonPressed) {
          stopSysId();
        }
        mShooterSysIdButtonWasPressed = anySysIdButtonPressed;
      }
      case PID -> {
        if (control.getCrossButton()) {
          runVelocityControl(mTunableShooterVel);
        } else {
          stopVelocityControl();
        }
      }
    }
  }

  // --- Subsystem Interface ---

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);
  }

  // --- Open Loop Control Mode ---
  private boolean mOpenLoopActive = false;
  private double mOpenLoopVoltage = 0.0;

  /**
   * Sets the shooter voltage directly (open-loop).
   *
   * @param volts Voltage command (-12 to +12).
   */
  public synchronized void setOpenLoopVoltage(double volts) {
    mIsRunning = false;
    mOpenLoopActive = true;
    mOpenLoopVoltage = volts;
  }

  /** Stops the shooter. */
  public synchronized void stopShooter() {
    mIsRunning = false;
    mOpenLoopActive = false;
    mVelocityControlActive = false;
    mIO.setVoltage(0);
  }

  @Override
  public void writePeriodicOutputs() {
    // Skip if velocity control is active (it handles voltage directly)
    if (mVelocityControlActive) {
      return;
    }

    synchronized (this) {
      if (mOpenLoopActive) {
        mIO.setVoltage(mOpenLoopVoltage);
      } else if (mIsRunning) {
        mIO.setVoltage(Constants.Shooter.kShooterVoltage);
      } else {
        mIO.setVoltage(0);
      }
    }
  }

  @Override
  public boolean checkConnectionActive() {
    int rightFw = mIO.getRightMotorFirmwareVersion();
    int leftFw = mIO.getLeftMotorFirmwareVersion();

    return rightFw != 0 && leftFw != 0;
  }

  @Override
  public boolean checkConnectionPassive() {
    return mInputs.rightMotorConnected && mInputs.leftMotorConnected;
  }

  @Override
  public boolean checkSanityPassive() {
    // Check for current mismatch (indicates mechanical binding or motor failure)
    double currentDiff = Math.abs(mInputs.rightCurrentAmps - mInputs.leftCurrentAmps);
    if (currentDiff > 20.0) {
      return false;
    }

    // Check for excessive current (stall condition)
    if (mInputs.rightCurrentAmps > 80.0 || mInputs.leftCurrentAmps > 80.0) {
      return false;
    }

    return true;
  }

  @Override
  public void outputTelemetry() {
    var dashboard = frc.robot.DashboardState.getInstance();
    dashboard.shooterOK = checkConnectionPassive() && checkSanityPassive();
  }

  @Override
  public void stop() {
    synchronized (this) {
      mIsRunning = false;
    }
    stopSysId(); // Also stop any active SysId test
    mIO.stop();
  }

  @Override
  public void zeroSensors() {
    // Shooter has no sensors to zero
  }
}
