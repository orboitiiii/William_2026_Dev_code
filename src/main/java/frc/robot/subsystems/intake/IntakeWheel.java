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

  public static IntakeWheel getInstance() {
    if (mInstance == null) {
      mInstance = new IntakeWheel();
    }
    return mInstance;
  }

  public enum IntakeState {
    IDLE,
    RUNNING,
    REVERSE
  }

  private final IntakeWheelIO mIO;
  private final IntakeWheelIO.IntakeWheelIOInputs mInputs = new IntakeWheelIO.IntakeWheelIOInputs();

  private IntakeState mState = IntakeState.IDLE;

  private IntakeWheel() {
    if (Constants.kHasIntakeWheel) {
      mIO = new IntakeWheelIOReal();
    } else {
      mIO = new IntakeWheelIO() {};
    }
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (IntakeWheel.this) {
              mState = IntakeState.IDLE;
            }
          }

          @Override
          public void onLoop(double timestamp) {}

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  public synchronized void setWantedState(IntakeState state) {
    mState = state;
  }

  public synchronized void toggle() {
    if (mState == IntakeState.IDLE) {
      mState = IntakeState.RUNNING;
    } else {
      mState = IntakeState.IDLE;
    }
  }

  // ============================================================
  // PER-STATE OPERATE METHODS (Orbit 1690 Pattern Decoupled)
  // ============================================================
  // The IntakeWheel state is now independently controlled via GlobalData toggles.

  private void processWantedState() {
    switch (frc.robot.GlobalData.intakeWheelWantedState) {
      case FORWARD:
        mState = IntakeState.RUNNING;
        break;
      case REVERSE:
        // We will overload RUNNING specifically for reversed voltage below
        // Or create a new REVERSE state if needed, but for simplicity,
        // we'll add REVERSE to IntakeState
        mState = IntakeState.REVERSE;
        break;
      case IDLE:
      default:
        mState = IntakeState.IDLE;
        break;
    }
  }

  @Override
  public void travelOperate() {
    processWantedState();
  }

  @Override
  public void intakeOperate() {
    processWantedState();
  }

  @Override
  public void scoreOperate() {
    processWantedState();
  }

  @Override
  public void passOperate() {
    processWantedState();
  }

  @Override
  public void climbOperate() {
    processWantedState();
  }

  @Override
  public void calibrateOperate() {
    mState = IntakeState.IDLE; // Safety override
  }

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);
  }

  @Override
  public void writePeriodicOutputs() {
    synchronized (this) {
      switch (mState) {
        case RUNNING:
          mIO.setVoltage(Constants.Intake.kIntakeVoltage);
          break;
        case REVERSE:
          mIO.setVoltage(-Constants.Intake.kIntakeVoltage);
          break;
        case IDLE:
        default:
          mIO.stop();
          break;
      }
    }
  }

  @Override
  public boolean checkConnectionActive() {
    int fw = mIO.getMotorFirmwareVersion();
    return fw != 0;
  }

  @Override
  public boolean checkConnectionPassive() {
    return mInputs.motorConnected;
  }

  @Override
  public boolean checkSanityPassive() {
    // 1. Current spike check (hard stall)
    if (mInputs.currentAmps > 80.0) {
      return false;
    }

    // 2. Mechanical binding / jammed object check
    // If the state is RUNNING or REVERSE, we are applying significant voltage
    // but if velocity is extremely low, it indicates a jam.
    if (mState == IntakeState.RUNNING || mState == IntakeState.REVERSE) {
      if (Math.abs(mInputs.appliedVolts) > 6.0 && Math.abs(mInputs.velocityRotationsPerSec) < 1.0) {
        return false;
      }
    }

    return true;
  }

  @Override
  public void outputTelemetry() {
    frc.robot.DashboardState.getInstance().intakeWheelsOK =
        checkConnectionPassive() && checkSanityPassive();
  }

  @Override
  public void stop() {
    synchronized (this) {
      mState = IntakeState.IDLE;
    }
    mIO.stop();
  }

  @Override
  public void zeroSensors() {}

  /** Returns true if the intake wheels are actively spinning forward to ingest a game piece. */
  public synchronized boolean isTakingIn() {
    return mState == IntakeState.RUNNING;
  }
}
