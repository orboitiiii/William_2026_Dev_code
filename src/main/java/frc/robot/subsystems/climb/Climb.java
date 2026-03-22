package frc.robot.subsystems.climb;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControlBoard;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;

/**
 * Climb Subsystem - Rope Pulling Mechanism.
 *
 * <p>
 * This subsystem controls a Kraken X44 motor through a 1:125 gearbox to pull a
 * string for
 * climbing. Features include:
 *
 * <ul>
 * <li><strong>Open-Loop / Closed-Loop Control</strong>
 * <li><strong>Testing Modes</strong>
 * </ul>
 */
public class Climb extends Subsystem {
  private static Climb mInstance;

  public static Climb getInstance() {
    if (mInstance == null) {
      mInstance = new Climb();
    }
    return mInstance;
  }

  // --- Hardware Abstraction ---
  private final ClimbIO mIO;
  private final ClimbIO.ClimbIOInputs mInputs = new ClimbIO.ClimbIOInputs();

  // --- Connection Monitoring / Debouncing ---
  private final Debouncer mMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private boolean mMotorConnectedDebounced = true;

  // --- State Variables ---
  private boolean mOpenLoopMode = false;
  private double mOpenLoopVoltageCommand = 0.0;

  private double mGoalPositionRotations = 0.0;
  private double mArbitraryFeedforward = 0.0;

  // --- Test Routine ---
  public enum ClimbTestRoutine {
    VOLTAGE,
    PID
  }

  private ClimbTestRoutine mTestRoutine = ClimbTestRoutine.PID;

  private Climb() {
    // In real environment we use ClimbIOReal. Add Simulation support if needed.
    mIO = new ClimbIOReal();
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Climb.this) {
              // Initialize goal to current position to avoid jerking
              mGoalPositionRotations = mInputs.positionRotations;
              mOpenLoopMode = true;
              mOpenLoopVoltageCommand = 0.0;
            }
          }

          @Override
          public void onLoop(double timestamp) {
            // Processing in writePeriodicOutputs
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  /**
   * Sets the voltage command for the climb motor.
   *
   * @param volts Voltage to apply. Positive values pull string up.
   */
  public synchronized void setOpenLoopVoltage(double volts) {
    mOpenLoopMode = true;
    mOpenLoopVoltageCommand = volts;
  }

  /**
   * Sets the target position for the closed-loop controller.
   *
   * @param positionRots Spool rotations target.
   */
  /**
   * Sets the target position for the closed-loop controller with feedforward.
   *
   * @param positionRots     Spool rotations target.
   * @param feedforwardVolts Arbitrary feedforward voltage to apply.
   */
  public synchronized void setTargetPosition(double positionRots, double feedforwardVolts) {
    mOpenLoopMode = false;
    mGoalPositionRotations = positionRots;
    mArbitraryFeedforward = feedforwardVolts;
  }

  /**
   * Sets the target position for the closed-loop controller.
   *
   * @param positionRots Spool rotations target.
   */
  public synchronized void setTargetPosition(double positionRots) {
    setTargetPosition(positionRots, 0.0);
  }

  // ============================================================
  // OPERATE MODES
  // ============================================================

  @Override
  public void climbOperate() {
    // This method is ONLY called when RobotState == CLIMB (Options held),
    // so D-Pad inputs are inherently gated — no extra safety check needed.
    var control = ControlBoard.getInstance();

    if (control.getClimbPreClimbPressed()) {
      // D-Pad ↑: Deploy rope to highest position (pre-climb)
      // Position derived from spool geometry: ~2.477 output rotations
      // through 1:125 gearbox corresponds to full rope extension.
      setTargetPosition(-2.47, 0.0);
    } else if (control.getClimbEngagePressed()) {
      // D-Pad ↓: Retract rope to engage climb
      // -0.5695 rots = partial retraction that hooks the cage bar.
      // Physics: Requires huge torque to lift the 60kg robot against gravity.
      // Applying nearly 12V arbitrary feedforward to overcome load.
      setTargetPosition(-0.5695, 11.5);
    } else if (control.getClimbStowPressed()) {
      // D-Pad ←: Return to zero (stow)
      // Physics: Also retracting under heavy load.
      setTargetPosition(0.0, 11.5);
    } else if (control.getClimbSafetyPressed()) {
      // D-Pad →: Open-loop safety pull-up (6V)
      // If the closed-loop target is unreachable due to mechanical
      // interference, this provides manual override torque to clear.
      setOpenLoopVoltage(6.0);
    }
    // No else branch: if no D-Pad is pressed, hold the last command.
    // This prevents the motor from re-zeroing between button presses.
  }

  @Override
  public void handleTestMode(ControlBoard control) {
    switch (mTestRoutine) {
      case VOLTAGE -> {
        double testVoltage = 6.0; // Start with safe low voltage
        if (control.getTriangleButton()) {
          setOpenLoopVoltage(testVoltage);
        } else if (control.getCrossButton()) {
          setOpenLoopVoltage(-testVoltage);
        } else {
          setOpenLoopVoltage(0.0);
        }
      }
      case PID -> {
        if (control.getTriangleButton()) {
          setTargetPosition(-2.0, 0.0);
        } else if (control.getCircleButton()) {
          setTargetPosition(-0.5695, 11.5);
        } else if (control.getCrossButton()) {
          setTargetPosition(0.0, 11.5);
        } else if (control.getSquareButton()) {
          setOpenLoopVoltage(6.0);
        } else {
          setOpenLoopVoltage(0.0);
        }
      }
    }
  }

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);
    mMotorConnectedDebounced = mMotorConnectedDebouncer.calculate(mInputs.motorConnected);
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    // Zero-Energy / Active Fail-Safe check (PHYSICS REQUIREMENT):
    // If motor is disconnected or sensors read NaN (Stale/Invalid),
    // immediately drop energy to 0 to prevent uncontrolled movement.
    if (!mMotorConnectedDebounced || Double.isNaN(mInputs.positionRotations)) {
      mIO.stop();
      return;
    }

    if (DriverStation.isDisabled()) {
      // Ensure brake mode is on to hold the robot if disabled mid-climb
      mIO.setBrakeMode(true);
      mIO.stop();
      return;
    }

    if (mOpenLoopMode) {
      mIO.setVoltage(mOpenLoopVoltageCommand);
    } else {
      mIO.setPositionSetpoint(mGoalPositionRotations, mArbitraryFeedforward);
    }
  }

  @Override
  public void stop() {
    setOpenLoopVoltage(0.0);
    mIO.stop();
  }

  @Override
  public void zeroSensors() {
    // Only zero via hardware config on boot for now.
  }

  @Override
  public boolean checkConnectionActive() {
    return true; // Firmware check could be implemented here
  }

  @Override
  public boolean checkConnectionPassive() {
    return mMotorConnectedDebounced;
  }

  @Override
  public boolean checkSanityPassive() {
    return true;
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Climb/Measured Rotations", mInputs.positionRotations);
    SmartDashboard.putNumber("Climb/Goal Rotations", mGoalPositionRotations);
    SmartDashboard.putNumber("Climb/Velocity RotPerSec", mInputs.velocityRotSec);
    SmartDashboard.putBoolean("Climb/Motor Connected", mMotorConnectedDebounced);
  }
}
