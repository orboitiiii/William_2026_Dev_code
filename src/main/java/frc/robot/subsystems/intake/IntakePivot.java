package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;

/**
 * Intake Pivot Subsystem - Dual-motor Four-Bar Linkage mechanism.
 *
 * <p>This subsystem controls the intake pivot arm using a four-bar linkage driven by two KrakenX60
 * motors. The linkage provides mechanical advantage and controlled motion for positioning the
 * intake at various angles.
 *
 * <p><strong>Motor Configuration</strong>:
 *
 * <ul>
 *   <li>Right Motor (CAN ID 30): Leader, counter-clockwise positive.
 *   <li>Left Motor (CAN ID 31): Follower, clockwise positive (inverted).
 * </ul>
 *
 * <p><strong>Control Strategy</strong>: Uses MotionMagic position control for smooth, profiled
 * motion. The left motor follows the right motor in opposing direction to produce synchronized
 * torque output for the four-bar linkage.
 *
 * <p><strong>Thread Safety</strong>: All state access is synchronized as the Looper runs on a
 * separate thread from the main robot loop.
 *
 * @see IntakePivotIO
 */
public class IntakePivot extends Subsystem {
  private static IntakePivot mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global IntakePivot instance.
   */
  public static IntakePivot getInstance() {
    if (mInstance == null) {
      mInstance = new IntakePivot();
    }
    return mInstance;
  }

  private final IntakePivotIO mIO;
  private final IntakePivotIO.IntakePivotIOInputs mInputs = new IntakePivotIO.IntakePivotIOInputs();

  /** Desired pivot target angle. */
  private Rotation2d mTargetAngle = new Rotation2d();

  /** Control mode flag: true = position control, false = open-loop voltage. */
  private boolean mIsClosedLoop = true;

  /** Open-loop voltage command (only used when mIsClosedLoop = false). */
  private double mOpenLoopVoltage = 0.0;

  private IntakePivot() {
    // Use real IO implementation; swap to simulation IO if needed
    mIO = new IntakePivotIOReal();
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (IntakePivot.this) {
              // Hold current position at enable
              mTargetAngle = mInputs.position;
              mIsClosedLoop = true;
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
   * Commands the pivot to a target angle using closed-loop position control.
   *
   * <p>Uses MotionMagic for smooth, trapezoid velocity profiled motion.
   *
   * @param angle Target pivot angle.
   */
  public synchronized void setTargetAngle(Rotation2d angle) {
    mTargetAngle = angle;
    mIsClosedLoop = true;
  }

  /**
   * Commands the pivot with open-loop voltage control.
   *
   * <p>Useful for manual jogging or calibration. Positive voltage moves the pivot in the positive
   * direction (as defined by the right motor convention).
   *
   * @param volts Voltage command (-12 to +12).
   */
  public synchronized void setOpenLoopVoltage(double volts) {
    mOpenLoopVoltage = volts;
    mIsClosedLoop = false;
  }

  /**
   * Returns the current pivot angle.
   *
   * @return Current pivot position as Rotation2d.
   */
  public synchronized Rotation2d getAngle() {
    return mInputs.position;
  }

  /**
   * Returns the target pivot angle.
   *
   * @return Target angle as Rotation2d.
   */
  public synchronized Rotation2d getTargetAngle() {
    return mTargetAngle;
  }

  /**
   * Checks if the pivot is at the target position within tolerance.
   *
   * @param toleranceDegrees Allowable error in degrees.
   * @return True if within tolerance.
   */
  public synchronized boolean atTarget(double toleranceDegrees) {
    double errorDegrees = Math.abs(mTargetAngle.minus(mInputs.position).getDegrees());
    return errorDegrees <= toleranceDegrees;
  }

  /**
   * Resets the motor encoder positions to the specified angle.
   *
   * <p>Used for calibration when the physical position is known (e.g., at a hard stop).
   *
   * @param angle The current physical angle to set as the encoder reference.
   */
  public synchronized void resetPosition(Rotation2d angle) {
    mIO.resetPosition(angle);
    mTargetAngle = angle;
  }

  // --- Subsystem Interface ---

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);
  }

  @Override
  public void writePeriodicOutputs() {
    synchronized (this) {
      if (mIsClosedLoop) {
        mIO.setPosition(mTargetAngle);
      } else {
        mIO.setVoltage(mOpenLoopVoltage);
      }
    }
  }

  @Override
  public boolean checkConnectionActive() {
    System.out.println("[IntakePivot] Running Active Connection Check...");

    int rightFw = mIO.getRightMotorFirmwareVersion();
    int leftFw = mIO.getLeftMotorFirmwareVersion();

    boolean rightOk = rightFw != 0;
    boolean leftOk = leftFw != 0;

    if (!rightOk) {
      System.err.println("[IntakePivot] Right Motor Firmware Read Failed!");
    } else {
      System.out.println("[IntakePivot] Right Motor FW: " + rightFw);
    }

    if (!leftOk) {
      System.err.println("[IntakePivot] Left Motor Firmware Read Failed!");
    } else {
      System.out.println("[IntakePivot] Left Motor FW: " + leftFw);
    }

    return rightOk && leftOk;
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
      System.err.println(
          "[IntakePivot] Current Mismatch: R="
              + mInputs.rightCurrentAmps
              + "A, L="
              + mInputs.leftCurrentAmps
              + "A");
      return false;
    }

    // Check for excessive current (stall condition)
    if (mInputs.rightCurrentAmps > 60.0 || mInputs.leftCurrentAmps > 60.0) {
      System.err.println(
          "[IntakePivot] Current Spike: R="
              + mInputs.rightCurrentAmps
              + "A, L="
              + mInputs.leftCurrentAmps
              + "A");
      return false;
    }

    return true;
  }

  @Override
  public void outputTelemetry() {
    // TODO: Publish pivot state to NetworkTables
    // SmartDashboard.putNumber("IntakePivot/AngleDeg",
    // mInputs.position.getDegrees());
    // SmartDashboard.putNumber("IntakePivot/TargetDeg", mTargetAngle.getDegrees());
  }

  @Override
  public void stop() {
    synchronized (this) {
      mIsClosedLoop = true;
      mOpenLoopVoltage = 0.0;
    }
    mIO.stop();
  }

  @Override
  public void zeroSensors() {
    mIO.resetPosition(new Rotation2d());
    synchronized (this) {
      mTargetAngle = new Rotation2d();
    }
  }
}
