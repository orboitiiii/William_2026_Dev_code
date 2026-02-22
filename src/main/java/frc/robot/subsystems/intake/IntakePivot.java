package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.framework.CSVLogWriter;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.framework.SysIdRoutine;

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

  public static IntakePivot getInstance() {
    if (mInstance == null) {
      mInstance = new IntakePivot();
    }
    return mInstance;
  }

  public enum PivotState {
    STOWED,
    INTAKE,
    SCORE_FUEL,
    MANUAL
  }

  private final IntakePivotIO mIO;
  private final IntakePivotIO.IntakePivotIOInputs mInputs = new IntakePivotIO.IntakePivotIOInputs();

  // --- SysId Integration ---
  private final SysIdRoutine mSysIdRoutine;
  private boolean mSysIdActive = false;

  private PivotState mState = PivotState.STOWED;
  private Rotation2d mTargetAngle = new Rotation2d();
  private boolean mIsClosedLoop = true;
  private double mOpenLoopVoltage = 0.0;

  // Gravity Compensation Map (Angle -> kG)
  private final InterpolatingDoubleTreeMap mGravityMap = new InterpolatingDoubleTreeMap();

  private IntakePivot() {
    mIO = new IntakePivotIOReal();

    // Populate Gravity Map
    for (double[] point : Constants.IntakePivot.kGravityMap) {
      mGravityMap.put(point[0], point[1]);
    }

    // Initialize SysId routine with IntakePivot-specific configuration
    mSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config()
                .setSubsystemName("IntakePivot")
                .setRampRate(Volts.of(1.0)) // 1 V/s for quasistatic
                .setStepVoltage(Volts.of(3.0)) // 3V step for dynamic
                .setTimeout(Seconds.of(10.0)) // 10s safety timeout
                .setLogWriter(new CSVLogWriter("sysid_IntakePivot")));
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (IntakePivot.this) {
              mTargetAngle = mInputs.position;
              mState = PivotState.STOWED;
              // Only enable closed-loop control if NOT in Test Mode
              if (!frc.robot.GlobalData.isTestMode) {
                mIsClosedLoop = true;
              } else {
                mIsClosedLoop = false;
              }
            }
          }

          @Override
          public void onLoop(double timestamp) {
            synchronized (IntakePivot.this) {
              switch (mState) {
                case STOWED:
                  mTargetAngle = Rotation2d.fromDegrees(80.0);
                  break;
                case INTAKE:
                  mTargetAngle = Rotation2d.fromDegrees(Constants.IntakePivot.kMinAngle);
                  break;
                case SCORE_FUEL:
                  mTargetAngle = Rotation2d.fromDegrees(Constants.IntakePivot.kMinAngle);
                  break;
                case MANUAL:
                  // Target angle set externally
                  break;
              }
            }
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  // --- SysId API (Test Mode Only) ---

  /**
   * Starts a SysId characterization test.
   *
   * @param type Test type (QUASISTATIC or DYNAMIC).
   * @param direction Test direction (FORWARD or REVERSE).
   */
  public void startSysId(SysIdRoutine.TestType type, SysIdRoutine.Direction direction) {
    mSysIdActive = true;
    mIsClosedLoop = false;
    mSysIdRoutine.start(type, direction);
  }

  /** Stops the active SysId test and saves data to CSV. */
  public void stopSysId() {
    if (mSysIdActive) {
      mSysIdRoutine.stop();
      mSysIdActive = false;
      mIO.setVoltage(0);
    }
  }

  /** Updates the SysId routine and applies the calculated voltage. */
  public void updateSysId() {
    if (mSysIdActive) {
      double timestamp = Timer.getFPGATimestamp();
      Rotation2d pos = getAngle();
      double velRotPerSec = mInputs.velocityRotationsPerSec; // Already in Rot/s
      mSysIdRoutine.update(timestamp, pos.getRotations(), velRotPerSec);
      mOpenLoopVoltage = mSysIdRoutine.getOutputVoltage();
      mIsClosedLoop = false; // Ensure we stay in open loop
    }
  }

  /** Checks if a SysId test is currently running. */
  public boolean isSysIdActive() {
    return mSysIdActive;
  }

  // ============================================================
  // PER-STATE OPERATE METHODS (Orbit 1690 Pattern Decoupled)
  // ============================================================
  // The IntakePivot state is now independently controlled via GlobalData toggles.

  private void processWantedState() {
    if (frc.robot.GlobalData.pivotWantsDown) {
      mState = PivotState.INTAKE;
    } else {
      mState = PivotState.STOWED;
    }
    mIsClosedLoop = true;
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
    mState = PivotState.STOWED;
    mIsClosedLoop = true;
  }

  // ============================================================
  // TEST MODE (Distributed Pattern)
  // ============================================================

  /** Test routine selector for IntakePivot. */
  public enum PivotTestRoutine {
    VOLTAGE,
    SYSID,
    CONTROL_LOOP
  }

  private PivotTestRoutine mTestRoutine = PivotTestRoutine.CONTROL_LOOP;
  private boolean mPivotSysIdButtonWasPressed = false;

  @Override
  public void handleTestMode(frc.robot.ControlBoard control) {
    switch (mTestRoutine) {
      case VOLTAGE -> {
        boolean fwd = control.getPivotForward();
        boolean rev = control.getPivotReverse();
        if (fwd) {
          setOpenLoopVoltage(1.0);
        } else if (rev) {
          setOpenLoopVoltage(-1.0);
        } else {
          setOpenLoopVoltage(0.0);
        }
      }
      case SYSID -> {
        boolean anySysIdButtonPressed =
            control.getTriangleButton() || control.getSquareButton() || control.getCrossButton();
        boolean circlePressed = control.getCircleButton();
        if (!isSysIdActive()) {
          setOpenLoopVoltage(0.0);
          if (control.getTriangleButton()) {
            startSysId(SysIdRoutine.TestType.QUASISTATIC, SysIdRoutine.Direction.FORWARD);
          } else if (control.getSquareButton()) {
            startSysId(SysIdRoutine.TestType.QUASISTATIC, SysIdRoutine.Direction.REVERSE);
          } else if (control.getCrossButton()) {
            startSysId(SysIdRoutine.TestType.DYNAMIC, SysIdRoutine.Direction.FORWARD);
          } else if (circlePressed) {
            startSysId(SysIdRoutine.TestType.DYNAMIC, SysIdRoutine.Direction.REVERSE);
          }
        }
        updateSysId();
        if (mPivotSysIdButtonWasPressed && !anySysIdButtonPressed && !circlePressed) {
          stopSysId();
        }
        mPivotSysIdButtonWasPressed = anySysIdButtonPressed || circlePressed;
      }
      case CONTROL_LOOP -> {
        if (control.getPivotForward()) {
          runTestPosition(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90));
        } else if (control.getPivotReverse()) {
          runTestPosition(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(23));
        } else {
          setOpenLoopVoltage(0.0);
        }
      }
    }
  }

  public synchronized void setWantedState(PivotState state) {
    mState = state;
    mIsClosedLoop = true;
  }

  public synchronized void setOpenLoopVoltage(double volts) {
    mOpenLoopVoltage = volts;
    mIsClosedLoop = false;
  }

  public synchronized void runTestPosition(Rotation2d target) {
    mState = PivotState.MANUAL;
    mTargetAngle = target;
    mIsClosedLoop = true;
  }

  public synchronized Rotation2d getAngle() {
    return mInputs.position;
  }

  public synchronized boolean atTarget(double toleranceDegrees) {
    double errorDegrees = Math.abs(mTargetAngle.minus(mInputs.position).getDegrees());
    return errorDegrees <= toleranceDegrees;
  }

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);
  }

  @Override
  public void writePeriodicOutputs() {
    synchronized (this) {
      synchronized (this) {
        if (mIsClosedLoop) {
          // Asymmetric Motion Magic Logic
          // Determine direction: Target > Current = UP (Positive Angle)
          // Note: Angles are Down=Positive? No, "Down = Positive Angle" in IO config.
          // Let's check Constants.
          // If Down is Positive, then:
          // Target > Current = Moving Down (Slower)
          // Target < Current = Moving Up (Faster)

          // Let's verify:
          // stow = 90 (vertical down?)
          // intake = 27 (angled up?)
          // so 90 -> 27 is UP. (Large to Small is UP)
          // 27 -> 90 is DOWN. (Small to Large is DOWN)

          double currentDeg = mInputs.position.getDegrees();
          double targetDeg = mTargetAngle.getDegrees();

          double cruiseVel;
          double accel;

          if (targetDeg > currentDeg) {
            // Moving to larger angle -> UP (e.g. 27 to 90) -> Fast
            cruiseVel = Constants.IntakePivot.kCruiseVelocityUp;
            accel = Constants.IntakePivot.kAccelerationUp;
          } else {
            // Moving to smaller angle -> DOWN (e.g. 90 to 27) -> Slow
            cruiseVel = Constants.IntakePivot.kCruiseVelocityDown;
            accel = Constants.IntakePivot.kAccelerationDown;
          }

          double kG = mGravityMap.get(mInputs.position.getDegrees());
          double gravityVolts = kG * mInputs.position.getCos();

          // Apply extra downward pressure if we are intaking and deployed
          boolean applyingDownPressure = false;
          double downPressureVolts = 0.0;

          if (mState == PivotState.INTAKE && atTarget(5.0)) {
            if (IntakeWheel.getInstance().isTakingIn()) {
              applyingDownPressure = true;
              // Pure voltage hold: overcome gravity + physical down pressure
              downPressureVolts = gravityVolts - Constants.IntakePivot.kIntakeDownPressureVolts;
            }
          }

          if (applyingDownPressure) {
            // Bypass MotionMagic entirely. Let the motor hold using pure voltage to prevent
            // PID windup or Trajectory Generator trapping at the physical boundary.
            mIO.setVoltage(downPressureVolts);
          } else {
            // Normal MotionMagic position tracking
            mIO.setPosition(mTargetAngle, gravityVolts, cruiseVel, accel);
          }
        } else {
          mIO.setVoltage(mOpenLoopVoltage);
        }
      }
    }
  }

  @Override
  public boolean checkConnectionActive() {
    int firmwareParams = mIO.getRightMotorFirmwareVersion();
    boolean rightOK = firmwareParams > 0;

    // Optional: Check left motor too if IO supports it (added method to IO
    // interface?)
    // Assuming mIO.getRightMotorFirmwareVersion checks the leader.

    return rightOK;
  }

  @Override
  public boolean checkConnectionPassive() {
    return true;
  }

  @Override
  public boolean checkSanityPassive() {
    // 1. Hard position bounds check
    // Pivot should never exceed [-5 deg, +110 deg] mapped via Constants
    double angleDeg = mInputs.position.getDegrees();
    if (angleDeg < Constants.IntakePivot.kMinAngle - 5.0
        || angleDeg > Constants.IntakePivot.kMaxAngle + 5.0) {
      return false;
    }

    // 2. Continuous over-current or massive disparity
    double currentDiff = Math.abs(mInputs.leftCurrentAmps - mInputs.rightCurrentAmps);
    if (currentDiff > 25.0 || mInputs.leftCurrentAmps > 80.0 || mInputs.rightCurrentAmps > 80.0) {
      return false;
    }

    // 3. Stall check (high voltage, zero/low movement)
    if (Math.abs(mInputs.rightAppliedVolts) > 6.0
        && Math.abs(mInputs.velocityRotationsPerSec) < 0.1) {
      return false;
    }

    return true;
  }

  @Override
  public void outputTelemetry() {
    frc.robot.DashboardState.getInstance().intakePivotOK =
        checkConnectionPassive() && checkSanityPassive();
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "IntakePivot/Angle", mInputs.position.getDegrees());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "IntakePivot/AppliedVolts", mInputs.rightAppliedVolts);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "IntakePivot/TargetAngle", mTargetAngle.getDegrees());
    double kG = mGravityMap.get(mInputs.position.getDegrees());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "IntakePivot/FeedForwardVolts", kG * mInputs.position.getCos());
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("IntakePivot/kG", kG);
  }

  @Override
  public void stop() {
    synchronized (this) {
      mIsClosedLoop = false;
      mOpenLoopVoltage = 0.0;
    }
    mIO.stop();
  }

  @Override
  public void zeroSensors() {
    mIO.resetPosition(Rotation2d.fromDegrees(Constants.IntakePivot.kMinAngle));
  }
}
