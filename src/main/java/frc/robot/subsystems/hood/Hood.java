package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.framework.CSVLogWriter;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.framework.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BooleanSupplier;

/**
 * Hood Subsystem - Projectile Launch Angle Adjustment.
 *
 * <p>This subsystem controls the hood mechanism that adjusts the projectile launch angle for
 * optimal trajectory. Key features:
 *
 * <ul>
 *   <li><strong>Position Control</strong>: Closed-loop control with velocity feedforward
 *   <li><strong>Calibration</strong>: Support for zeroing at a known position
 *   <li><strong>Soft Limits</strong>: Prevents exceeding mechanical travel limits
 *   <li><strong>Connection Debouncing</strong>: Filters transient CAN disconnections
 *   <li><strong>Tunable PID</strong>: Runtime-adjustable gains
 * </ul>
 *
 * <p><strong>Angle Definition</strong>: Hood angle is measured from the horizontal plane. Higher
 * angles (steeper) are used for longer shots to achieve the required trajectory.
 *
 * <p><strong>Reference</strong>: Based on Team 6328's 2024 Hood architecture.
 *
 * @see HoodIO
 * @see frc.robot.subsystems.shooter.ShotCalculator
 */
public class Hood extends Subsystem {
  private static Hood mInstance;

  public static Hood getInstance() {
    if (mInstance == null) {
      mInstance = new Hood();
    }
    return mInstance;
  }

  // --- Angle Limits ---

  private static final double kMinAngle = Constants.Hood.kMinAngleRads;

  private static final double kMaxAngle = Constants.Hood.kMaxAngleRads;

  // --- Hardware Abstraction ---
  private final HoodIO mIO;
  private final HoodIO.HoodIOInputs mInputs = new HoodIO.HoodIOInputs();

  // --- Connection Monitoring ---

  /**
   * Debouncer for motor connection status.
   *
   * <p><strong>Physics Note</strong>: CAN bus communication can experience transient failures (EMI,
   * bus load spikes) that don't indicate actual hardware disconnection. A 0.5 second falling edge
   * debounce filters these false positives.
   */
  private final Debouncer mMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  /** Cached debounced motor connection status. */
  private boolean mMotorConnectedDebounced = true;

  // --- Coast Override ---

  /**
   * Coast mode override supplier.
   *
   * <p>When this returns true, the hood will coast instead of brake when disabled. Used during
   * manual mechanism positioning or maintenance.
   */
  private BooleanSupplier mCoastOverride = () -> false;

  // --- Target Tracking ---

  /** Goal angle from ShotCalculator or manual command. */
  private double mGoalAngleRads = kMinAngle;

  /** Goal velocity for feedforward (from ShotCalculator). */
  private double mGoalVelocityRadsPerSec = 0.0;

  /**
   * Offset from encoder zero to physical angle.
   *
   * <p>Not used when motor is pre-initialized to correct position in HoodIOReal.
   */
  private double mHoodOffset = 0.0;

  /**
   * Whether the hood has been zeroed (calibrated).
   *
   * <p>Defaults to true since motor is initialized to 65 degrees in HoodIOReal.
   */
  private boolean mZeroed = true;

  /** Cached "at goal" status. */
  private boolean mAtGoal = false;

  /** Open-loop mode flag: when true, writePeriodicOutputs skips PID control. */
  private boolean mOpenLoopMode = false;

  // --- SysId Integration ---

  /** SysIdRoutine for characterizing kS, kV, kA, kG. */
  private final SysIdRoutine mSysIdRoutine;

  /** Whether a SysId test is currently active. */
  private boolean mSysIdActive = false;

  private Hood() {
    if (Constants.kHasHood) {
      mIO = new HoodIOReal();
    } else {
      mIO = new HoodIO() {};
    }

    // Initialize SysId routine for Hood characterization
    // Uses Arm_Cosine gravity type since hood angle affects gravity load
    mSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config()
                .setSubsystemName("Hood")
                .setRampRate(Volts.of(0.5)) // 0.5 V/s for quasistatic (slower for precision)
                .setStepVoltage(Volts.of(2.0)) // 2V step for dynamic
                .setTimeout(Seconds.of(10.0)) // 10s safety timeout
                .setLogWriter(new CSVLogWriter("sysid_Hood")));
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Hood.this) {
              // Initialize goal to current position
              mGoalAngleRads = getMeasuredAngleRads();
            }
          }

          @Override
          public void onLoop(double timestamp) {
            // Control logic handled in writePeriodicOutputs
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  // --- Public API ---

  /**
   * Sets the goal angle and velocity from ShotCalculator.
   *
   * @param angleRads Target hood angle in radians.
   * @param velocityRadsPerSec Feedforward velocity in radians per second.
   */
  public synchronized void setGoalParams(double angleRads, double velocityRadsPerSec) {
    mGoalAngleRads = MathUtil.clamp(angleRads, kMinAngle, kMaxAngle);
    mGoalVelocityRadsPerSec = velocityRadsPerSec;
    mOpenLoopMode = false; // Exit open-loop mode when a goal is set
  }

  /**
   * Sets the goal angle with zero velocity feedforward.
   *
   * @param angleRads Target hood angle in radians.
   */
  public synchronized void setTargetAngle(double angleRads) {
    setGoalParams(angleRads, 0.0);
  }

  /**
   * Sets the hood motor voltage directly (open-loop control).
   *
   * <p><strong>Warning</strong>: Bypasses closed-loop control and soft limits. Use only for testing
   * and calibration.
   *
   * @param volts Voltage command (-12 to +12).
   */
  public synchronized void setOpenLoopVoltage(double volts) {
    mOpenLoopMode = true; // Enter open-loop mode
    mIO.setVoltage(volts);
  }

  // --- SysId API (Test Mode Only) ---

  /**
   * Starts a SysId characterization test.
   *
   * <p><strong>Usage</strong>: Call this to begin a quasistatic or dynamic test in the forward or
   * reverse direction. Data is logged to CSV for analysis.
   *
   * @param type Test type (QUASISTATIC or DYNAMIC).
   * @param direction Test direction (FORWARD or REVERSE).
   */
  public void startSysId(SysIdRoutine.TestType type, SysIdRoutine.Direction direction) {
    mSysIdActive = true;
    mOpenLoopMode = true; // Disable PID control during SysId
    mSysIdRoutine.start(type, direction);
  }

  /**
   * Stops the active SysId test and saves data to CSV.
   *
   * <p>Call this when releasing the test button to end data collection.
   */
  public void stopSysId() {
    if (mSysIdActive) {
      mSysIdRoutine.stop();
      mSysIdActive = false;
      mIO.setVoltage(0);

      // Set goal to current position so hood doesn't jump back
      mGoalAngleRads = getMeasuredAngleRads();
      mGoalVelocityRadsPerSec = 0.0;

      mOpenLoopMode = false;
    }
  }

  /**
   * Updates the SysId routine and applies the calculated voltage.
   *
   * <p>Call this every cycle while a SysId test is active.
   */
  public void updateSysId() {
    if (mSysIdActive) {
      double timestamp = Timer.getFPGATimestamp();
      // Position in rotations (mechanism rotations, not radians)
      double positionRotations = mInputs.positionRads / (2 * Math.PI);
      // Velocity in rotations per second (mechanism rotations/s)
      double velocityRotPerSec = mInputs.velocityRadsPerSec / (2 * Math.PI);

      mSysIdRoutine.update(timestamp, positionRotations, velocityRotPerSec);
      mIO.setVoltage(mSysIdRoutine.getOutputVoltage());
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

  /**
   * Enables or disables software position limits.
   *
   * <p><strong>Warning</strong>: Disabling soft limits allows travel beyond safe ranges.
   *
   * @param enabled True to enable limits, false to disable.
   */
  public synchronized void setSoftLimitsEnabled(boolean enabled) {
    mIO.setSoftLimitsEnabled(enabled);
  }

  /**
   * Sets the coast override supplier.
   *
   * <p>When the supplier returns true, the hood will coast instead of brake when disabled.
   *
   * @param override A BooleanSupplier that returns true to enable coast mode.
   */
  public void setCoastOverride(BooleanSupplier override) {
    mCoastOverride = override;
  }

  /**
   * Zeros the hood encoder at the current physical position.
   *
   * <p>Call this when the hood is at the minimum angle hard stop.
   *
   * <p><strong>Calibration Procedure</strong>: Move hood to minimum position (hard stop), then call
   * this method to establish the zero reference.
   */
  public synchronized void zero() {
    // Calculate offset: physical min angle - current encoder reading
    mHoodOffset = kMinAngle - mInputs.positionRads;
    mZeroed = true;
  }

  /**
   * Returns the current measured hood angle in radians.
   *
   * @return Physical hood angle (encoder reading + offset).
   */
  public synchronized double getMeasuredAngleRads() {
    // Motor is initialized to correct angle in HoodIOReal, offset not needed
    return mInputs.positionRads;
  }

  /**
   * Returns whether the hood is at the commanded goal.
   *
   * @return True if within position tolerance.
   */
  public synchronized boolean isAtGoal() {
    return mAtGoal;
  }

  /**
   * Returns whether the hood has been zeroed.
   *
   * @return True if zero() has been called.
   */
  public synchronized boolean isZeroed() {
    return mZeroed;
  }

  // ============================================================
  // PER-STATE OPERATE METHODS (Orbit 1690 Pattern)
  // ============================================================

  @Override
  public void travelOperate() {
    // Stow to Maximum angle (Retracted)
    setGoalParams(kMaxAngle, 0.0);
  }

  @Override
  public void scoreOperate() {
    var params = frc.robot.GlobalData.currentShotParams;
    if (params != null && params.isValid) {
      setGoalParams(params.hoodAngleRad, params.hoodVelocityRadPerSec);
    }
  }

  @Override
  public void passOperate() {
    // Hold current position explicitly with 0 velocity
    setGoalParams(getMeasuredAngleRads(), 0.0);
  }

  @Override
  public void climbOperate() {
    // Stow to minimum angle
    setTargetAngle(kMaxAngle);
  }

  @Override
  public void scoreTestOperate() {
    // Stow to minimum angle
    setTargetAngle(Math.toRadians(62.5));
  }

  // ============================================================
  // TEST MODE (Distributed Pattern)
  // ============================================================

  /** Test routine selector for Hood. */
  public enum HoodTestRoutine {
    VOLTAGE,
    SYSID,
    PID
  }

  private HoodTestRoutine mHoodTestRoutine = HoodTestRoutine.SYSID;
  private boolean mHoodSysIdButtonWasPressed = false;

  @Override
  public void handleTestMode(frc.robot.ControlBoard control) {
    switch (mHoodTestRoutine) {
      case VOLTAGE -> {
        double testVoltage = 1.0;
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
        if (mHoodSysIdButtonWasPressed && !anySysIdButtonPressed) {
          stopSysId();
        }
        mHoodSysIdButtonWasPressed = anySysIdButtonPressed;
      }
      case PID -> {
        if (control.getCrossButton()) {
          setGoalParams(Math.toRadians(62.5), 0.0);
        } else if (control.getTriangleButton()) {
          setGoalParams(Math.toRadians(82), 0.0);
        } else {
          setOpenLoopVoltage(0.0);
        }
      }
    }
  }

  // --- Subsystem Lifecycle ---

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);

    // Update debounced connection status
    mMotorConnectedDebounced = mMotorConnectedDebouncer.calculate(mInputs.motorConnected);
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    // When disabled: determine brake vs coast based on override
    if (DriverStation.isDisabled() || !mZeroed) {
      mAtGoal = false;

      if (mCoastOverride.getAsBoolean()) {
        mIO.setOutputMode(HoodIO.HoodOutputMode.COAST);
      } else {
        mIO.setOutputMode(HoodIO.HoodOutputMode.BRAKE);
      }

      mIO.stop();
      return;
    }

    // Clamp goal to legal range
    double clampedGoal = MathUtil.clamp(mGoalAngleRads, kMinAngle, kMaxAngle);

    // Skip PID control if in open-loop mode (manual voltage testing)
    if (mOpenLoopMode) {
      // Open-loop voltage is already set via setOpenLoopVoltage()
      // Just update "at goal" status (always false in open-loop)
      mAtGoal = false;
      return;
    }

    // Convert to encoder setpoint (physical angle - offset)
    double encoderSetpoint = clampedGoal - mHoodOffset;

    // Command hardware
    // Get Robot Acceleration (X-axis forward) from Drive subsystem
    // Note: Assumes X-axis acceleration is the primary disturbance
    double robotAccel = Drive.getInstance().getInputs().accelMetersPerSec2[0];

    // Calculate Arbitrary Feedforward
    // Formula: kG * cos(theta) + kARobot * sin(theta) * Acc_Robot
    // Note: kV and Mechanism kA are handled by the Talon's onboard Slot0 controller
    // kS is applied manually here to control directionality
    double angleRads = getMeasuredAngleRads();

    // Determine direction for kS application
    // If moving (non-non-zero velocity goal), use velocity direction.
    // If holding (zero velocity goal), use error direction to overcome stiction.
    double direction = Math.signum(mGoalVelocityRadsPerSec);
    if (Math.abs(mGoalVelocityRadsPerSec) < 1e-3) {
      double error = mGoalAngleRads - angleRads;
      if (Math.abs(error) > Constants.Hood.kPositionToleranceRads * 0.5) {
        direction = Math.signum(error);
      } else {
        direction = 0.0;
      }
    }

    double kS = Constants.Hood.kS * direction;

    // Only apply robot acceleration compensation if we are actively tracking/moving
    double accelComp = 0.0;
    if (Math.abs(mGoalVelocityRadsPerSec) > 1e-3) {
      accelComp = Constants.Hood.kARobot * Math.sin(angleRads) * robotAccel;
    }

    double arbFF = kS + Constants.Hood.kG * Math.cos(angleRads) + accelComp;

    mIO.setPositionSetpoint(encoderSetpoint, mGoalVelocityRadsPerSec, arbFF);

    // Check if at goal (using constant tolerance)
    double measuredAngle = getMeasuredAngleRads();
    double toleranceRads = Constants.Hood.kPositionToleranceRads;
    mAtGoal =
        DriverStation.isEnabled()
            && mZeroed
            && Math.abs(measuredAngle - clampedGoal) < toleranceRads;
  }

  @Override
  public void stop() {
    setOpenLoopVoltage(0.0);
    mIO.stop();
  }

  @Override
  public void zeroSensors() {
    // Do NOT auto-zero on startup - motor is already initialized to 65 degrees
    // Call zero() manually only when hood is at the physical hard stop
  }

  @Override
  public boolean checkConnectionActive() {
    int fw = mIO.getMotorFirmwareVersion();
    return fw != 0;
  }

  @Override
  public boolean checkConnectionPassive() {
    // Use debounced status to avoid false positives
    return mMotorConnectedDebounced;
  }

  @Override
  public boolean checkSanityPassive() {
    // Check if measured angle is within expected range
    double measured = getMeasuredAngleRads();
    return measured >= kMinAngle - 0.1 && measured <= kMaxAngle + 0.1;
  }

  @Override
  public void outputTelemetry() {
    var dashboard = frc.robot.DashboardState.getInstance();
    dashboard.elevationAngle = Math.toDegrees(getMeasuredAngleRads());
    dashboard.hoodOK = checkConnectionPassive() && checkSanityPassive();

    // SmartDashboard.putNumber("Hood/Angle",
    // Math.toDegrees(getMeasuredAngleRads()));
    // SmartDashboard.putNumber("Hood/Goal", Math.toDegrees(mGoalAngleRads));
    // SmartDashboard.putBoolean("Hood/AtGoal", mAtGoal);
    // SmartDashboard.putBoolean("Hood/OpenLoop", mOpenLoopMode);
    // SmartDashboard.putBoolean("Hood/Zeroed", mZeroed);
    // SmartDashboard.putNumber("Hood/Volts", mInputs.appliedVolts);
    // SmartDashboard.putNumber("Hood/Current", mInputs.currentAmps);
  }
}
