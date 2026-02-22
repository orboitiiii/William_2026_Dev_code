package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PoseHistory;
import frc.robot.framework.CSVLogWriter;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.framework.SysIdRoutine;

/**
 * Turret Subsystem - Field-Relative Continuous Rotation Mechanism.
 *
 * <p>This subsystem manages the turret's rotation for shoot-on-move targeting. Key features:
 *
 * <ul>
 *   <li><strong>TrapezoidProfile</strong>: Smooth motion planning with velocity/acceleration limits
 *   <li><strong>Field-Relative Tracking</strong>: Compensates for robot rotation during targeting
 *   <li><strong>Wrap-Around Handling</strong>: Selects optimal equivalent angle to minimize travel
 *   <li><strong>ShootState Modes</strong>: Different angle limits for tracking vs active shooting
 * </ul>
 *
 * <p><strong>Coordinate Frame</strong>: Positive angle is counter-clockwise when viewed from above.
 * Zero is defined as the turret facing forward relative to the robot chassis.
 *
 * <p><strong>Reference</strong>: Based on Team 6328's 2024 Turret architecture.
 *
 * @see TurretIO
 * @see frc.robot.subsystems.shooter.ShotCalculator
 */
public class Turret extends Subsystem {
  private static Turret mInstance;

  public static Turret getInstance() {
    if (mInstance == null) {
      mInstance = new Turret();
    }
    return mInstance;
  }

  // --- Angle Limit Constants ---

  /** Physical minimum angle (CCW limit). */
  private static final double kMinAngle = Constants.Turret.kMinAngleRads;

  /** Physical maximum angle (CW limit). */
  private static final double kMaxAngle = Constants.Turret.kMaxAngleRads;

  /** Overlap margin for tracking mode. */
  private static final double kTrackOverlapMargin = Constants.Turret.kTrackOverlapMarginRads;

  /** Center of the angle range. */
  private static final double kTrackCenter = (kMinAngle + kMaxAngle) / 2;

  /** Tracking mode minimum angle (allows exceeding ±180°). */
  private static final double kTrackMin = kTrackCenter - Math.PI - kTrackOverlapMargin;

  /** Tracking mode maximum angle. */
  private static final double kTrackMax = kTrackCenter + Math.PI + kTrackOverlapMargin;

  // --- State Machine ---

  /** Turret operating modes with different angle constraints. */
  public enum ShootState {
    /**
     * Active shooting mode: Uses full physical angle range ±210°.
     *
     * <p>Used when the robot is aligned and ready to shoot.
     */
    ACTIVE_SHOOTING,

    /**
     * Tracking mode: Allows exceeding ±180° for continuous tracking.
     *
     * <p>Used when passively following a target while the robot maneuvers.
     */
    TRACKING,

    /**
     * Robot relative mode: Explicitly stops field tracking and locks to a specific chassis angle.
     *
     * <p>Used for safe stowing to prevent the turret from violently reacting to chassis rotation.
     */
    ROBOT_RELATIVE
  }

  // --- Hardware Abstraction ---
  private final TurretIO mIO;
  private final TurretIO.TurretIOInputs mInputs = new TurretIO.TurretIOInputs();

  // --- Motion Profile ---
  private TrapezoidProfile.State mSetpoint = new TrapezoidProfile.State(0.0, 0.0);
  private final LinearFilter mChassisOmegaFilter = LinearFilter.movingAverage(3);

  // --- SysId Integration ---

  /** SysIdRoutine for characterizing kS, kV, kA. */
  private final SysIdRoutine mSysIdRoutine;

  /** Field-relative goal angle from ShotCalculator. */
  private double mGoalAngleRadsField = 0.0;

  /** Last selected robot-relative goal angle (for wrap-around continuity). */
  private double mLastGoalAngleRads = 0.0;

  /** Locked robot-relative goal angle. */
  private double mRobotRelativeGoalRads = 0.0;

  /** Current operating mode. */
  private ShootState mShootState = ShootState.TRACKING;

  /** Whether the turret has been zeroed (calibrated). */
  private boolean mZeroed = false;

  /** Cached "at goal" status for external queries. */
  private boolean mAtGoal = false;

  /** Open-loop mode flag: when true, writePeriodicOutputs skips closed-loop control. */
  private boolean mOpenLoopMode = false;

  // --- SysId Integration ---

  /** Whether a SysId test is currently active. */
  private boolean mSysIdActive = false;

  private Turret() {
    if (Constants.kHasTurret) {
      mIO = new TurretIOReal();
    } else {
      mIO = new TurretIO() {};
    }

    // --- CRT Absolute Angle Initialization ---
    // At startup, read the two Through Bore Encoders (CANcoders) and use the
    // Chinese Remainder Theorem to calculate the unique turret angle within one
    // rotation.
    // Then set the motor encoder to this position.
    double crtAngle = mIO.initializeAbsolutePosition();
    if (!Double.isNaN(crtAngle)) {
      mZeroed = true;
      mSetpoint = new TrapezoidProfile.State(crtAngle, 0.0);
      mLastGoalAngleRads = crtAngle;
      System.out.println("[Turret] CRT Init OK: " + Math.toDegrees(crtAngle) + " deg");
    } else {
      // Fail-Safe: mZeroed remains false -> writePeriodicOutputs will not control
      // motor
      System.err.println("[Turret] CRITICAL: CRT initialization FAILED - turret NOT zeroed");
    }

    // Initialize SysId routine for Turret characterization
    // Uses simple motor type since turret rotates horizontally (no gravity
    // component)
    mSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config()
                .setSubsystemName("Turret")
                .setRampRate(Volts.of(0.5)) // 0.5 V/s for quasistatic
                .setStepVoltage(Volts.of(2.0)) // 2V step for dynamic
                .setTimeout(Seconds.of(10.0)) // 10s safety timeout
                .setLogWriter(new CSVLogWriter("sysid_Turret")));
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Turret.this) {
              // Initialize setpoint to current position to avoid jump on enable
              mSetpoint = new TrapezoidProfile.State(getPositionRads(), 0.0);
              mLastGoalAngleRads = getPositionRads();
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
   * Sets the field-relative target angle.
   *
   * <p>Called by Superstructure with parameters from ShotCalculator.
   *
   * @param angleRads Field-relative target angle.
   */
  public synchronized void setFieldRelativeTarget(double angleRads) {
    mGoalAngleRadsField = angleRads;
    // CRITICAL FIX: Ensure we exit open-loop mode when a target is set
    mOpenLoopMode = false;
  }

  /**
   * Sets the field-relative target angle with zero velocity feedforward.
   *
   * @param angleRads Field-relative target angle.
   */
  public synchronized void setTargetAngle(double angleRads) {
    setFieldRelativeTarget(angleRads);
  }

  /**
   * Sets the robot-relative target angle, locking the turret to the chassis.
   *
   * @param angle Robot-relative target angle.
   */
  public synchronized void setRobotRelativeTarget(Rotation2d angle) {
    mRobotRelativeGoalRads = angle.getRadians();
    mShootState = ShootState.ROBOT_RELATIVE;
    mOpenLoopMode = false;
  }

  /**
   * Sets the current operating mode.
   *
   * @param state The desired ShootState.
   */
  public synchronized void setShootState(ShootState state) {
    mShootState = state;
  }

  /**
   * Sets the turret motor voltage directly (open-loop control).
   *
   * <p><strong>Warning</strong>: Bypasses closed-loop control and soft limits. Use only for testing
   * and calibration.
   *
   * @param volts Voltage command (-12 to +12).
   */
  public synchronized void setOpenLoopVoltage(double volts) {
    // Soft limits should remain enabled for safety in all modes, including
    // Test/SysId
    // if (!mOpenLoopMode) {
    // mIO.setSoftLimitsEnabled(false);
    // }
    mOpenLoopMode = true;
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

      // Set goal to current position so turret doesn't jump back
      mLastGoalAngleRads = getPositionRads();
      mGoalAngleRadsField = mLastGoalAngleRads;

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
   * Zeros the turret encoder to the current physical position.
   *
   * <p>Call this when the turret is at a known reference position (e.g., forward-facing).
   */
  public synchronized void zero() {
    mZeroed = true;
    // Reset setpoint to current position
    mSetpoint = new TrapezoidProfile.State(0.0, 0.0);
    mLastGoalAngleRads = 0.0;
  }

  /**
   * Returns the current turret position in radians.
   *
   * @return Robot-relative turret angle.
   */
  public synchronized double getPositionRads() {
    return mInputs.positionRads;
  }

  /**
   * Returns the current turret position as a Rotation2d.
   *
   * @return Robot-relative turret angle.
   */
  public synchronized Rotation2d getAngle() {
    return Rotation2d.fromRadians(mInputs.positionRads);
  }

  /**
   * Returns the current turret velocity in radians per second.
   *
   * @return Robot-relative turret angular velocity.
   */
  public synchronized double getVelocityRadsPerSec() {
    return mInputs.velocityRadsPerSec;
  }

  /**
   * Returns whether the turret is at the commanded goal.
   *
   * @return True if position and velocity are within tolerance.
   */
  public synchronized boolean isAtGoal() {
    return mAtGoal;
  }

  /**
   * Returns whether the turret has been zeroed.
   *
   * @return True if zero() has been called.
   */
  public synchronized boolean isZeroed() {
    return mZeroed;
  }

  /**
   * Returns whether a valid vision target is available for shooting.
   *
   * <p>Conditions for valid target:
   *
   * <ul>
   *   <li>Turret is at the commanded goal angle (within tolerance)
   *   <li>Turret is zeroed (calibrated)
   *   <li>Vision subsystem has recent valid pose estimate
   * </ul>
   *
   * @return True if the turret has locked onto a valid target.
   */
  public synchronized boolean hasTarget() {
    // Must be zeroed, at goal, and have recent vision data
    if (!mZeroed || !mAtGoal) {
      return false;
    }

    // Check if we have recent vision data (within 0.5 seconds)
    double timeSinceVision =
        edu.wpi.first.wpilibj.Timer.getFPGATimestamp()
            - PoseHistory.getInstance().lastUsedMegatagTimestamp();

    return timeSinceVision < 0.5;
  }

  // --- Wrap-Around Logic ---

  /**
   * Selects the optimal equivalent angle to minimize travel distance.
   *
   * <p><strong>Physics Principle</strong>: A continuous rotation turret can reach the same
   * field-relative direction via multiple robot-relative angles (e.g., 30° = 390° = -330°).
   * Selecting the closest equivalent angle minimizes motion time and mechanical wear.
   *
   * @param robotRelativeGoalRads The raw robot-relative goal angle.
   * @return The best equivalent angle within legal limits.
   */
  private double selectBestAngle(double robotRelativeGoalRads) {
    // Determine legal range based on current mode
    double minLegal = (mShootState == ShootState.ACTIVE_SHOOTING) ? kMinAngle : kTrackMin;
    double maxLegal = (mShootState == ShootState.ACTIVE_SHOOTING) ? kMaxAngle : kTrackMax;

    double bestAngle = Double.NaN;
    boolean hasBestAngle = false;

    // Check all equivalent angles within ±2 full rotations
    for (int i = -2; i <= 2; i++) {
      double candidate = robotRelativeGoalRads + Math.PI * 2.0 * i;

      // Skip if outside legal range
      if (candidate < minLegal || candidate > maxLegal) {
        continue;
      }

      if (!hasBestAngle) {
        bestAngle = candidate;
        hasBestAngle = true;
      } else if (Math.abs(mLastGoalAngleRads - candidate)
          < Math.abs(mLastGoalAngleRads - bestAngle)) {
        // Prefer the angle closest to the last goal (continuity)
        bestAngle = candidate;
      }
    }

    return bestAngle;
  }

  // ============================================================
  // PER-STATE OPERATE METHODS (Orbit 1690 Pattern)
  // ============================================================

  @Override
  public void travelOperate() {
    // Lock strictly to 0.0 degrees (forward) relative to chassis to prevent gyro
    // thrashing
    setRobotRelativeTarget(Rotation2d.kZero);
  }

  @Override
  public void scoreOperate() {
    var params = frc.robot.GlobalData.currentShotParams;
    if (params != null && params.isValid) {
      setFieldRelativeTarget(params.turretAngleRad);
      mShootState = ShootState.ACTIVE_SHOOTING;
    } else {
      mShootState = ShootState.TRACKING;
    }
  }

  @Override
  public void passOperate() {
    // Lock strictly to 0.0 degrees (forward) relative to chassis
    setRobotRelativeTarget(Rotation2d.kZero);
  }

  @Override
  public void climbOperate() {
    // Return to center for safe climbing
    setRobotRelativeTarget(Rotation2d.kZero);
  }

  // ============================================================
  // TEST MODE (Distributed Pattern)
  // ============================================================

  /** Test routine selector for Turret. */
  public enum TurretTestRoutine {
    VOLTAGE,
    SYSID,
    PID
  }

  private TurretTestRoutine mTurretTestRoutine = TurretTestRoutine.PID;
  private double mTunableTurretDeg = 0.0;
  private boolean mTurretSysIdButtonWasPressed = false;

  @Override
  public void handleTestMode(frc.robot.ControlBoard control) {
    switch (mTurretTestRoutine) {
      case VOLTAGE -> {
        double testVoltage = 3.0;
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
        if (mTurretSysIdButtonWasPressed && !anySysIdButtonPressed) {
          stopSysId();
        }
        mTurretSysIdButtonWasPressed = anySysIdButtonPressed;
      }
      case PID -> {
        // Manual Zero Override (Triangle)
        if (control.getTriangleButton()) {
          zero();
        }

        if (control.getCrossButton()) {
          setTargetAngle(Math.toRadians(-180));
        } else if (control.getCircleButton()) {
          setTargetAngle(Math.toRadians(0));
        } else if (control.getSquareButton()) {
          setTargetAngle(Math.toRadians(30));
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
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    // Skip closed-loop control if in open-loop mode (manual voltage testing or
    // SysId)
    // This check is FIRST to allow manual testing before calibration
    if (mOpenLoopMode) {
      // Open-loop voltage is already set via setOpenLoopVoltage() or SysId
      // Just update "at goal" status (always false in open-loop)
      mAtGoal = false;
      return;
    }

    // When disabled or not zeroed: hold current position as setpoint, don't move
    if (DriverStation.isDisabled() || !mZeroed) {
      mSetpoint = new TrapezoidProfile.State(getPositionRads(), 0.0);
      mLastGoalAngleRads = getPositionRads();
      mAtGoal = false;
      mIO.stop();
      return;
    }

    // Get robot state for coordinate transformation
    Rotation2d robotHeading = PoseHistory.getInstance().getLatestFieldToVehicle().getRotation();
    double rawRobotOmega = PoseHistory.getInstance().getLatestFieldVelocity().omegaRadiansPerSecond;
    double filteredRobotOmega = mChassisOmegaFilter.calculate(rawRobotOmega);

    double robotRelativeGoalRads;

    if (mShootState == ShootState.ROBOT_RELATIVE) {
      robotRelativeGoalRads = mRobotRelativeGoalRads;
    } else {
      // Transform field-relative goal to robot-relative
      robotRelativeGoalRads = mGoalAngleRadsField - robotHeading.getRadians();
    }

    // Select optimal equivalent angle (wrap-around handling)
    double bestAngle = selectBestAngle(robotRelativeGoalRads);

    // Fallback: clamp to legal range if no valid equivalent found
    if (Double.isNaN(bestAngle)) {
      bestAngle = MathUtil.clamp(robotRelativeGoalRads, kMinAngle, kMaxAngle);
    }

    mLastGoalAngleRads = bestAngle;

    // Check if PHYSICALLY at goal (using measured position instead of software
    // setpoint)
    // Velocity tracking is heavily affected by odometry noise, so we rely primarily
    // on physical position
    // coupled with Phoenix 6 onboard feedforward to ensure accurate
    // shoot-on-the-move.
    mAtGoal = Math.abs(bestAngle - getPositionRads()) < Constants.Turret.kPositionToleranceRads;

    // Chassis Counter-Rotation Feedforward
    // We want the turret to spin exactly opposite to the robot's rotation to stay
    // locked on target.
    // V = omega_rad_per_sec * (1 rot / 2pi rad) * kV_V_per_rot_per_sec
    double chassisCounterVoltage = 0.0;
    if (mShootState != ShootState.ROBOT_RELATIVE) {
      // The turret needs to move at -filteredRobotOmega relative to chassis just to
      // stay still on the field.
      // Phoenix 6 kV is calibrated in Volts / (rot/s).
      double omegaRotPerSec = -filteredRobotOmega / (2.0 * Math.PI);
      chassisCounterVoltage = omegaRotPerSec * Constants.Turret.kV;
    }

    // Update telemetry state
    mSetpoint = new TrapezoidProfile.State(bestAngle, 0.0);

    // Command hardware
    // Completely removed dynamic tracking TrapezoidProfile recalculation to prevent
    // thrashing.
    // The dual-state TurretIOReal handles switching between MotionMagic (for large
    // sweeps)
    // and PositionVoltage (for micro-tracking).
    mIO.setPositionSetpoint(bestAngle, 0.0, chassisCounterVoltage);
  }

  @Override
  public void stop() {
    setOpenLoopVoltage(0.0);
    mIO.stop();
  }

  @Override
  public void zeroSensors() {
    zero();
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
    // Check if position is within expected range
    double pos = mInputs.positionRads;
    return pos >= kMinAngle - 0.1 && pos <= kMaxAngle + 0.1;
  }

  @Override
  public void outputTelemetry() {
    var dashboard = frc.robot.DashboardState.getInstance();
    dashboard.turretAngle = Math.toDegrees(getPositionRads());
    dashboard.turretOK = checkConnectionPassive() && checkSanityPassive();

    SmartDashboard.putNumber("Turret/MotorEncoderAbsRot", mInputs.motorEncoderAbsPosRotations);
    SmartDashboard.putNumber("Turret/AuxEncoderAbsRot", mInputs.auxEncoderAbsPosRotations);
    SmartDashboard.putNumber("Turret/Angle", Math.toDegrees(getPositionRads()));

    // Suggested Offsets for Zeroing
    double suggestedMotorOffset =
        Constants.Turret.kMotorEncoderOffsetRotations - mInputs.motorEncoderAbsPosRotations;
    double suggestedAuxOffset =
        Constants.Turret.kAuxEncoderOffsetRotations - mInputs.auxEncoderAbsPosRotations;
    SmartDashboard.putNumber("Turret/SuggestedMotorOffset", suggestedMotorOffset);
    SmartDashboard.putNumber("Turret/SuggestedAuxOffset", suggestedAuxOffset);

    // Goal Debugging
    SmartDashboard.putNumber("Turret/GoalAngleFieldDeg", Math.toDegrees(mGoalAngleRadsField));
    Rotation2d robotHeading = PoseHistory.getInstance().getLatestFieldToVehicle().getRotation();
    SmartDashboard.putNumber("Turret/RobotHeadingDeg", robotHeading.getDegrees());
    double robotRelativeGoalRads = mGoalAngleRadsField - robotHeading.getRadians();
    SmartDashboard.putNumber("Turret/GoalAngleRobotDeg", Math.toDegrees(robotRelativeGoalRads));
    SmartDashboard.putNumber("Turret/SetpointDeg", Math.toDegrees(mSetpoint.position));
    SmartDashboard.putBoolean("Turret/AtGoal", mAtGoal);
    SmartDashboard.putString("Turret/ShootState", mShootState.toString());
  }
}
