package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.DashboardState;
import frc.robot.RobotState;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveSetpointGenerator;

/**
 * Drive Subsystem - Swerve Drive Controller.
 *
 * <p>This subsystem manages the complete swerve drive system including:
 *
 * <ul>
 *   <li><strong>Kinematics</strong>: Chassis speed ↔ module state conversion
 *   <li><strong>Odometry</strong>: Wheel-based position estimation
 *   <li><strong>Control</strong>: Velocity closed-loop with slip protection
 * </ul>
 *
 * <p><strong>State Machine</strong>:
 *
 * <pre>
 * STOP ←──────────────────────────────────────────────┐
 *   │                                                 │
 *   ├──→ TELEOP_DRIVE (Operator ChassisSpeeds)        │
 *   │                                                 │
 *   ├──→ PATH_FOLLOWING (Trajectory ChassisSpeeds)    │
 *   │                                                 │
 *   ├──→ OPEN_LOOP (Direct voltage, testing)          │
 *   │                                                 │
 *   └──→ CALIBRATING (Module offset calibration)  ────┘
 * </pre>
 *
 * <p><strong>Coordinate Frame</strong>: All poses and speeds use the WPILib field coordinate system
 * (origin at field corner, +X toward red alliance, +Y to driver's left, +θ counterclockwise).
 *
 * @see SwerveModule
 * @see SwerveSetpointGenerator
 * @see ESUKF
 */
public class Drive extends Subsystem {
  private static Drive mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global Drive instance.
   */
  public static Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  // --- Hardware Abstraction ---
  private final DriveIO mIO;
  private final DriveIO.DriveIOInputs mInputs = new DriveIO.DriveIOInputs();

  // --- Logic Components ---
  private final SwerveModule[] mModules;
  private final SwerveDriveKinematics mKinematics;
  private final SwerveDriveOdometry mOdometry;
  private final SwerveSetpointGenerator mSetpointGenerator;

  /** Reusable buffer for odometry updates (avoids allocation). */
  private final SwerveModulePosition[] mModulePositions;

  // --- State Machine ---

  /** Drive subsystem operating states. */
  public enum DriveState {
    /** Direct voltage control for testing and SysId. */
    OPEN_LOOP,
    /** Operator-controlled driving via ChassisSpeeds. */
    TELEOP_DRIVE,
    /** Autonomous trajectory following. */
    PATH_FOLLOWING,
    /** Swerve module offset calibration mode. */
    CALIBRATING,
    /** All motors stopped, safe shutdown state. */
    STOP
  }

  private DriveState mCurrentState = DriveState.STOP;

  /** Cached desired chassis speeds (set by teleop or path following). */
  private ChassisSpeeds mDesiredChassisSpeeds = new ChassisSpeeds();

  private Drive() {
    mIO = new DriveIOReal();

    mModules =
        new SwerveModule[] {
          new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
        };

    mKinematics =
        new SwerveDriveKinematics(
            Constants.Swerve.kFLPos,
            Constants.Swerve.kFRPos,
            Constants.Swerve.kBLPos,
            Constants.Swerve.kBRPos);

    mOdometry =
        new SwerveDriveOdometry(
            mKinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            });

    mSetpointGenerator = new SwerveSetpointGenerator(mKinematics);
    mModulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };
    try {
      Thread.sleep(250);
    } catch (InterruptedException e) {
    }
    mIO.updateInputs(mInputs);
    zeroSensors();
  }

  @Override
  public synchronized void readPeriodicInputs() {
    mIO.updateInputs(mInputs);

    // Update odometry with latest wheel positions
    for (int i = 0; i < 4; i++) {
      mModules[i].updatePosition(
          mInputs.drivePositionRotations[i],
          mInputs.steerPositionRotations[i],
          mModulePositions[i]);
    }

    mOdometry.update(mInputs.gyroYaw, mModulePositions);

    // Publish to RobotState for vision latency compensation
    RobotState.getInstance()
        .addFieldToVehicleObservation(mInputs.timestamp, mOdometry.getPoseMeters());
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    switch (mCurrentState) {
      case STOP:
        mDesiredChassisSpeeds = new ChassisSpeeds(0, 0, 0);
        handleVelocityControl();
        break;

      case TELEOP_DRIVE:
        handleVelocityControl();
        break;

      case OPEN_LOOP:
        // Open-loop voltage control handled externally
        break;

      case PATH_FOLLOWING:
        handleVelocityControl();
        break;

      case CALIBRATING:
        handleCalibration();
        break;
    }
  }

  /**
   * Executes velocity closed-loop control for all modules.
   *
   * <p>Converts chassis speeds → module states → motor commands.
   */
  private void handleVelocityControl() {
    SwerveModuleState[] setpointStates =
        mSetpointGenerator.generateSetpoint(mDesiredChassisSpeeds, mInputs.driveSupplyVoltage);

    for (int i = 0; i < 4; i++) {
      SwerveModule.ModuleIO ioCmd =
          mModules[i].updateSetpoint(setpointStates[i], mInputs.steerPositionRotations[i], false);

      mIO.setDriveVelocity(i, ioCmd.driveDemand);
      mIO.setSteerPosition(i, ioCmd.steerDemand);
    }
  }

  // --- Public API ---

  /**
   * Sets teleop drive inputs.
   *
   * <p>Converts normalized joystick inputs to robot chassis speeds.
   *
   * @param translation Normalized translation vector (X=forward, Y=left).
   * @param rotation Normalized rotation rate (positive = CCW).
   * @param fieldRelative True for field-relative driving.
   */
  public synchronized void setTeleopInputs(
      Translation2d translation, double rotation, boolean fieldRelative) {
    double vx = translation.getX() * Constants.Swerve.kMaxDriveVelocity;
    double vy = translation.getY() * Constants.Swerve.kMaxDriveVelocity;
    double omega = rotation * Constants.Swerve.kMaxAngularVelocity;

    if (fieldRelative) {
      // CRITICAL: Use Odometry rotation (which includes zero-offset) for
      // field-relative drive.
      // mInputs.gyroYaw is raw hardware yaw and doesn't respect the "Zero Gyro" user
      // operation.
      // If the hardware is mounted 180° offset, raw yaw causes inverted controls even
      // after zeroing.
      Rotation2d fieldHeading = mOdometry.getPoseMeters().getRotation();
      mDesiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, fieldHeading);
    } else {
      mDesiredChassisSpeeds = new ChassisSpeeds(vx, vy, omega);
    }
  }

  /** Transitions to teleop drive state. */
  public synchronized void startTeleop() {
    mCurrentState = DriveState.TELEOP_DRIVE;
  }

  /** Transitions to path following state. */
  public synchronized void startPathFollowing() {
    mCurrentState = DriveState.PATH_FOLLOWING;
  }

  /**
   * Sets chassis speeds for path following.
   *
   * <p>Only applies if currently in PATH_FOLLOWING state.
   *
   * @param speeds Robot-relative chassis speeds from trajectory follower.
   */
  public synchronized void setPathFollowingSpeeds(ChassisSpeeds speeds) {
    if (mCurrentState == DriveState.PATH_FOLLOWING) {
      mDesiredChassisSpeeds = speeds;
    }
  }

  /**
   * Resets odometry to a known pose.
   *
   * <p>This method should be called when the robot's absolute position is known (e.g., from vision
   * or at match start).
   *
   * @param pose The new robot pose in field coordinates.
   */
  public synchronized void resetOdometry(Pose2d pose) {
    // FIX: Do NOT set hardware gyro here. CTRE/Pigeon setYaw has latency
    // (~10-20ms).
    // If we setYaw(0) and immediately read gyro, we get the OLD value (e.g. 45).
    // The odometry would then calculate Offset = 0 - 45 = -45.
    // When gyro eventually updates to 0, Odometry sees 0 - 45 = -45, causing
    // rotation.

    // Instead, we use the CURRENT gyro reading and tell Odometry "This reading
    // corresponds to Pose
    // X".
    // Odometry calculates the offset instantly: Offset = TargetRot - CurrentGyro.
    mOdometry.resetPosition(mInputs.gyroYaw, mModulePositions, pose);

    System.out.println("[Drive] Odometry reset to: " + pose);
  }

  /**
   * Returns the current robot pose from odometry.
   *
   * <p>Historically "FusedPose" meant ESUKF fusion. Now simplified to pure odometry. Kept for API
   * compatibility.
   *
   * @return The robot pose in field coordinates.
   */
  public synchronized Pose2d getFusedPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * Returns the raw odometry pose (before ESUKF fusion).
   *
   * <p>Useful for debugging to compare against fused pose.
   *
   * @return The raw wheel odometry pose.
   */
  public synchronized Pose2d getOdometryPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * Returns the current sensor inputs for diagnostics and characterization.
   *
   * @return The latest DriveIOInputs snapshot.
   */
  public synchronized DriveIO.DriveIOInputs getInputs() {
    return mInputs;
  }

  // --- Calibration API ---

  private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
  private boolean mCalibrationExecuted = false;

  /**
   * Initiates swerve module offset calibration.
   *
   * <p><strong>Precondition</strong>: All wheels must be manually aligned to 0 degrees (straight
   * forward) before calling this method.
   */
  public synchronized void startCalibration() {
    mCalibrationExecuted = false;
    mCurrentState = DriveState.CALIBRATING;
    System.out.println("[Drive] Starting Swerve module calibration...");
  }

  /**
   * Executes calibration outside the normal state machine.
   *
   * <p>Called directly from Test Mode where writePeriodicOutputs is bypassed. Reads CANcoder
   * positions, computes offsets, applies them, and plays an audible confirmation tone.
   *
   * <p><strong>Offset Calculation</strong>:
   *
   * <pre>
   * Given: A = current absolute position (with old offset)
   *        O_old = current MagnetOffset
   *        R = true raw position = A - O_old
   *
   * Goal: Find O_new such that A_new = 0 when wheel is at 0°
   *       0 = R + O_new  →  O_new = -R = -(A - O_old) = O_old - A
   * </pre>
   */
  public synchronized void executeCalibrationDirect() {
    double[] absolutePositions = mIO.getAbsolutePositionsRotations();
    double[] currentOffsets = mIO.getCurrentMagnetOffsets();

    double[] newOffsets = new double[4];
    for (int i = 0; i < 4; i++) {
      newOffsets[i] = currentOffsets[i] - absolutePositions[i];

      // Normalize to [-0.5, 0.5] rotations
      while (newOffsets[i] > 0.5) newOffsets[i] -= 1.0;
      while (newOffsets[i] < -0.5) newOffsets[i] += 1.0;
    }

    for (int i = 0; i < 4; i++) {
      mIO.setMagnetOffset(i, newOffsets[i]);
    }

    printCalibrationResults(absolutePositions, newOffsets);

    // Audible confirmation (C5 note ≈ 523 Hz)
    mIO.playTone(523.0);

    // Stop tone after 500ms (non-blocking)
    new Thread(
            () -> {
              try {
                Thread.sleep(500);
              } catch (InterruptedException ignored) {
              }
              synchronized (Drive.this) {
                mIO.playTone(0);
              }
            })
        .start();
  }

  /** Handles calibration within the state machine. */
  private void handleCalibration() {
    if (mCalibrationExecuted) {
      return;
    }

    executeCalibrationDirect();
    mCalibrationExecuted = true;
    mCurrentState = DriveState.STOP;
  }

  private void printCalibrationResults(double[] rawPositions, double[] newOffsets) {
    System.out.println("========================================");
    System.out.println("[Swerve Calibration] Calibration Complete!");
    System.out.println("========================================");
    System.out.printf("%-8s | %-14s | %-14s%n", "Module", "Raw Position", "New Offset");
    System.out.println("---------|----------------|----------------");
    for (int i = 0; i < 4; i++) {
      System.out.printf(
          "%-8s | %14.8f | %14.8f%n",
          MODULE_NAMES[i] + " (" + i + ")", rawPositions[i], newOffsets[i]);
    }
    System.out.println("========================================");
    System.out.println("Copy these values to Constants.java:");
    System.out.printf("kFLOffset = %.8f;%n", newOffsets[0]);
    System.out.printf("kFROffset = %.8f;%n", newOffsets[1]);
    System.out.printf("kBLOffset = %.8f;%n", newOffsets[2]);
    System.out.printf("kBROffset = %.8f;%n", newOffsets[3]);
    System.out.println("========================================");
  }

  /**
   * Sets all drive motors to a fixed duty cycle (Test Mode diagnostic).
   *
   * @param dutyCycle Duty cycle from -1.0 to 1.0.
   */
  public synchronized void setAllDriveMotorsDutyCycle(double dutyCycle) {
    double voltage = dutyCycle * 12.0;
    for (int i = 0; i < 4; i++) {
      mIO.setDriveVoltage(i, voltage);
    }
  }

  /** Stops all motors and transitions to STOP state. */
  public synchronized void stop() {
    mCurrentState = DriveState.STOP;
    for (int i = 0; i < 4; i++) {
      mIO.setDriveVoltage(i, 0);
      mIO.setSteerVoltage(i, 0);
    }
  }

  /**
   * Returns the current gyroscope heading.
   *
   * @return The robot heading as a Rotation2d.
   */
  public Rotation2d getHeading() {
    return mInputs.gyroYaw;
  }

  @Override
  public void zeroSensors() {
    resetOdometry(new Pose2d());
  }

  private SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < 4; i++) {
      mModulePositions[i] =
          mModules[i].getPosition(
              mInputs.drivePositionRotations[i], mInputs.steerPositionRotations[i]);
    }
    return mModulePositions;
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Drive.this) {
              // Clear any pending velocity commands
              mDesiredChassisSpeeds = new ChassisSpeeds(0, 0, 0);

              mCurrentState = DriveState.TELEOP_DRIVE;
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

  // --- Telemetry ---

  private SwerveModuleState[] mDesiredModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  /** Cached setpoint states for diagnostics (updated during velocity control). */
  private SwerveModuleState[] mLastSetpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  @Override
  public synchronized boolean checkConnectionActive() {
    // Active Query: Read Firmware Versions from all CAN devices
    System.out.println("[Drive] Running Active Connection Check...");
    boolean allOk = true;

    // Check Drive Motors (4x)
    int[] driveFw = mIO.getDriveMotorFirmwareVersions();
    for (int i = 0; i < 4; i++) {
      if (driveFw[i] == 0) {
        System.err.println("[Drive] Drive Motor " + i + " Firmware Read Failed!");
        allOk = false;
      } else {
        System.out.println("[Drive] Drive Motor " + i + " FW: " + driveFw[i]);
      }
    }

    // Check Steer Motors (4x)
    int[] steerFw = mIO.getSteerMotorFirmwareVersions();
    for (int i = 0; i < 4; i++) {
      if (steerFw[i] == 0) {
        System.err.println("[Drive] Steer Motor " + i + " Firmware Read Failed!");
        allOk = false;
      } else {
        System.out.println("[Drive] Steer Motor " + i + " FW: " + steerFw[i]);
      }
    }

    // Check Pigeon
    int pigeonFw = mIO.getPigeonFirmwareVersion();
    if (pigeonFw == 0) {
      System.err.println("[Drive] Pigeon Firmware Read Failed!");
      allOk = false;
    } else {
      System.out.println("[Drive] Pigeon FW: " + pigeonFw);
    }

    System.out.println("[Drive] Active Connection Check Complete. All OK: " + allOk);
    return allOk;
  }

  @Override
  public synchronized boolean checkConnectionPassive() {
    // Passive: Check cached status from IO layer
    if (!mInputs.pigeonConnected) {
      return false;
    }
    for (int i = 0; i < 4; i++) {
      if (!mInputs.driveMotorConnected[i]) {
        return false;
      }
      if (!mInputs.steerMotorConnected[i]) {
        return false;
      }
      if (!mInputs.cancoderConnected[i]) {
        return false;
      }
    }
    return true;
  }

  @Override
  public synchronized boolean checkSanityPassive() {
    // kV Model Sanity Check: ω_expected = V_applied / kV
    // Only check when voltage is applied (avoid divide-by-zero false positives)
    double kV = Constants.Swerve.Control.kDrivekV;
    double threshold = 4.0; // Rot/s tolerance

    for (int i = 0; i < 4; i++) {
      double volts = Math.abs(mInputs.driveAppliedVolts[i]);
      if (volts > 2.0) { // Only check if significant voltage applied
        double omegaExpected = volts / kV;
        double omegaMeasured = Math.abs(mInputs.driveVelocityRotationsPerSec[i]);
        double error = Math.abs(omegaMeasured - omegaExpected);

        if (error > threshold) {
          System.err.println(
              "[Drive] Module "
                  + i
                  + " Sanity Fail: Expected ω="
                  + omegaExpected
                  + " Measured ω="
                  + omegaMeasured);
          return false;
        }
      }
    }
    return true;
  }

  @Override
  public void outputTelemetry() {
    DashboardState.getInstance().robotPose = getFusedPose();
  }

  // --- Test Mode Diagnostics ---

  /**
   * Publishes Swerve control error diagnostics to SmartDashboard.
   *
   * <p>Called during Test Mode to provide real-time feedback for PID tuning. Displays for each
   * module:
   *
   * <ul>
   *   <li><strong>Steer Error</strong>: Angle error in degrees (setpoint - actual)
   *   <li><strong>Velocity Error</strong>: Speed error in m/s (setpoint - actual)
   *   <li><strong>Actual Velocity</strong>: Current wheel velocity in m/s
   *   <li><strong>Setpoint Velocity</strong>: Commanded wheel velocity in m/s
   * </ul>
   *
   * <p><strong>Usage</strong>: Call this from testPeriodic() while running a velocity test.
   */
  public synchronized void publishTestDiagnostics() {
    for (int i = 0; i < 4; i++) {
      // Get actual module state from sensors
      // CRITICAL: Use CANcoder absolute position (normalized to [-0.5, 0.5]
      // rotations)
      // instead of TalonFX accumulated position which can be e.g. 4321 rotations
      double actualAngleRotations = mInputs.steerAbsolutePositionRotations[i];
      double actualVelocityRotPerSec = mInputs.driveVelocityRotationsPerSec[i];
      double actualVelocityMps =
          actualVelocityRotPerSec * Constants.Swerve.Control.kWheelCircumference;

      // Get setpoint from last control cycle
      double setpointAngleDeg = mLastSetpointStates[i].angle.getDegrees();
      double setpointVelocityMps = mLastSetpointStates[i].speedMetersPerSecond;

      // Calculate errors - CANcoder is already normalized
      double actualAngleDeg = actualAngleRotations * 360.0;
      double steerErrorDeg = setpointAngleDeg - actualAngleDeg;
      // Normalize to [-180, 180]
      while (steerErrorDeg > 180) steerErrorDeg -= 360;
      while (steerErrorDeg < -180) steerErrorDeg += 360;

      double velocityErrorMps = setpointVelocityMps - actualVelocityMps;

      // Publish to SmartDashboard
      SmartDashboard.putNumber("Swerve/" + MODULE_NAMES[i] + "/SteerError_deg", steerErrorDeg);
      SmartDashboard.putNumber(
          "Swerve/" + MODULE_NAMES[i] + "/VelocityError_mps", velocityErrorMps);
      SmartDashboard.putNumber(
          "Swerve/" + MODULE_NAMES[i] + "/ActualVelocity_mps", actualVelocityMps);
      SmartDashboard.putNumber(
          "Swerve/" + MODULE_NAMES[i] + "/SetpointVelocity_mps", setpointVelocityMps);
      SmartDashboard.putNumber("Swerve/" + MODULE_NAMES[i] + "/ActualAngle_deg", actualAngleDeg);
      SmartDashboard.putNumber(
          "Swerve/" + MODULE_NAMES[i] + "/SetpointAngle_deg", setpointAngleDeg);
    }

    // Publish gyro data
    SmartDashboard.putNumber("Swerve/Gyro/Yaw_deg", mInputs.gyroYaw.getDegrees());
    SmartDashboard.putNumber(
        "Swerve/Gyro/YawRate_degps", Math.toDegrees(mInputs.gyroYawVelocityRadPerSec));
  }

  /**
   * Runs a Swerve test with a specified chassis speed and publishes diagnostics.
   *
   * <p>This method sets the desired chassis speeds and caches the setpoint states for error
   * calculation. Call {@link #publishTestDiagnostics()} separately to publish the error data.
   *
   * @param vx Forward velocity in m/s (robot-relative).
   * @param vy Strafe velocity in m/s (robot-relative).
   * @param omega Angular velocity in rad/s.
   */
  public synchronized void runDiagnosticDrive(double vx, double vy, double omega) {
    mDesiredChassisSpeeds = new ChassisSpeeds(vx, vy, omega);
    SwerveModuleState[] setpointStates =
        mSetpointGenerator.generateSetpoint(mDesiredChassisSpeeds, mInputs.driveSupplyVoltage);

    for (int i = 0; i < 4; i++) {
      mLastSetpointStates[i] = setpointStates[i];

      SwerveModule.ModuleIO ioCmd =
          mModules[i].updateSetpoint(setpointStates[i], mInputs.steerPositionRotations[i], false);

      mIO.setDriveVelocity(i, ioCmd.driveDemand);
      mIO.setSteerPosition(i, ioCmd.steerDemand);
    }
  }

  // --- Test Utilities ---

  /**
   * Forces all modules to a specific state (testing only).
   *
   * <p>Bypasses normal kinematic optimization for offset/PID verification.
   *
   * @param targetState The state to apply to all modules.
   */
  public synchronized void runVelocityTest(SwerveModuleState targetState) {
    mCurrentState = DriveState.OPEN_LOOP;

    for (int i = 0; i < 4; i++) {
      mDesiredModuleStates[i] = targetState;

      SwerveModule.ModuleIO ioCmd =
          mModules[i].updateSetpoint(targetState, mInputs.steerPositionRotations[i], false);

      mIO.setDriveVelocity(i, ioCmd.driveDemand);
      mIO.setSteerPosition(i, ioCmd.steerDemand);
    }
  }
}
