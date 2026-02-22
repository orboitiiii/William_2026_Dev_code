package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.PoseHistory;
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

    // Publish pose to RobotState for vision latency compensation
    PoseHistory.getInstance()
        .addFieldToVehicleObservation(mInputs.timestamp, mOdometry.getPoseMeters());

    // Publish velocity to RobotState for shoot-on-move compensation
    PoseHistory.getInstance().addFieldVelocityObservation(mInputs.timestamp, getFieldVelocity());

    // Publish IMU motion measurements to RobotState (254 pattern)
    // Centralizes all IMU data flow to avoid duplicate Pigeon2 references
    PoseHistory.getInstance()
        .addDriveMotionMeasurements(
            mInputs.timestamp,
            mInputs.gyroYawVelocityRadPerSec,
            mInputs.gyroPitchVelocityRadPerSec,
            mInputs.gyroRollVelocityRadPerSec,
            mInputs.accelMetersPerSec2[0],
            mInputs.accelMetersPerSec2[1],
            mInputs.accelMetersPerSec2[2]);
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    switch (mCurrentState) {
      case STOP:
        mDesiredChassisSpeeds.vxMetersPerSecond = 0;
        mDesiredChassisSpeeds.vyMetersPerSecond = 0;
        mDesiredChassisSpeeds.omegaRadiansPerSecond = 0;
        // Do NOT run PID in STOP state. Just output 0V to prevent jitter on enable.
        for (int i = 0; i < 4; i++) {
          mIO.setDriveVoltage(i, 0);
          mIO.setSteerVoltage(i, 0);
        }
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

  private boolean mBorderProtectionEnabled = true;
  private double mConstraintSpeedFactor = 1.0;

  public synchronized void setTeleopInputs(
      Translation2d translation, double rotation, boolean fieldRelative) {
    double vx = translation.getX() * Constants.Swerve.kMaxDriveVelocity * mConstraintSpeedFactor;
    double vy = translation.getY() * Constants.Swerve.kMaxDriveVelocity * mConstraintSpeedFactor;
    double omega = rotation * Constants.Swerve.kMaxAngularVelocity * mConstraintSpeedFactor;

    if (mBorderProtectionEnabled
        && !frc.robot.DashboardState.getInstance().isDriveProtectDisabled()) {
      // 1. Only protect if intake pivot is down AND wheels are spinning
      boolean intakeActive =
          frc.robot.GlobalData.pivotWantsDown
              && (frc.robot.GlobalData.intakeWheelWantedState
                      == frc.robot.GlobalData.IntakeActiveState.FORWARD
                  || frc.robot.GlobalData.intakeWheelWantedState
                      == frc.robot.GlobalData.IntakeActiveState.REVERSE);

      if (intakeActive) {
        // Compute the field-relative velocity vector
        double fieldVx =
            fieldRelative
                ? vx
                : vx
                        * Math.cos(
                            frc.robot.subsystems.RobotStateEstimator.getInstance()
                                .getEstimatedPose()
                                .getRotation()
                                .getRadians())
                    - vy
                        * Math.sin(
                            frc.robot.subsystems.RobotStateEstimator.getInstance()
                                .getEstimatedPose()
                                .getRotation()
                                .getRadians());
        double fieldVy =
            fieldRelative
                ? vy
                : vx
                        * Math.sin(
                            frc.robot.subsystems.RobotStateEstimator.getInstance()
                                .getEstimatedPose()
                                .getRotation()
                                .getRadians())
                    + vy
                        * Math.cos(
                            frc.robot.subsystems.RobotStateEstimator.getInstance()
                                .getEstimatedPose()
                                .getRotation()
                                .getRadians());

        double robotX =
            frc.robot.subsystems.RobotStateEstimator.getInstance().getEstimatedPose().getX();
        double robotY =
            frc.robot.subsystems.RobotStateEstimator.getInstance().getEstimatedPose().getY();

        // Dynamic Scaling Factor: 1.0 = full speed, 0.0 = full stop
        double dynamicScaleX = 1.0;
        double dynamicScaleY = 1.0;

        // Helper constants for scaling
        final double kDangerZoneRadius = 1.0; // Starts slowing down 1m before collision
        final double kMaxSlowdownVelocity =
            2.0; // The threshold velocity (m/s) that leads to max clamping
        final double kAbsoluteMinSpeedClamp = 0.2; // Floor of the speed limiter

        // Check 1: Are we driving out of bounds? (Field Edges)
        if (robotX > (frc.robot.FieldConstants.fieldLength - kDangerZoneRadius) && fieldVx > 0) {
          double depth = robotX - (frc.robot.FieldConstants.fieldLength - kDangerZoneRadius);
          double speedFactor =
              Math.max(
                  kAbsoluteMinSpeedClamp,
                  1.0 - (depth / kDangerZoneRadius) * (fieldVx / kMaxSlowdownVelocity));
          dynamicScaleX = Math.min(dynamicScaleX, speedFactor);
        }
        if (robotX < kDangerZoneRadius && fieldVx < 0) {
          double depth = kDangerZoneRadius - robotX;
          double speedFactor =
              Math.max(
                  kAbsoluteMinSpeedClamp,
                  1.0 - (depth / kDangerZoneRadius) * (Math.abs(fieldVx) / kMaxSlowdownVelocity));
          dynamicScaleX = Math.min(dynamicScaleX, speedFactor);
        }
        if (robotY > (frc.robot.FieldConstants.fieldWidth - kDangerZoneRadius) && fieldVy > 0) {
          double depth = robotY - (frc.robot.FieldConstants.fieldWidth - kDangerZoneRadius);
          double speedFactor =
              Math.max(
                  kAbsoluteMinSpeedClamp,
                  1.0 - (depth / kDangerZoneRadius) * (fieldVy / kMaxSlowdownVelocity));
          dynamicScaleY = Math.min(dynamicScaleY, speedFactor);
        }
        if (robotY < kDangerZoneRadius && fieldVy < 0) {
          double depth = kDangerZoneRadius - robotY;
          double speedFactor =
              Math.max(
                  kAbsoluteMinSpeedClamp,
                  1.0 - (depth / kDangerZoneRadius) * (Math.abs(fieldVy) / kMaxSlowdownVelocity));
          dynamicScaleY = Math.min(dynamicScaleY, speedFactor);
        }

        // Define generic bounding boxes for obstacles [MinX, MaxX, MinY, MaxY]
        // Including Blue and Red side obstacles.
        double[][] obstacleBounds = {
          // Blue Hub
          {
            frc.robot.FieldConstants.Hub.nearLeftCorner.getX(),
            frc.robot.FieldConstants.Hub.farRightCorner.getX(),
            frc.robot.FieldConstants.Hub.nearRightCorner.getY(),
            frc.robot.FieldConstants.Hub.nearLeftCorner.getY()
          },
          // Red Hub (Opposing)
          {
            frc.robot.FieldConstants.Hub.oppNearRightCorner.getX(),
            frc.robot.FieldConstants.Hub.oppFarLeftCorner.getX(),
            frc.robot.FieldConstants.Hub.oppNearRightCorner.getY(),
            frc.robot.FieldConstants.Hub.oppNearLeftCorner.getY()
          },

          // Blue Tower
          {
            frc.robot.FieldConstants.Tower.frontFaceX - frc.robot.FieldConstants.Tower.depth,
            frc.robot.FieldConstants.Tower.frontFaceX,
            frc.robot.FieldConstants.Tower.centerPoint.getY()
                - frc.robot.FieldConstants.Tower.width / 2.0,
            frc.robot.FieldConstants.Tower.centerPoint.getY()
                + frc.robot.FieldConstants.Tower.width / 2.0
          },
          // Red Tower
          {
            frc.robot.FieldConstants.fieldLength - frc.robot.FieldConstants.Tower.frontFaceX,
            frc.robot.FieldConstants.fieldLength
                - frc.robot.FieldConstants.Tower.frontFaceX
                + frc.robot.FieldConstants.Tower.depth,
            frc.robot.FieldConstants.Tower.oppCenterPoint.getY()
                - frc.robot.FieldConstants.Tower.width / 2.0,
            frc.robot.FieldConstants.Tower.oppCenterPoint.getY()
                + frc.robot.FieldConstants.Tower.width / 2.0
          },

          // Blue Left Bump
          {
            frc.robot.FieldConstants.LeftBump.nearLeftCorner.getX(),
            frc.robot.FieldConstants.LeftBump.farRightCorner.getX(),
            frc.robot.FieldConstants.LeftBump.nearLeftCorner.getY()
                - frc.robot.FieldConstants.LeftBump.depth,
            frc.robot.FieldConstants.LeftBump.nearLeftCorner.getY()
          },
          // Blue Right Bump
          {
            frc.robot.FieldConstants.RightBump.nearLeftCorner.getX(),
            frc.robot.FieldConstants.RightBump.farRightCorner.getX(),
            frc.robot.FieldConstants.RightBump.nearLeftCorner.getY()
                - frc.robot.FieldConstants.RightBump.depth,
            frc.robot.FieldConstants.RightBump.nearLeftCorner.getY()
          },

          // Red Left Bump (Opposing)
          {
            frc.robot.FieldConstants.LeftBump.oppNearLeftCorner.getX(),
            frc.robot.FieldConstants.LeftBump.oppFarRightCorner.getX(),
            frc.robot.FieldConstants.LeftBump.oppNearLeftCorner.getY()
                - frc.robot.FieldConstants.LeftBump.depth,
            frc.robot.FieldConstants.LeftBump.oppNearLeftCorner.getY()
          },
          // Red Right Bump (Opposing)
          {
            frc.robot.FieldConstants.RightBump.oppNearLeftCorner.getX(),
            frc.robot.FieldConstants.RightBump.oppFarRightCorner.getX(),
            frc.robot.FieldConstants.RightBump.oppNearLeftCorner.getY()
                - frc.robot.FieldConstants.RightBump.depth,
            frc.robot.FieldConstants.RightBump.oppNearLeftCorner.getY()
          }
        };

        for (double[] bound : obstacleBounds) {
          double minX = bound[0];
          double maxX = bound[1];
          double minY = bound[2];
          double maxY = bound[3];

          // Inflate bounds by the danger zone
          double dangerMinX = minX - kDangerZoneRadius;
          double dangerMaxX = maxX + kDangerZoneRadius;
          double dangerMinY = minY - kDangerZoneRadius;
          double dangerMaxY = maxY + kDangerZoneRadius;

          // Check if robot is inside the danger zone of this obstacle
          if (robotX > dangerMinX
              && robotX < dangerMaxX
              && robotY > dangerMinY
              && robotY < dangerMaxY) {

            // X-axis interference
            if (robotX < minX && fieldVx > 0) { // Approaching from Left
              double depth = robotX - dangerMinX;
              double speedFactor =
                  Math.max(
                      kAbsoluteMinSpeedClamp,
                      1.0 - (depth / kDangerZoneRadius) * (fieldVx / kMaxSlowdownVelocity));
              dynamicScaleX = Math.min(dynamicScaleX, speedFactor);
            } else if (robotX > maxX && fieldVx < 0) { // Approaching from Right
              double depth = dangerMaxX - robotX;
              double speedFactor =
                  Math.max(
                      kAbsoluteMinSpeedClamp,
                      1.0
                          - (depth / kDangerZoneRadius)
                              * (Math.abs(fieldVx) / kMaxSlowdownVelocity));
              dynamicScaleX = Math.min(dynamicScaleX, speedFactor);
            }

            // Y-axis interference
            if (robotY < minY && fieldVy > 0) { // Approaching from Bottom
              double depth = robotY - dangerMinY;
              double speedFactor =
                  Math.max(
                      kAbsoluteMinSpeedClamp,
                      1.0 - (depth / kDangerZoneRadius) * (fieldVy / kMaxSlowdownVelocity));
              dynamicScaleY = Math.min(dynamicScaleY, speedFactor);
            } else if (robotY > maxY && fieldVy < 0) { // Approaching from Top
              double depth = dangerMaxY - robotY;
              double speedFactor =
                  Math.max(
                      kAbsoluteMinSpeedClamp,
                      1.0
                          - (depth / kDangerZoneRadius)
                              * (Math.abs(fieldVy) / kMaxSlowdownVelocity));
              dynamicScaleY = Math.min(dynamicScaleY, speedFactor);
            }
          }
        }
        double minScale = Math.min(dynamicScaleX, dynamicScaleY);
        vx *= minScale;
        vy *= minScale;
      }
    }

    if (fieldRelative) {
      Rotation2d fieldHeading = mOdometry.getPoseMeters().getRotation();
      // Reuse existing object to avoid allocation in hot path
      ChassisSpeeds fieldSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, fieldHeading);

      // FIX: Do not overwrite trajectory commands if we are in PATH_FOLLOWING mode
      if (mCurrentState != DriveState.PATH_FOLLOWING) {
        mDesiredChassisSpeeds.vxMetersPerSecond = fieldSpeeds.vxMetersPerSecond;
        mDesiredChassisSpeeds.vyMetersPerSecond = fieldSpeeds.vyMetersPerSecond;
        mDesiredChassisSpeeds.omegaRadiansPerSecond = fieldSpeeds.omegaRadiansPerSecond;
      }
    } else {
      if (mCurrentState != DriveState.PATH_FOLLOWING) {
        mDesiredChassisSpeeds.vxMetersPerSecond = vx;
        mDesiredChassisSpeeds.vyMetersPerSecond = vy;
        mDesiredChassisSpeeds.omegaRadiansPerSecond = omega;
      }
    }
  }

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

    // Sync RobotStateEstimator to default/vision pose
    // This prevents the two estimators from permanently diverging
    // Only call if RobotStateEstimator is initialized to prevent circular
    // dependency during startup
    if (frc.robot.subsystems.RobotStateEstimator.hasInstance()) {
      frc.robot.subsystems.RobotStateEstimator.getInstance().resetPose(pose);
    }
  }

  /**
   * Sets the gyroscope yaw to a field-relative heading (Orbit2: reset gyro when close/trusted).
   *
   * <p>Call only when vision heading is trusted (e.g. close range or multi-tag). Pigeon has ~10–20
   * ms latency; avoid calling every cycle. Odometry is not reset here; the estimator will fuse
   * subsequent vision/odometry.
   *
   * @param fieldYawDegrees Desired robot heading in field frame (degrees).
   */
  public void setGyroYawFromVision(double fieldYawDegrees) {
    mIO.setGyroYaw(fieldYawDegrees);
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
      mIO.setSteerVoltage(i, 0);
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
   * Stops all motors using OPEN_LOOP state.
   *
   * <p>Unlike stop() which uses STOP state (still runs velocity control in writePeriodicOutputs),
   * this method uses OPEN_LOOP which does NOT apply any control, allowing external voltage settings
   * to persist without being overwritten.
   *
   * <p>Use this in Test Mode to prevent PID from fighting manual wheel alignment.
   */
  public synchronized void stopOpenLoop() {
    mCurrentState = DriveState.OPEN_LOOP;
    for (int i = 0; i < 4; i++) {
      mIO.setDriveVoltage(i, 0);
      mIO.setSteerVoltage(i, 0);
    }
  }

  /**
   * Sets the drive state to OPEN_LOOP without modifying motor outputs.
   *
   * <p>Use this before calling setAllDriveMotorsDutyCycle() to prevent writePeriodicOutputs from
   * overwriting external voltage control.
   */
  public synchronized void setOpenLoopState() {
    mCurrentState = DriveState.OPEN_LOOP;
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

  public synchronized SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < 4; i++) {
      mModulePositions[i] =
          mModules[i].getPosition(
              mInputs.drivePositionRotations[i], mInputs.steerPositionRotations[i]);
    }
    return mModulePositions;
  }

  public synchronized SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] =
          mModules[i].getState(
              mInputs.driveVelocityRotationsPerSec[i], mInputs.steerPositionRotations[i]);
    }
    return states;
  }

  /**
   * Returns the robot-relative velocity computed from wheel odometry.
   *
   * <p>Uses inverse kinematics to convert individual module states (wheel velocities) back to
   * chassis speeds. This represents the robot's velocity in its own reference frame.
   *
   * <p><strong>Coordinate Frame</strong>: Robot-relative (+X forward, +Y left, +ω CCW).
   *
   * @return The robot-relative velocity as ChassisSpeeds.
   */
  public synchronized ChassisSpeeds getRobotVelocity() {
    return mKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the field-relative velocity computed from wheel odometry.
   *
   * <p>Transforms the robot-relative velocity to the field coordinate system using the current
   * heading from odometry. This is required for shoot-on-move calculations where the projectile
   * inherits the robot's inertial (field-relative) velocity at launch.
   *
   * <p><strong>Physics Note</strong>: Field-relative velocity is used because the game piece's
   * trajectory is governed by Newton's first law in the inertial (field) frame.
   *
   * @return The field-relative velocity as ChassisSpeeds.
   */
  public synchronized ChassisSpeeds getFieldVelocity() {
    ChassisSpeeds robotRelative = getRobotVelocity();
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelative, mOdometry.getPoseMeters().getRotation());
  }

  public SwerveDriveKinematics getKinematics() {
    return mKinematics;
  }

  // ============================================================
  // PER-STATE OPERATE METHODS (Orbit 1690 Pattern)
  // ============================================================
  // Drive is special: it keeps its own internal DriveState (TELEOP_DRIVE,
  // PATH_FOLLOWING, etc.) but reads constraints from GlobalData.

  @Override
  public void travelOperate() {
    // Auto mode: trajectory thread controls Drive via setPathFollowingSpeeds().
    // Skip joystick input to prevent overwriting MPC commands.
    if (mCurrentState == DriveState.PATH_FOLLOWING) {
      return;
    }
    var control = frc.robot.ControlBoard.getInstance();
    setTeleopInputs(control.getTranslation(), control.getRotation(), true);
  }

  @Override
  public void intakeOperate() {
    if (mCurrentState == DriveState.PATH_FOLLOWING) {
      return;
    }
    var control = frc.robot.ControlBoard.getInstance();
    setTeleopInputs(control.getTranslation(), control.getRotation(), true);
  }

  @Override
  public void scoreOperate() {
    if (mCurrentState == DriveState.PATH_FOLLOWING) {
      return;
    }
    var control = frc.robot.ControlBoard.getInstance();
    setTeleopInputs(control.getTranslation(), control.getRotation(), true);
  }

  @Override
  public void passOperate() {
    if (mCurrentState == DriveState.PATH_FOLLOWING) {
      return;
    }
    var control = frc.robot.ControlBoard.getInstance();
    setTeleopInputs(control.getTranslation(), control.getRotation(), true);
  }

  @Override
  public void climbOperate() {
    if (mCurrentState == DriveState.PATH_FOLLOWING) {
      return;
    }
    var control = frc.robot.ControlBoard.getInstance();
    setTeleopInputs(control.getTranslation(), control.getRotation(), true);
  }

  @Override
  public void calibrateOperate() {
    stop();
    var control = frc.robot.ControlBoard.getInstance();
    if (control.getCalibrateButton()) {
      executeCalibrationDirect();
    }
    if (control.getZeroGyro()) {
      zeroSensors();
    }
  }

  // ============================================================
  // TEST MODE (Distributed Pattern)
  // ============================================================

  /** Test routine selector for Drive. */
  public enum DriveTestRoutine {
    SWERVE_CALIBRATION,
    DRIVE_DIRECTION_TEST
  }

  private DriveTestRoutine mDriveTestRoutine = DriveTestRoutine.SWERVE_CALIBRATION;

  @Override
  public void handleTestMode(frc.robot.ControlBoard control) {
    switch (mDriveTestRoutine) {
      case SWERVE_CALIBRATION -> {
        stop();
        if (control.getCalibrateButton()) {
          executeCalibrationDirect();
        }
        if (control.getZeroGyro()) {
          zeroSensors();
        }
      }
      case DRIVE_DIRECTION_TEST -> {
        setOpenLoopState();
        setAllDriveMotorsDutyCycle(0.1);
      }
    }
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Drive.this) {
              // Clear any pending velocity commands
              mDesiredChassisSpeeds.vxMetersPerSecond = 0;
              mDesiredChassisSpeeds.vyMetersPerSecond = 0;
              mDesiredChassisSpeeds.omegaRadiansPerSecond = 0;

              // CRITICAL: Don't override special modes (Test/Calibrate) on startup
              // Also preserve PATH_FOLLOWING if it was set just before enabling (e.g., auto
              // init)
              if (mCurrentState != DriveState.OPEN_LOOP
                  && mCurrentState != DriveState.CALIBRATING
                  && mCurrentState != DriveState.PATH_FOLLOWING) {
                mCurrentState = DriveState.TELEOP_DRIVE;
              }
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
    boolean allOk = true;

    int[] driveFw = mIO.getDriveMotorFirmwareVersions();
    for (int i = 0; i < 4; i++) {
      if (driveFw[i] == 0) {
        allOk = false;
      }
    }

    int[] steerFw = mIO.getSteerMotorFirmwareVersions();
    for (int i = 0; i < 4; i++) {
      if (steerFw[i] == 0) {
        allOk = false;
      }
    }

    int pigeonFw = mIO.getPigeonFirmwareVersion();
    if (pigeonFw == 0) {
      allOk = false;
    }

    return allOk;
  }

  @Override
  public boolean checkConnectionPassive() {
    // Check if any of the 4 steer or 4 drive motors are disconnected
    for (int i = 0; i < 4; i++) {
      if (!mInputs.driveMotorConnected[i] || !mInputs.steerMotorConnected[i]) {
        return false;
      }
    }
    return true;
  }

  @Override
  public boolean checkSanityPassive() {
    // 1. Module Stall / Overcurrent Detection
    for (int i = 0; i < 4; i++) {
      // Drive motor stall (high voltage, low speed)
      if (Math.abs(mInputs.driveAppliedVolts[i]) > 8.0
          && Math.abs(mInputs.driveVelocityRotationsPerSec[i]) < 0.5) {
        return false;
      }

      // Massive current spike indicating physical bind
      if (mInputs.driveCurrentAmps[i] > 100.0 || mInputs.steerCurrentAmps[i] > 80.0) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void outputTelemetry() {
    frc.robot.DashboardState.getInstance().driveOK =
        checkConnectionPassive() && checkSanityPassive();
    var dashboard = frc.robot.DashboardState.getInstance();
    // dashboard.robotPose = getFusedPose();
    dashboard.robotSlipping = !checkSanityPassive();

    // Debugging Telemetry
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
        "Drive/GyroYaw", mInputs.gyroYaw.getDegrees());
    for (int i = 0; i < 4; i++) {
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
          "Drive/Module" + i + "/AbsPos", mInputs.steerAbsolutePositionRotations[i]);
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
          "Drive/Module" + i + "/SteerPos", mInputs.steerPositionRotations[i]);
      edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber(
          "Drive/Module" + i + "/DrivePos", mInputs.drivePositionRotations[i]);
    }
  }

  // --- Test Mode Diagnostics ---

  /**
   * Publishes Swerve control error diagnostics.
   *
   * <p>Called during Test Mode to provide real-time feedback for PID tuning. Output is written to
   * DashboardState fields.
   */
  public synchronized void publishTestDiagnostics() {
    // No-op: diagnostics output via DashboardState
  }

  /**
   * Runs a Swerve test with a specified chassis speed and publishes diagnostics.
   *
   * <p>This method sets the desired chassis speeds and caches the setpoint states for error
   * calculation.
   *
   * @param vx Forward velocity in m/s (robot-relative).
   * @param vy Strafe velocity in m/s (robot-relative).
   * @param omega Angular velocity in rad/s.
   */
  public synchronized void runDiagnosticDrive(double vx, double vy, double omega) {
    mDesiredChassisSpeeds.vxMetersPerSecond = vx;
    mDesiredChassisSpeeds.vyMetersPerSecond = vy;
    mDesiredChassisSpeeds.omegaRadiansPerSecond = omega;
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
