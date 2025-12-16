package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.DashboardState;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveSetpointGenerator;

public class Drive extends Subsystem {
  private static Drive mInstance;

  public static Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  // State Machine Definitions
  public enum DriveMode {
    OPEN_LOOP,
    HEADING_CONTROL,
    PATH_FOLLOWING
  }

  public static class PeriodicIO {
    // Inputs
    public double timestamp;
    public Rotation2d gyro_heading = new Rotation2d();
    public SwerveModulePosition[] module_positions = new SwerveModulePosition[4];
    public SwerveModuleState[] module_states = new SwerveModuleState[4];
    public Pose2d odometry_pose = new Pose2d();

    // Outputs
    public SwerveModuleState[] curr_module_target_states =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
  }

  private final SwerveModule[] mModules;
  private final Pigeon2 mPigeon;

  private final SwerveDriveKinematics mKinematics;
  private final SwerveSetpointGenerator mSetpointGenerator;
  private final SwerveDriveOdometry mOdometry;

  // Controllers
  private final PIDController mHeadingController;

  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private DriveMode mVehicleMode = DriveMode.OPEN_LOOP;

  private ChassisSpeeds mDesiredSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  private Rotation2d mTargetHeading = new Rotation2d();
  private boolean mIsFieldRelative = true;

  private Drive() {
    mModules =
        new SwerveModule[] {
          new SwerveModule(
              "FL",
              Constants.Swerve.kFLDriveId,
              Constants.Swerve.kFLSteerId,
              Constants.Swerve.kFLEncoderId,
              Constants.Swerve.kFLOffset,
              Constants.Swerve.kFLPos),
          new SwerveModule(
              "FR",
              Constants.Swerve.kFRDriveId,
              Constants.Swerve.kFRSteerId,
              Constants.Swerve.kFREncoderId,
              Constants.Swerve.kFROffset,
              Constants.Swerve.kFRPos),
          new SwerveModule(
              "BL",
              Constants.Swerve.kBLDriveId,
              Constants.Swerve.kBLSteerId,
              Constants.Swerve.kBLEncoderId,
              Constants.Swerve.kBLOffset,
              Constants.Swerve.kBLPos),
          new SwerveModule(
              "BR",
              Constants.Swerve.kBRDriveId,
              Constants.Swerve.kBRSteerId,
              Constants.Swerve.kBREncoderId,
              Constants.Swerve.BROffset,
              Constants.Swerve.kBRPos),
        };

    mPigeon = new Pigeon2(Constants.Swerve.kPigeonId);
    mKinematics =
        new SwerveDriveKinematics(
            Constants.Swerve.kFLPos,
            Constants.Swerve.kFRPos,
            Constants.Swerve.kBLPos,
            Constants.Swerve.kBRPos);
    mSetpointGenerator = new SwerveSetpointGenerator(mKinematics);
    mOdometry =
        new SwerveDriveOdometry(mKinematics, new Rotation2d(), getModulePositions(), new Pose2d());

    // Standard Heading Controller (Snap to Angle)
    // Constants should eventually be moved to Constants.java
    mHeadingController = new PIDController(4.0, 0.0, 0.1);
    mHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    mHeadingController.setTolerance(Math.toRadians(0.5));
  }

  @Override
  public synchronized void readPeriodicInputs() {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();
    mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getYaw().getValueAsDouble());

    for (int i = 0; i < 4; i++) {
      mPeriodicIO.module_positions[i] = mModules[i].getPosition();
      mPeriodicIO.module_states[i] = mModules[i].getState();
    }

    mOdometry.update(mPeriodicIO.gyro_heading, mPeriodicIO.module_positions);
    mPeriodicIO.odometry_pose = mOdometry.getPoseMeters();

    // Push to RobotState history for vision latency compensation
    frc.robot.RobotState.getInstance()
        .addFieldToVehicleObservation(mPeriodicIO.timestamp, mPeriodicIO.odometry_pose);
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    for (int i = 0; i < 4; i++) {
      mModules[i].setDesiredState(mPeriodicIO.curr_module_target_states[i], false);
    }
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Drive.this) {
              mVehicleMode = DriveMode.OPEN_LOOP;
              mSetpointGenerator.reset();
              mHeadingController.reset();
            }
          }

          @Override
          public void onLoop(double timestamp) {
            synchronized (Drive.this) {
              handleState();
            }
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  // Pre-allocate to avoid GC in the loop
  private final ChassisSpeeds mOutputSpeeds = new ChassisSpeeds();

  private void handleState() {
    switch (mVehicleMode) {
      case OPEN_LOOP:
        // Pure Driver Control
        mOutputSpeeds.vxMetersPerSecond = mDesiredSpeeds.vxMetersPerSecond;
        mOutputSpeeds.vyMetersPerSecond = mDesiredSpeeds.vyMetersPerSecond;
        mOutputSpeeds.omegaRadiansPerSecond = mDesiredSpeeds.omegaRadiansPerSecond;
        break;

      case HEADING_CONTROL:
        // Driver Control Translation + PID Control Rotation
        double rotationOutput =
            mHeadingController.calculate(
                mPeriodicIO.gyro_heading.getRadians(), mTargetHeading.getRadians());

        mOutputSpeeds.vxMetersPerSecond = mDesiredSpeeds.vxMetersPerSecond;
        mOutputSpeeds.vyMetersPerSecond = mDesiredSpeeds.vyMetersPerSecond;
        mOutputSpeeds.omegaRadiansPerSecond = rotationOutput;
        break;

      case PATH_FOLLOWING:
        // Fully managed by external path follower outputting to mDesiredSpeeds
        mOutputSpeeds.vxMetersPerSecond = mDesiredSpeeds.vxMetersPerSecond;
        mOutputSpeeds.vyMetersPerSecond = mDesiredSpeeds.vyMetersPerSecond;
        mOutputSpeeds.omegaRadiansPerSecond = mDesiredSpeeds.omegaRadiansPerSecond;
        break;

      default:
        mOutputSpeeds.vxMetersPerSecond = 0.0;
        mOutputSpeeds.vyMetersPerSecond = 0.0;
        mOutputSpeeds.omegaRadiansPerSecond = 0.0;
        break;
    }

    // Common Field Relative Transformation (if needed)
    ChassisSpeeds targetSpeeds = mOutputSpeeds;
    if (mIsFieldRelative) {
      // Unfortunately, fromFieldRelativeSpeeds creates a new object by definition
      // We can implement our own static helper to avoid this if we really want 0 allocation
      // But for standard WPILib, this is the one necessary allocation.
      // Or we can perform the rotation math manually here.

      // Manual Field Relative Calculation for Zero Allocs:
      // Robot_x = Field_x * cos(theta) + Field_y * sin(theta)
      // Robot_y = -Field_x * sin(theta) + Field_y * cos(theta)
      // Robot_omega = Field_omega

      double cos = mPeriodicIO.gyro_heading.getCos();
      double sin = mPeriodicIO.gyro_heading.getSin();
      double fieldX = mOutputSpeeds.vxMetersPerSecond;
      double fieldY = mOutputSpeeds.vyMetersPerSecond;

      targetSpeeds =
          new ChassisSpeeds(
              fieldX * cos + fieldY * sin,
              -fieldX * sin + fieldY * cos,
              mOutputSpeeds.omegaRadiansPerSecond);
    }

    mPeriodicIO.curr_module_target_states = mSetpointGenerator.generateSetpoint(targetSpeeds);
  }

  // --- Request Methods (API) ---

  /** Basic Teleop Driving (Open Loop) */
  public synchronized void setOpenLoop(
      Translation2d translation, double rotation, boolean fieldRelative) {
    if (mVehicleMode != DriveMode.OPEN_LOOP) {
      mVehicleMode = DriveMode.OPEN_LOOP;
    }
    mIsFieldRelative = fieldRelative;
    mDesiredSpeeds =
        new ChassisSpeeds(
            translation.getX() * Constants.Swerve.kMaxDriveVelocity,
            translation.getY() * Constants.Swerve.kMaxDriveVelocity,
            rotation * Constants.Swerve.kMaxAngularVelocity);
  }

  /**
   * Heading Control (Snap to Angle) via PID Driver controls translation, Robot controls rotation.
   */
  public synchronized void setHeadingControl(Translation2d translation, Rotation2d targetHeading) {
    if (mVehicleMode != DriveMode.HEADING_CONTROL) {
      mVehicleMode = DriveMode.HEADING_CONTROL;
      mHeadingController.reset();
    }

    // In Heading Control, normally we are always Field Relative
    mIsFieldRelative = true;

    // We only set Translational speeds here, Rotation is calculated in handleState
    mDesiredSpeeds =
        new ChassisSpeeds(
            translation.getX() * Constants.Swerve.kMaxDriveVelocity,
            translation.getY() * Constants.Swerve.kMaxDriveVelocity,
            0.0 // Ignored in HEADING_CONTROL handleState
            );
    mTargetHeading = targetHeading;
  }

  /** Path Following / Autonomous */
  public synchronized void setAutoPathOutput(ChassisSpeeds pathSpeeds) {
    if (mVehicleMode != DriveMode.PATH_FOLLOWING) {
      mVehicleMode = DriveMode.PATH_FOLLOWING;
    }

    mIsFieldRelative = true;
    mDesiredSpeeds = pathSpeeds;
  }

  @Override
  public void checkSystem() {
    // Implement system checks (e.g. iterate modules and check for faults)
  }

  @Override
  public void outputTelemetry() {
    var dashboard = DashboardState.getInstance();
    dashboard.driveMode = mVehicleMode.ordinal(); // 0: Open, 1: Heading, 2: Path
    dashboard.driveHeadingDegrees = mPeriodicIO.gyro_heading.getDegrees();
    dashboard.robotPose = mPeriodicIO.odometry_pose;
  }

  @Override
  public void stop() {
    mDesiredSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  @Override
  public void zeroSensors() {
    mPigeon.setYaw(0);
    mOdometry.resetPosition(mPeriodicIO.gyro_heading, getModulePositions(), new Pose2d());
    readPeriodicInputs();
  }

  // Helper for initialization only
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] pos = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) pos[i] = mModules[i].getPosition();
    return pos;
  }

  public synchronized Rotation2d getHeading() {
    return mPeriodicIO.gyro_heading;
  }

  public synchronized ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        mKinematics.toChassisSpeeds(mPeriodicIO.module_states), mPeriodicIO.gyro_heading);
  }

  public synchronized Pose2d getPose() {
    return mPeriodicIO.odometry_pose;
  }
}
