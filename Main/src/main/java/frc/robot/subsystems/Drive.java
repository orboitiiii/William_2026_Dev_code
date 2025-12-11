package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  public enum DriveMode {
    MANUAL,
    AUTO
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

  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private DriveMode mVehicleMode = DriveMode.MANUAL;

  private ChassisSpeeds mDesiredSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
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

    mPigeon = new Pigeon2(Constants.Swerve.kPigeonId, Constants.Swerve.kCanivoreBusName);
    mKinematics =
        new SwerveDriveKinematics(
            Constants.Swerve.kFLPos,
            Constants.Swerve.kFRPos,
            Constants.Swerve.kBLPos,
            Constants.Swerve.kBRPos);
    mSetpointGenerator = new SwerveSetpointGenerator(mKinematics);
    mOdometry =
        new SwerveDriveOdometry(mKinematics, new Rotation2d(), getModulePositions(), new Pose2d());
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
              mVehicleMode = DriveMode.MANUAL;
              mSetpointGenerator.reset();
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

  private void handleState() {
    switch (mVehicleMode) {
      case MANUAL:
        updateManual();
        break;
      case AUTO:
        updateAuto();
        break;
      default:
        break;
    }
  }

  private void updateManual() {
    ChassisSpeeds robotSpeeds = mDesiredSpeeds;
    if (mIsFieldRelative) {
      robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(mDesiredSpeeds, mPeriodicIO.gyro_heading);
    }

    mPeriodicIO.curr_module_target_states = mSetpointGenerator.generateSetpoint(robotSpeeds);
  }

  private void updateAuto() {
    ChassisSpeeds robotSpeeds = mDesiredSpeeds;
    if (mIsFieldRelative) {
      robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(mDesiredSpeeds, mPeriodicIO.gyro_heading);
    }
    // Placeholder for path following logic
    mPeriodicIO.curr_module_target_states = mSetpointGenerator.generateSetpoint(robotSpeeds);
  }

  @Override
  public void checkSystem() {
    // Implement system checks (e.g. iterate modules and check for faults)
  }

  @Override
  public void outputTelemetry() {
    var dashboard = DashboardState.getInstance();
    dashboard.driveMode = (mVehicleMode == DriveMode.MANUAL) ? 0 : 1;
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

  public synchronized void setDesiredSpeeds(ChassisSpeeds desireSpeeds, boolean fieldRelative) {
    mDesiredSpeeds = desireSpeeds;
    mIsFieldRelative = fieldRelative;
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
}
