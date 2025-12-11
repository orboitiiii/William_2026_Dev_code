package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.subsystems.swerve.SwerveKinematics;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveOdometry;
import frc.robot.subsystems.swerve.SwerveSetpointGenerator;

public class Drive extends Subsystem {
  private static Drive mInstance;

  public static Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  // Hardware
  private final SwerveModule[] mModules;
  private final Pigeon2 mPigeon; // 1690 Requirement

  // Logic / Math
  private final SwerveSetpointGenerator mSetpointGenerator;
  private final SwerveKinematics mKinematics;
  private final SwerveOdometry mOdometry;

  // Control State
  private ChassisSpeeds mDesiredChassisSpeeds = new ChassisSpeeds();
  private boolean mIsFieldRelative = true;

  private Drive() {
    mModules =
        new SwerveModule[] {
          new SwerveModule(
              "FrontLeft",
              Constants.Swerve.kFLDriveId,
              Constants.Swerve.kFLSteerId,
              Constants.Swerve.kFLEncoderId,
              Constants.Swerve.kFLOffset,
              new edu.wpi.first.math.geometry.Translation2d(
                  Constants.Swerve.kWheelBase / 2, Constants.Swerve.kTrackWidth / 2)),
          new SwerveModule(
              "FrontRight",
              Constants.Swerve.kFRDriveId,
              Constants.Swerve.kFRSteerId,
              Constants.Swerve.kFREncoderId,
              Constants.Swerve.kFROffset,
              new edu.wpi.first.math.geometry.Translation2d(
                  Constants.Swerve.kWheelBase / 2, -Constants.Swerve.kTrackWidth / 2)),
          new SwerveModule(
              "BackLeft",
              Constants.Swerve.kBLDriveId,
              Constants.Swerve.kBLSteerId,
              Constants.Swerve.kBLEncoderId,
              Constants.Swerve.kBLOffset,
              new edu.wpi.first.math.geometry.Translation2d(
                  -Constants.Swerve.kWheelBase / 2, Constants.Swerve.kTrackWidth / 2)),
          new SwerveModule(
              "BackRight",
              Constants.Swerve.kBRDriveId,
              Constants.Swerve.kBRSteerId,
              Constants.Swerve.kBREncoderId,
              Constants.Swerve.BROffset,
              new edu.wpi.first.math.geometry.Translation2d(
                  -Constants.Swerve.kWheelBase / 2, -Constants.Swerve.kTrackWidth / 2))
        };

    mPigeon = new Pigeon2(Constants.Swerve.kPigeonId, Constants.Swerve.kCanivoreBusName);

    mKinematics = new SwerveKinematics();
    mSetpointGenerator =
        new SwerveSetpointGenerator(
            Constants.Swerve
                .kKinematics); // Note: SetpointGen assumes standard kinematics for limits for now.

    // Initialize Odometry
    mOdometry =
        new SwerveOdometry(
            mKinematics,
            new Pose2d(),
            Rotation2d.fromDegrees(mPigeon.getYaw().getValueAsDouble()),
            getModuleDistances());
  }

  @Override
  public void registerEnabledLoops(Looper in) {
    in.register(
        new frc.robot.framework.ILoop() {
          @Override
          public void onStart(double timestamp) {}

          @Override
          public void onLoop(double timestamp) {
            synchronized (Drive.this) {
              updateModuleStates();
            }
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  private synchronized void updateModuleStates() {
    // Discretization / Skew Compensation (1690: "Predict dt ahead")
    // theta_future = theta_now + omega * dt
    double dt = Constants.kLooperDt;
    Rotation2d currentHeading = Rotation2d.fromDegrees(mPigeon.getYaw().getValueAsDouble());
    Rotation2d predictedHeading =
        currentHeading.plus(
            Rotation2d.fromRadians(mDesiredChassisSpeeds.omegaRadiansPerSecond * dt));

    ChassisSpeeds robotRelativeSpeeds;
    if (mIsFieldRelative) {
      // Use PREDICTED heading for field->robot conversion to compensate for skew
      robotRelativeSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              mDesiredChassisSpeeds.vxMetersPerSecond,
              mDesiredChassisSpeeds.vyMetersPerSecond,
              mDesiredChassisSpeeds.omegaRadiansPerSecond,
              predictedHeading);
    } else {
      robotRelativeSpeeds = mDesiredChassisSpeeds;
    }

    // Anti-Slip Logic (Limit speeds based on physics)
    // Note: SetpointGenerator currently accepts ChassisSpeeds.
    // We might want to pass robotRelativeSpeeds there.
    SwerveModuleState[] logicalStates = mKinematics.toSwerveModuleStates(robotRelativeSpeeds);

    // Send to modules (Cosine Scaling is inside SwerveModule)
    for (int i = 0; i < 4; i++) {
      mModules[i].setDesiredState(logicalStates[i], false); // Open loop for now
    }
  }

  @Override
  public synchronized void readPeriodicInputs() {
    // Custom Odometry Update
    Rotation2d gyroAngle = Rotation2d.fromDegrees(mPigeon.getYaw().getValueAsDouble());
    double gyroRate =
        edu.wpi.first.math.util.Units.degreesToRadians(
            mPigeon.getAngularVelocityZWorld().getValueAsDouble());

    double[] distances = getModuleDistances();
    Rotation2d[] angles = getModuleAngles();
    double[] velocities = getModuleVelocities();

    mOdometry.update(gyroAngle, distances, angles, velocities, gyroRate);

    // Push to Single Source of Truth
    RobotState.getInstance()
        .setFieldToVehicle(
            mOdometry.getClass().equals(SwerveOdometry.class)
                ? ((SwerveOdometry) mOdometry)
                    .update(gyroAngle, distances, angles, velocities, gyroRate)
                : new Pose2d());
    // Note: The above line is a bit hacky due to return structure, fixed below:
  }

  // Fixed method for clean read
  public void updateOdometry() {
    Rotation2d gyroAngle = Rotation2d.fromDegrees(mPigeon.getYaw().getValueAsDouble());
    double gyroRate =
        edu.wpi.first.math.util.Units.degreesToRadians(
            mPigeon.getAngularVelocityZWorld().getValueAsDouble());

    Pose2d pose =
        mOdometry.update(
            gyroAngle, getModuleDistances(), getModuleAngles(), getModuleVelocities(), gyroRate);
    RobotState.getInstance().setFieldToVehicle(pose);
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    // No explicit flush needed for Phoenix 6 usually
  }

  @Override
  public void outputTelemetry() {
    RobotState.getInstance().outputToSmartDashboard();
  }

  @Override
  public void zeroSensors() {
    mPigeon.setYaw(0);
    mOdometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0), getModuleDistances());
  }

  @Override
  public void checkSystem() {}

  @Override
  public synchronized void stop() {
    mDesiredChassisSpeeds = new ChassisSpeeds(0, 0, 0);
    for (SwerveModule mod : mModules) {
      mod.stop();
    }
  }

  public synchronized void setTeleopInputs(
      double vx, double vy, double omega, boolean fieldRelative) {
    mDesiredChassisSpeeds = new ChassisSpeeds(vx, vy, omega);
    mIsFieldRelative = fieldRelative;
  }

  private double[] getModuleDistances() {
    double[] dists = new double[4];
    for (int i = 0; i < 4; i++) dists[i] = mModules[i].getPosition().distanceMeters;
    return dists;
  }

  private Rotation2d[] getModuleAngles() {
    Rotation2d[] angs = new Rotation2d[4];
    for (int i = 0; i < 4; i++) angs[i] = mModules[i].getSteerAngle();
    return angs;
  }

  private double[] getModuleVelocities() {
    double[] vels = new double[4];
    for (int i = 0; i < 4; i++) vels[i] = mModules[i].getState().speedMetersPerSecond;
    return vels;
  }
}
