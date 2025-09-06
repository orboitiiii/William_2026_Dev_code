package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class SwerveController {
  private final SwerveConfig config;
  private final Translation2d[] moduleOffsets;
  private final SwerveModule[] modules;
  private final AccelLimiter accelLimiter;
  private final ArcOdometry arcOdometry = new ArcOdometry();

  public final FomPoseTracker poseTracker = new FomPoseTracker();
  public final SkidDetector skid = new SkidDetector(1.6);

  private ChassisSpeeds bodyCommand = new ChassisSpeeds();
  private ChassisSpeeds prevBodyCommand = new ChassisSpeeds();

  public SwerveController(SwerveConfig cfg, SwerveModule[] modules) {
    this.config = cfg;
    this.modules = modules;
    this.moduleOffsets = cfg.moduleTranslations();
    this.accelLimiter = new AccelLimiter(cfg);
  }

  public void reset(Pose2d startPose) {
    poseTracker.reset(startPose);
    accelLimiter.reset();
    for (SwerveModule m : modules) {
      m.lastSteerRad = SwerveUtil.wrap(m.io.getSteerAngleRad());
      m.lastDriveMps = m.io.getDriveVelocityMps();
      m.ignoreForOdom = false;
    }
  }

  public void updateEstimation(
      double imuYawRad,
      double imuYawRateRadPerS,
      java.util.Optional<Pose2d> visionPose,
      java.util.Optional<Double> visionFom) {

    Translation2d robotDelta = arcOdometry.updateRobotFrameDelta(modules);
    Translation2d fieldDelta = SwerveUtil.rotate(robotDelta, imuYawRad);
    double deltaYawRad = imuYawRateRadPerS * config.dt;

    poseTracker.predict(fieldDelta, deltaYawRad);
    visionPose.ifPresent(p -> poseTracker.correctWithVision(p, visionFom.orElse(0.01)));
  }

  public void executeFieldCommand(ChassisSpeeds fieldCommand, Rotation2d robotHeading) {
    double[] measuredWheelMps = new double[4];
    for (int i = 0; i < 4; i++) {
      measuredWheelMps[i] = Math.abs(modules[i].io.getDriveVelocityMps());
    }

    boolean[] skiddingWheel = skid.detect(moduleOffsets, prevBodyCommand, measuredWheelMps);
    boolean anySkid = false;
    for (int i = 0; i < 4; i++) {
      modules[i].ignoreForOdom = skiddingWheel[i];
      anySkid |= skiddingWheel[i];
    }
    if (anySkid) {
      poseTracker.setOdomFom(1.0);
    }

    Translation2d bodyVel =
        SwerveUtil.rotate(
            new Translation2d(fieldCommand.vxMetersPerSecond, fieldCommand.vyMetersPerSecond),
            -robotHeading.getRadians());

    bodyCommand =
        accelLimiter.limit(
            new ChassisSpeeds(bodyVel.getX(), bodyVel.getY(), fieldCommand.omegaRadiansPerSecond));

    ModuleState[] desiredStates =
        SwerveKinematics.inverseKinematics(config, moduleOffsets, bodyCommand);

    for (int i = 0; i < 4; i++) {
      SwerveModule mod = modules[i];

      ModuleState optimizedState =
          SwerveKinematics.optimize(desiredStates[i], new Rotation2d(mod.io.getSteerAngleRad()));
      mod.io.setSteerTargetAngleRad(optimizedState.angle.getRadians());

      double currentWheelMps = mod.io.getDriveVelocityMps();
      double estimatedAccelMps2 = (optimizedState.speedMps - mod.lastDriveMps) / config.dt;

      double feedforwardVolts =
          SwerveUtil.sign(optimizedState.speedMps) * config.kS
              + config.kV * optimizedState.speedMps
              + config.kA * estimatedAccelMps2;

      double feedbackVolts = config.kPSpeed * (optimizedState.speedMps - currentWheelMps);

      mod.io.setDriveVoltage(feedforwardVolts + feedbackVolts);
      mod.lastDriveMps = currentWheelMps;
    }

    prevBodyCommand = bodyCommand;
  }

  public ChassisSpeeds getBodyCommand() {
    return bodyCommand;
  }

  public Pose2d getRobotPose() {
    return poseTracker.pose;
  }

  public void holdXLock() {
    double[] xLockAnglesRad = {Math.PI / 4.0, -Math.PI / 4.0, -Math.PI / 4.0, Math.PI / 4.0};
    for (int i = 0; i < modules.length; i++) {
      modules[i].io.setSteerTargetAngleRad(xLockAnglesRad[i]);
      modules[i].io.setDriveVoltage(0.0);
    }
    bodyCommand = new ChassisSpeeds();
  }

  public void stop() {
    for (SwerveModule m : modules) {
      m.io.setDriveVoltage(0.0);
    }
    bodyCommand = new ChassisSpeeds();
  }
}
