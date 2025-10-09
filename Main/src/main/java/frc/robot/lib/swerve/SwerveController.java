package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * High-level controller that converts field-relative chassis commands into individual wheel
 * commands, limits acceleration, detects skid, and maintains pose estimation.
 */
public final class SwerveController {
  private final SwerveConfig config;
  private final Translation2d[] modulePositions;
  private final SwerveModule[] swerveModules;
  private final AccelLimiter accelLimiter;
  private final ArcOdometry arcOdometry = new ArcOdometry();
  private final FomPoseTracker poseTracker = new FomPoseTracker();
  private final SkidDetector wheelSkidDetector = new SkidDetector(1.6);

  /** the most recent chassis command in robot frame */
  private ChassisSpeeds currentChassisCommand = new ChassisSpeeds();

  /** the previous chassis command used for skid detection */
  private ChassisSpeeds previousChassisCommand = new ChassisSpeeds();

  public SwerveController(final SwerveConfig config, final SwerveModule[] modules) {
    this.config = config;
    this.swerveModules = modules;
    this.modulePositions = config.moduleTranslations();
    this.accelLimiter = new AccelLimiter(config);
  }

  /**
   * Resets internal state to a known starting pose. Must be called at the start of a match.
   *
   * @param startPose initial robot pose in the field frame
   */
  public void reset(final Pose2d startPose) {
    poseTracker.reset(startPose);
    accelLimiter.reset();
    for (SwerveModule module : swerveModules) {
      module.lastSteerRad = SwerveUtil.wrap(module.io.getSteerAngleRad());
      module.lastDriveMps = module.io.getDriveVelocityMps();
      module.ignoreForOdom = false;
    }
  }

  /**
   * Incorporates sensor measurements into the pose tracker. Pass in the yaw in radians and
   * optionally a vision pose and its figure of merit (FoM).
   *
   * @param imuYawRad current yaw from the IMU in radians
   * @param imuYawRateRadPerS yaw rate from the IMU in radians per second
   * @param visionPose optional vision-estimated pose in field coordinates
   * @param visionFom optional figure of merit for the vision measurement (lower = better)
   */
  public void updatePoseEstimation(
      final double imuYawRad,
      final double imuYawRateRadPerS,
      final java.util.Optional<Pose2d> visionPose,
      final java.util.Optional<Double> visionFom,
      final double avgTagAreaPercentage) {
    Translation2d robotDelta = arcOdometry.updateRobotFrameDelta(swerveModules);
    Translation2d fieldDelta = SwerveUtil.rotate(robotDelta, imuYawRad);
    double deltaYawRad = imuYawRateRadPerS * config.dt;

    poseTracker.predictPose(fieldDelta, deltaYawRad);
    if (visionPose.isPresent()) {
      if (Math.abs(imuYawRateRadPerS) > Math.PI) {
        double fom = visionFom.orElse(0.15);
        poseTracker.correctPoseWithVision(visionPose.get(), fom);
      }
      if (avgTagAreaPercentage > 0.80) {
        double fom = visionFom.orElse(0.00001);
        poseTracker.correctPoseWithVision(visionPose.get(), fom);
      } else if (avgTagAreaPercentage > 0.60) {
        double fom = visionFom.orElse(0.005);
        poseTracker.correctPoseWithVision(visionPose.get(), fom);
      } else if (avgTagAreaPercentage > 0.40) {
        double fom = visionFom.orElse(0.01);
        poseTracker.correctPoseWithVision(visionPose.get(), fom);
      } else if (avgTagAreaPercentage > 0.22) {
        double fom = visionFom.orElse(0.02);
        poseTracker.correctPoseWithVision(visionPose.get(), fom);
      } else if (avgTagAreaPercentage < 0.22) {
        double fom = visionFom.orElse(0.035);
        poseTracker.correctPoseWithVision(visionPose.get(), fom);
      }
    }
  }

  /**
   * Executes a field-relative chassis speed command. Handles skid detection, acceleration limiting,
   * wheel state optimization, and drive voltage calculation.
   *
   * @param fieldCommand desired chassis speeds in the field frame
   * @param robotHeading current robot heading
   */
  public void executeFieldRelativeCommand(
      final ChassisSpeeds fieldCommand, final Rotation2d robotHeading) {

    poseTracker.setOdometryFom(0.02);

    // Collect measured wheel speeds for skid detection
    double[] measuredWheelSpeeds = new double[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) {
      measuredWheelSpeeds[i] = Math.abs(swerveModules[i].io.getDriveVelocityMps());
    }

    // Detect skidding wheels and mark them to be ignored by odometry
    boolean[] skiddingWheel =
        wheelSkidDetector.detectSkid(modulePositions, previousChassisCommand, measuredWheelSpeeds);
    boolean anySkid = false;
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].ignoreForOdom = skiddingWheel[i];
      anySkid |= skiddingWheel[i];
    }
    if (anySkid) {
      poseTracker.setOdometryFom(
          Math.hypot(
                  previousChassisCommand.vxMetersPerSecond,
                  previousChassisCommand.vyMetersPerSecond)
              / 2);
    }

    // Convert field-relative command into robot frame
    Translation2d bodyVelocity =
        SwerveUtil.rotate(
            new Translation2d(fieldCommand.vxMetersPerSecond, fieldCommand.vyMetersPerSecond),
            -robotHeading.getRadians());

    // Limit acceleration before computing module commands
    currentChassisCommand =
        accelLimiter.limit(
            new ChassisSpeeds(
                bodyVelocity.getX(), bodyVelocity.getY(), fieldCommand.omegaRadiansPerSecond));

    // Compute desired module states using kinematics
    ModuleState[] desiredModuleStates =
        SwerveKinematics.inverseKinematics(config, modulePositions, currentChassisCommand);

    // Send optimized state to each module with feedforward and feedback control
    for (int i = 0; i < swerveModules.length; i++) {
      SwerveModule module = swerveModules[i];

      ModuleState optimizedState =
          SwerveKinematics.optimize(
              desiredModuleStates[i], new Rotation2d(module.io.getSteerAngleRad()));
      module.io.setSteerTargetAngleRad(optimizedState.angle.getRadians());

      double currentWheelSpeed = module.io.getDriveVelocityMps();
      double estimatedAccel = (optimizedState.speedMps - module.lastDriveMps) / config.dt;

      double feedforwardVoltage =
          SwerveUtil.sign(optimizedState.speedMps) * config.kS
              + config.kV * optimizedState.speedMps
              + config.kA * estimatedAccel;

      double feedbackVoltage = config.kPSpeed * (optimizedState.speedMps - currentWheelSpeed);

      module.io.setDriveVoltage(feedforwardVoltage + feedbackVoltage);
      module.lastDriveMps = currentWheelSpeed;
    }

    previousChassisCommand = currentChassisCommand;
  }

  /** Returns the most recent body frame command used for the modules. */
  public ChassisSpeeds getCurrentBodyCommand() {
    return currentChassisCommand;
  }

  /** Returns the robot pose estimated by odometry and vision fusion. */
  public Pose2d getEstimatedRobotPose() {
    return poseTracker.estimatedPose;
  }

  /**
   * Actively hold the robot in an X-lock configuration by commanding each wheel to ±45° and zero
   * drive voltage. This increases static friction to resist external forces.
   */
  public void engageXLock() {
    final double[] xLockAngles = {Math.PI / 4.0, -Math.PI / 4.0, -Math.PI / 4.0, Math.PI / 4.0};
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].io.setSteerTargetAngleRad(xLockAngles[i]);
      swerveModules[i].io.setDriveVoltage(0.0);
    }
    currentChassisCommand = new ChassisSpeeds();
  }

  /** Immediately stops driving all modules by zeroing drive voltage. */
  public void stopAllModules() {
    for (SwerveModule module : swerveModules) {
      module.io.setDriveVoltage(0.0);
    }
    currentChassisCommand = new ChassisSpeeds();
  }
}
