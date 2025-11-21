package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

public final class SwerveController {
  private final SwerveConfig config;
  private final Translation2d[] modulePositions;
  private final SwerveModule[] swerveModules;

  // Libraries
  private final AccelLimiter accelLimiter;
  private final ArcOdometry arcOdometry = new ArcOdometry();
  private final FomPoseTracker poseTracker = new FomPoseTracker();
  private final SkidDetector wheelSkidDetector = new SkidDetector(1.5);

  // State
  private ChassisSpeeds currentChassisCommand = new ChassisSpeeds();
  private ChassisSpeeds previousChassisCommand = new ChassisSpeeds();

  public SwerveController(final SwerveConfig config, final SwerveModule[] modules) {
    this.config = config;
    this.swerveModules = modules;
    this.modulePositions = config.moduleTranslations();
    this.accelLimiter = new AccelLimiter(config);
  }

  public void reset(final Pose2d startPose) {
    poseTracker.reset(startPose);
    accelLimiter.reset();
    for (SwerveModule m : swerveModules) {
      m.lastSteerRad = SwerveUtil.wrap(m.io.getSteerAngleRad());
      m.lastDriveMps = m.io.getDriveVelocityMps();
      m.ignoreForOdom = false;
    }
  }

  // =================================================================================
  //  Odometry Update (Prediction)
  // =================================================================================

  /**
   * Updates the robot pose based purely on wheel encoders and gyro. This is the "Prediction" step
   * of the filter.
   *
   * @param yaw The current gyro angle (Robot Heading).
   * @param yawRate The current gyro angular velocity (rad/s).
   * @return The predicted Pose2d.
   */
  public Pose2d updateOdometry(Rotation2d yaw, double yawRate) {
    // Calculate delta in Robot Frame
    Translation2d robotDelta = arcOdometry.updateRobotFrameDelta(swerveModules);

    // Rotate to Field Frame (using Gyro Angle)
    Translation2d fieldDelta = SwerveUtil.rotate(robotDelta, yaw.getRadians());

    // Calculate delta Yaw (using Gyro Rate for better temporal resolution)
    double deltaYaw = yawRate * config.dt;

    // Push to Tracker
    poseTracker.predict(fieldDelta, deltaYaw);

    return poseTracker.get();
  }

  // =================================================================================
  //  Vision Update (Correction)
  // =================================================================================

  /**
   * fuses a vision measurement into the current pose estimate. This is the "Correction" step of the
   * filter.
   *
   * @param visionPose The pose observed by the camera.
   * @param visionFom The uncertainty (Standard Deviation) of the vision data.
   * @return The corrected Pose2d.
   */
  public Pose2d updateVision(Pose2d visionPose, double visionFom) {
    poseTracker.correct(visionPose, visionFom);
    return poseTracker.get();
  }

  // =================================================================================
  //  3. Fused Update (Orchestration)
  // =================================================================================

  /**
   * The main entry point for pose estimation. Calls Odometry update and conditionally calls Vision
   * update.
   *
   * @param yaw Current Gyro Yaw.
   * @param yawRate Current Gyro Rate.
   * @param visionPose Optional vision measurement.
   * @param visionFom Optional vision uncertainty.
   * @return The final estimated Pose2d.
   */
  public Pose2d updatePoseEstimation(
      Rotation2d yaw, double yawRate, Optional<Pose2d> visionPose, Optional<Double> visionFom) {

    // Always run prediction first
    Pose2d predictedPose = updateOdometry(yaw, yawRate);

    // Guard Clause: If no vision, return prediction immediately
    if (visionPose.isEmpty()) {
      return predictedPose;
    }

    // Guard Clause: Reject vision if spinning too fast (Motion Blur / Latency issues)
    if (Math.abs(yawRate) > 2.5) { // Threshold: 2.5 rad/s
      return predictedPose;
    }

    // Apply Correction
    // Default FoM to 0.9 if not provided (Conservative fallback)
    return updateVision(visionPose.get(), visionFom.orElse(0.9));
  }

  // =================================================================================
  //  Command Execution (Control Logic)
  // =================================================================================

  public void executeFieldRelativeCommand(
      final ChassisSpeeds fieldCommand, final Rotation2d robotHeading) {
    // Skid Detection & Odometry Trust Adjustment
    handleSkidDetection();

    // Convert to Body Frame
    Translation2d linearField =
        new Translation2d(fieldCommand.vxMetersPerSecond, fieldCommand.vyMetersPerSecond);
    Translation2d linearBody = SwerveUtil.rotate(linearField, -robotHeading.getRadians());

    // Acceleration Limiting (Rate Limiter)
    ChassisSpeeds targetBody =
        new ChassisSpeeds(linearBody.getX(), linearBody.getY(), fieldCommand.omegaRadiansPerSecond);
    currentChassisCommand = accelLimiter.limit(targetBody);

    // Inverse Kinematics -> Optimization -> Output
    ModuleState[] desiredStates =
        SwerveKinematics.inverseKinematics(config, modulePositions, currentChassisCommand);

    for (int i = 0; i < swerveModules.length; i++) {
      applyModuleState(i, desiredStates[i]);
    }

    previousChassisCommand = currentChassisCommand;
  }

  private void handleSkidDetection() {
    double[] speeds = new double[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++)
      speeds[i] = Math.abs(swerveModules[i].io.getDriveVelocityMps());

    boolean[] skidding =
        wheelSkidDetector.detectSkid(modulePositions, previousChassisCommand, speeds);
    boolean anySkid = false;

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].ignoreForOdom = skidding[i];
      anySkid |= skidding[i];
    }

    // If skidding, distrust odometry (Increase Uncertainty)
    if (anySkid) poseTracker.setUncertainty(0.1);
    else poseTracker.setUncertainty(0.02);
  }

  private void applyModuleState(int index, ModuleState desiredState) {
    SwerveModule mod = swerveModules[index];

    // Optimization: +- 180 degree logic
    ModuleState optimized =
        SwerveKinematics.optimize(desiredState, new Rotation2d(mod.io.getSteerAngleRad()));

    // Set Angle
    mod.io.setSteerTargetAngleRad(optimized.angle.getRadians());

    // Calculate Feedforward + Feedback
    double currentSpeed = mod.io.getDriveVelocityMps();
    double accel = (optimized.speedMps - mod.lastDriveMps) / config.dt;

    double ff =
        (SwerveUtil.sign(optimized.speedMps) * config.kS)
            + (config.kV * optimized.speedMps)
            + (config.kA * accel);

    double pid = config.kPSpeed * (optimized.speedMps - currentSpeed);

    mod.io.setDriveVoltage(ff + pid);
    mod.lastDriveMps = optimized.speedMps;
  }

  public Pose2d getEstimatedRobotPose() {
    return poseTracker.get();
  }

  public void stopAllModules() {
    for (SwerveModule m : swerveModules) m.io.setDriveVoltage(0.0);
    currentChassisCommand = new ChassisSpeeds();
  }

  public void engageXLock() {
    double[] angles = {Math.PI / 4, -Math.PI / 4, -Math.PI / 4, Math.PI / 4};
    for (int i = 0; i < 4; i++) {
      swerveModules[i].io.setSteerTargetAngleRad(angles[i]);
      swerveModules[i].io.setDriveVoltage(0.0);
    }
    currentChassisCommand = new ChassisSpeeds();
  }
}
