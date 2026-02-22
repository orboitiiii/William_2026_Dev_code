package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.PoseHistory;
import frc.robot.VisionConstants;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Vision Subsystem - Reference FRC 254 2025 implementation.
 *
 * <p>Handles AprilTag detection and provides robot pose estimation. Supports multiple cameras and
 * various pose estimation algorithms.
 *
 * <p>Core features:
 *
 * <ul>
 *   <li>Triple camera MegaTag pose estimation processing (Front, Left, Turret-Right)
 *   <li>Inverse-variance weighting fusion of multi-camera estimates
 *   <li>Single-tag gyro fusion for improved accuracy (fuseWithGyro)
 *   <li>Dynamic standard deviation adjustment
 *   <li>Vision measurement rejection logic
 *   <li>Dynamic Inverse Kinematics for Turret-Mounted Cameras
 * </ul>
 */
public class VisionSubsystem extends Subsystem {
  private static VisionSubsystem mInstance;

  public static VisionSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new VisionSubsystem(new VisionIOLimelight());
    }
    return mInstance;
  }

  private final VisionIO io;
  private final PoseHistory state;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

  private boolean useVision = true;
  private int acceptedEstimateCount = 0;

  private double lastGyroResetTime = 0.0;

  // Task 1: Create Turret History Trajectory Buffer
  // Solves Temporal Desync Fallacy caused by vision processing delays
  private final TimeInterpolatableBuffer<Rotation2d> mTurretAngleHistory =
      TimeInterpolatableBuffer.createBuffer(1.5);

  /**
   * Creates a new vision subsystem.
   *
   * @param io Vision IO implementation.
   * @param state PoseHistory for pose history and angular velocity queries.
   */
  public VisionSubsystem(VisionIO io, PoseHistory state) {
    this.io = io;
    this.state = state;
  }

  /** Creates a vision subsystem with default PoseHistory. */
  public VisionSubsystem(VisionIO io) {
    this(io, PoseHistory.getInstance());
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            mTurretAngleHistory.clear();
          }

          @Override
          public void onLoop(double timestamp) {
            // Vision processing is handled in readPeriodicInputs/writePeriodicOutputs
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  @Override
  public void readPeriodicInputs() {
    io.readInputs(inputs);

    // Record the turret angle at the current timestamp, used later for delaying
    // synchronization
    double currentTime = Timer.getFPGATimestamp();
    mTurretAngleHistory.addSample(currentTime, Turret.getInstance().getAngle());
  }

  @Override
  public void writePeriodicOutputs() {
    logCameraInputs("Vision/CameraFront", inputs.cameraFront, inputs.cameraFrontConnected);
    logCameraInputs("Vision/CameraLeft", inputs.cameraLeft, inputs.cameraLeftConnected);
    logCameraInputs("Vision/CameraRight", inputs.cameraRight, inputs.cameraRightConnected);

    updatePipelineAndCrop(
        VisionConstants.kFrontLimelightName,
        inputs.cameraFront.seesTarget,
        inputs.cameraFront.txDeg,
        inputs.cameraFront.tyDeg,
        inputs.cameraFront.latencyPipelineMs);
    updatePipelineAndCrop(
        VisionConstants.kLeftLimelightName,
        inputs.cameraLeft.seesTarget,
        inputs.cameraLeft.txDeg,
        inputs.cameraLeft.tyDeg,
        inputs.cameraLeft.latencyPipelineMs);
    updatePipelineAndCrop(
        VisionConstants.kRightLimelightName,
        inputs.cameraRight.seesTarget,
        inputs.cameraRight.txDeg,
        inputs.cameraRight.tyDeg,
        inputs.cameraRight.latencyPipelineMs);

    if (!useVision || frc.robot.DashboardState.getInstance().isApriltagDisabled()) {
      return;
    }

    // List to collect all valid estimates from the current cycle
    List<VisionFieldPoseEstimate> validEstimates = new ArrayList<>();

    // Process Static Camera: Front (Converts Transform2d to Transform3d)
    Transform3d frontTransform3d =
        new Transform3d(
            new Translation3d(
                VisionConstants.kRobotToFrontCamera.getX(),
                VisionConstants.kRobotToFrontCamera.getY(),
                0.0),
            new Rotation3d(
                0.0, 0.0, VisionConstants.kRobotToFrontCamera.getRotation().getRadians()));

    processStaticCamera(inputs.cameraFront, "CameraFront", frontTransform3d)
        .ifPresent(validEstimates::add);

    // Process Static Camera: Left
    Transform3d leftTransform3d =
        new Transform3d(
            new Translation3d(
                VisionConstants.kRobotToLeftCamera.getX(),
                VisionConstants.kRobotToLeftCamera.getY(),
                0.0),
            new Rotation3d(
                0.0, 0.0, VisionConstants.kRobotToLeftCamera.getRotation().getRadians()));

    processStaticCamera(inputs.cameraLeft, "CameraLeft", leftTransform3d)
        .ifPresent(validEstimates::add);

    // Process Dynamic Camera: Right (Turret mounted)
    processDynamicCamera(
            inputs.cameraRight, "CameraRight", VisionConstants.kTurretCenterToRightCamera)
        .ifPresent(validEstimates::add);

    // Fuse or select estimate
    Optional<VisionFieldPoseEstimate> accepted = Optional.empty();

    if (validEstimates.size() == 1) {
      accepted = Optional.of(validEstimates.get(0));
    } else if (validEstimates.size() > 1) {
      accepted = Optional.of(fuseMultipleEstimates(validEstimates));
    }

    // Send accepted estimate to RobotState
    accepted.ifPresent(
        est -> {
          state.updateMegatagEstimate(est);
          acceptedEstimateCount++;
          frc.robot.DashboardState.getInstance().lastVisionTimestamp = est.getTimestampSeconds();
        });

    frc.robot.DashboardState.getInstance().frontLLOK = inputs.cameraFrontConnected;
  }

  private void updatePipelineAndCrop(
      String cameraName, boolean seesTarget, double txDeg, double tyDeg, double latencyMs) {
    if (!seesTarget) {
      io.setPipelineIndex(cameraName, VisionConstants.kPipelineSearch);
      io.setCropWindow(cameraName, -1.0, 1.0, -1.0, 1.0);
      return;
    }
    io.setPipelineIndex(cameraName, VisionConstants.kPipelineTrack);
    double half =
        VisionConstants.kCropHalfSizeNorm + latencyMs * VisionConstants.kCropMarginPerMsLatency;
    double xNorm = (txDeg + VisionConstants.kFovHorizDeg / 2.0) / VisionConstants.kFovHorizDeg;
    double yNorm = (tyDeg + VisionConstants.kFovVertDeg / 2.0) / VisionConstants.kFovVertDeg;
    double xMin = MathUtil.clamp((xNorm - half) * 2.0 - 1.0, -1.0, 1.0);
    double xMax = MathUtil.clamp((xNorm + half) * 2.0 - 1.0, -1.0, 1.0);
    double yMin = MathUtil.clamp((yNorm - half) * 2.0 - 1.0, -1.0, 1.0);
    double yMax = MathUtil.clamp((yNorm + half) * 2.0 - 1.0, -1.0, 1.0);
    io.setCropWindow(cameraName, xMin, xMax, yMin, yMax);
  }

  /** Fuses multiple vision pose estimates iteratively using inverse-variance weighting. */
  private VisionFieldPoseEstimate fuseMultipleEstimates(List<VisionFieldPoseEstimate> estimates) {
    if (estimates.size() == 0) return null;
    if (estimates.size() == 1) return estimates.get(0);

    VisionFieldPoseEstimate fused = estimates.get(0);
    for (int i = 1; i < estimates.size(); i++) {
      fused = fuseEstimates(fused, estimates.get(i));
    }
    return fused;
  }

  private VisionFieldPoseEstimate fuseEstimates(
      VisionFieldPoseEstimate a, VisionFieldPoseEstimate b) {
    if (b.getTimestampSeconds() < a.getTimestampSeconds()) {
      var tmp = a;
      a = b;
      b = tmp;
    }

    Optional<Pose2d> poseAtA = state.getFieldToRobot(a.getTimestampSeconds());
    Optional<Pose2d> poseAtB = state.getFieldToRobot(b.getTimestampSeconds());

    Pose2d poseA = a.getVisionRobotPoseMeters();
    Pose2d poseB = b.getVisionRobotPoseMeters();

    if (poseAtA.isPresent() && poseAtB.isPresent()) {
      edu.wpi.first.math.geometry.Twist2d odometryTwist = poseAtA.get().log(poseAtB.get());
      poseA = poseA.exp(odometryTwist);
    }

    var varianceA = a.getVisionMeasurementStdDevs().elementTimes(a.getVisionMeasurementStdDevs());
    var varianceB = b.getVisionMeasurementStdDevs().elementTimes(b.getVisionMeasurementStdDevs());

    Rotation2d fusedHeading;
    if (varianceA.get(2, 0) < VisionConstants.kLargeVariance
        && varianceB.get(2, 0) < VisionConstants.kLargeVariance) {
      double weightRotA = 1.0 / varianceA.get(2, 0);
      double weightRotB = 1.0 / varianceB.get(2, 0);
      double t = weightRotB / (weightRotA + weightRotB);
      fusedHeading = poseA.getRotation().interpolate(poseB.getRotation(), t);
    } else {
      fusedHeading = poseB.getRotation();
    }

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    Pose2d fusedPose =
        new Pose2d(
            new Translation2d(
                (poseA.getX() * weightAx + poseB.getX() * weightBx) / (weightAx + weightBx),
                (poseA.getY() * weightAy + poseB.getY() * weightBy) / (weightAy + weightBy)),
            fusedHeading);

    Matrix<N3, N1> fusedStdDev =
        VecBuilder.fill(
            Math.sqrt(1.0 / (weightAx + weightBx)),
            Math.sqrt(1.0 / (weightAy + weightBy)),
            Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    int numTags = Math.max(a.getNumTags(), b.getNumTags());
    double time = b.getTimestampSeconds();
    double avgDist =
        (a.getAvgTagDistMeters() > 0 && b.getAvgTagDistMeters() > 0)
            ? Math.min(a.getAvgTagDistMeters(), b.getAvgTagDistMeters())
            : (a.getAvgTagDistMeters() > 0 ? a.getAvgTagDistMeters() : b.getAvgTagDistMeters());

    return new VisionFieldPoseEstimate(fusedPose, time, fusedStdDev, numTags, avgDist);
  }

  private Optional<VisionFieldPoseEstimate> fuseWithGyro(
      MegatagPoseEstimate poseEstimate, VisionIO.CameraInputs cam, Pose2d visionPose) {

    if (poseEstimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
      return Optional.empty();
    }

    if (poseEstimate.fiducialIds().length > 1) {
      return Optional.empty();
    }

    final double kHighYawLookbackS = VisionConstants.kHighYawLookbackSeconds;
    final double kHighYawVelocityRadS = VisionConstants.kHighYawVelocityRadPerSec;

    Optional<Double> maxYawRate =
        state.getMaxAbsDriveYawAngularVelocityInRange(
            poseEstimate.timestampSeconds() - kHighYawLookbackS, poseEstimate.timestampSeconds());

    if (maxYawRate.isPresent() && maxYawRate.get() > kHighYawVelocityRadS) {
      return Optional.empty();
    }

    var priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
    if (priorPose.isEmpty()) {
      return Optional.empty();
    }

    // Uses the dynamically translated correct visionPose
    Translation2d visionEstimatedTranslation = visionPose.getTranslation();
    Rotation2d gyroYaw = priorPose.get().getRotation();

    Pose2d posteriorPose = new Pose2d(visionEstimatedTranslation, gyroYaw);

    double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex];
    double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex];
    double xyStd = Math.max(xStd, yStd);

    return Optional.of(
        new VisionFieldPoseEstimate(
            posteriorPose,
            poseEstimate.timestampSeconds(),
            VecBuilder.fill(xyStd, xyStd, VisionConstants.kLargeVariance),
            poseEstimate.fiducialIds().length,
            poseEstimate.avgTagDist()));
  }

  /**
   * 嘗試透過高置信度的視覺訊號強制重置 Pigeon 陀螺儀 (Orbit2 邏輯)。
   *
   * <p>💡 反直覺工程觀點 (Counter-Intuitive Engineering Insight): 在高速運動時如果強制寫入硬體陀螺儀的 Yaw (如呼叫
   * setYaw)，會導致嚴重的相位跳變 (Phase Step)。 雖然直覺上更新感測器應該會讓控制「立刻變準」，但對底盤的 Swerve PID 來說，
   * 突然收到一個巨大的位移誤差，會輸出極端的角速度補償，導致馬達抽搐甚至發生過電流保護。 因此除了視覺資料本身必須具備極高可信度（超近距離 <2.0m、大面積/多標籤），
   * 真正的最後防線是「零分配的底盤速限檢查」：機器人必須近乎靜止。
   */
  private void attemptGyroReset(MegatagPoseEstimate poseEstimate, Pose2d visionPose) {
    if (Timer.getFPGATimestamp() - lastGyroResetTime < VisionConstants.kGyroResetDebounceSeconds) {
      return;
    }

    // 1. Distance constraint: Must be very close.
    if (poseEstimate.avgTagDist() > VisionConstants.kGyroResetMaxDistanceMeters) {
      return;
    }

    // 2. Tag constraint: Must have multiple tags, OR a single tag that is very
    // large (close).
    if (poseEstimate.fiducialIds().length < VisionConstants.kGyroResetMinTagCount) {
      if (poseEstimate.avgTagArea() < VisionConstants.kGyroResetMinTagArea) {
        return;
      }
    }

    // 3. Physical State constraint: Zero-allocation chassis stationary check
    // We check raw module speed and gyro speed mapped right out of the IO buffer to
    // avoid GC Jitter.
    if (Drive.getInstance() == null) {
      return;
    }
    var driveInputs = Drive.getInstance().getInputs();

    // Check module speeds
    for (int i = 0; i < 4; i++) {
      if (Math.abs(driveInputs.driveVelocityRotationsPerSec[i]) > 0.5) {
        return;
      }
    }

    // Check rotational speeds
    if (Math.abs(driveInputs.gyroYawVelocityRadPerSec) > 0.05) {
      return;
    }

    // Execute reset safely
    double headingToSet = visionPose.getRotation().getDegrees();
    Drive.getInstance().setGyroYawFromVision(headingToSet);
    lastGyroResetTime = Timer.getFPGATimestamp();

    System.out.println(
        "[Vision] Orbit-2 Gyro Calibration Triggered! Re-Aligned Pigeon to: "
            + headingToSet
            + " deg");
  }

  /** Processes a chassis mounted camera with a static transform. */
  private Optional<VisionFieldPoseEstimate> processStaticCamera(
      VisionIO.CameraInputs cam, String label, Transform3d robotToCamera) {

    if (!cam.seesTarget || cam.megatagPoseEstimate == null || cam.pose3d == null) {
      return Optional.empty();
    }

    MegatagPoseEstimate poseEstimate = cam.megatagPoseEstimate;

    // Apply Static Inverse Kinematics
    Pose3d cameraFieldPose = cam.pose3d;
    Pose3d robotFieldPose = cameraFieldPose.transformBy(robotToCamera.inverse());

    // Get true 2D chassis coordinates
    Pose2d correctedVisionPose = robotFieldPose.toPose2d();

    // Orbit2: Attempt to reset gyro hardware if close, trusted, and stationary
    attemptGyroReset(poseEstimate, correctedVisionPose);

    // Process MegaTag estimate using corrected pose
    Optional<VisionFieldPoseEstimate> mtEstimate =
        processMegatagPoseEstimate(poseEstimate, cam, correctedVisionPose, false);

    // Try gyro fusion for single-tag
    Optional<VisionFieldPoseEstimate> gyroEstimate =
        fuseWithGyro(poseEstimate, cam, correctedVisionPose);

    if (mtEstimate.isPresent()) {
      return mtEstimate;
    } else if (gyroEstimate.isPresent()) {
      return gyroEstimate;
    } else {
      return Optional.empty();
    }
  }

  /** Processes a turret mounted camera with a dynamic inverse kinematics. */
  private Optional<VisionFieldPoseEstimate> processDynamicCamera(
      VisionIO.CameraInputs cam, String label, Transform3d turretToCamera) {

    if (!cam.seesTarget || cam.megatagPoseEstimate == null || cam.pose3d == null) {
      return Optional.empty();
    }

    MegatagPoseEstimate poseEstimate = cam.megatagPoseEstimate;

    // Task 2: Construct Dynamic Inverse Kinematics Chain
    // Read historical angle (compensating for network and vision processing
    // latency)
    Rotation2d historicalAngle =
        mTurretAngleHistory
            .getSample(poseEstimate.timestampSeconds())
            .orElse(Turret.getInstance().getAngle());

    // Define robotToTurret
    Transform3d robotToTurret =
        new Transform3d(
            VisionConstants.kRobotToTurretCenter.getTranslation(),
            new Rotation3d(0, 0, historicalAngle.getRadians()));

    // Calculate dynamicRobotToCamera
    Transform3d dynamicRobotToCamera = robotToTurret.plus(turretToCamera);

    // Infer chassis coordinates
    Pose3d cameraFieldPose =
        cam.pose3d; // Raw factory camera field coordinates (assuming Web UI camera offsets are 0)
    Pose3d robotFieldPose = cameraFieldPose.transformBy(dynamicRobotToCamera.inverse());

    // Get true 2D chassis coordinates
    Pose2d correctedVisionPose = robotFieldPose.toPose2d();

    // Orbit2: Attempt to reset gyro hardware if close, trusted, and stationary
    attemptGyroReset(poseEstimate, correctedVisionPose);

    // Process MegaTag estimate using corrected pose
    // For dynamic cameras, we force disregard MT1 Heading (isDynamic = true)
    Optional<VisionFieldPoseEstimate> mtEstimate =
        processMegatagPoseEstimate(poseEstimate, cam, correctedVisionPose, true);

    // Try gyro fusion for single-tag
    Optional<VisionFieldPoseEstimate> gyroEstimate =
        fuseWithGyro(poseEstimate, cam, correctedVisionPose);

    if (mtEstimate.isPresent()) {
      return mtEstimate;
    } else if (gyroEstimate.isPresent()) {
      return gyroEstimate;
    } else {
      return Optional.empty();
    }
  }

  private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
      MegatagPoseEstimate poseEstimate,
      VisionIO.CameraInputs cam,
      Pose2d visionPose,
      boolean isDynamic) {

    if (poseEstimate.timestampSeconds() <= state.lastUsedMegatagTimestamp()) {
      return Optional.empty();
    }

    if (poseEstimate.fiducialIds().length < 2) {
      if (cam.fiducialObservations != null) {
        for (var fiducial : cam.fiducialObservations) {
          if (fiducial.ambiguity > VisionConstants.kDefaultAmbiguityThreshold) {
            return Optional.empty();
          }
        }
      }

      if (poseEstimate.avgTagArea() < VisionConstants.kTagMinAreaForSingleTagMegatag) {
        return Optional.empty();
      }

      Optional<Pose2d> priorPose = state.getFieldToRobot(poseEstimate.timestampSeconds());
      if (poseEstimate.avgTagArea() < VisionConstants.kTagAreaThresholdForYawCheck
          && priorPose.isPresent()) {
        double yawDiff =
            Math.abs(
                MathUtil.angleModulus(
                    priorPose.get().getRotation().getRadians()
                        - visionPose.getRotation().getRadians()));

        if (yawDiff > Math.toRadians(VisionConstants.kDefaultYawDiffThreshold)) {
          return Optional.empty();
        }
      }
    }

    Optional<Double> maxYawRate =
        state.getMaxAbsDriveYawAngularVelocityInRange(
            poseEstimate.timestampSeconds() - VisionConstants.kHighYawLookbackSeconds,
            poseEstimate.timestampSeconds());

    if (maxYawRate.isPresent() && maxYawRate.get() > VisionConstants.kHighYawVelocityRadPerSec) {
      return Optional.empty();
    }

    if (visionPose.getX() < 0
        || visionPose.getY() < 0
        || Double.isNaN(visionPose.getX())
        || Double.isNaN(visionPose.getY())) {
      return Optional.empty();
    }

    double scaleFactor = 1.0 / Math.max(poseEstimate.quality(), 0.1);
    double xStd = cam.standardDeviations[VisionConstants.kMegatag1XStdDevIndex] * scaleFactor;
    double yStd = cam.standardDeviations[VisionConstants.kMegatag1YStdDevIndex] * scaleFactor;

    // Task 3: MT1 Noise Rejection (Specifically for dynamic turret camera)
    // Since MT1 relies on pure vision PnP, tiny pixel jitters over 3 meters are
    // amplified into intense Yaw noise.
    // If it is dynamic, completely discard MT1 Heading: completely distruct the
    // camera yaw caused by turret rotation.
    double rotStd = isDynamic ? 9999999.0 : cam.standardDeviations[2] * scaleFactor;

    double xyStd = Math.max(xStd, yStd);
    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStd, xyStd, rotStd);

    return Optional.of(
        new VisionFieldPoseEstimate(
            visionPose,
            poseEstimate.timestampSeconds(),
            visionStdDevs,
            poseEstimate.fiducialIds().length,
            poseEstimate.avgTagDist()));
  }

  private void logCameraInputs(String prefix, VisionIO.CameraInputs cam, boolean connected) {}

  @Override
  public void stop() {}

  @Override
  public void zeroSensors() {
    acceptedEstimateCount = 0;
  }

  @Override
  public boolean checkConnectionActive() {
    return inputs.cameraFrontConnected || inputs.cameraLeftConnected || inputs.cameraRightConnected;
  }

  @Override
  public boolean checkConnectionPassive() {
    return inputs.cameraFrontConnected || inputs.cameraLeftConnected || inputs.cameraRightConnected;
  }

  @Override
  public boolean checkSanityPassive() {
    if (!useVision) {
      return true;
    }
    return acceptedEstimateCount > 0;
  }

  @Override
  public void outputTelemetry() {}

  public void setUseVision(boolean useVision) {
    this.useVision = useVision;
  }

  public boolean isUsingVision() {
    return useVision;
  }

  public boolean isCameraFrontConnected() {
    return inputs.cameraFrontConnected;
  }

  public boolean isCameraLeftConnected() {
    return inputs.cameraLeftConnected;
  }

  public boolean isCameraRightConnected() {
    return inputs.cameraRightConnected;
  }
}
