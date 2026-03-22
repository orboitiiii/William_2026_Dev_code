package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.PoseHistory;
import frc.robot.VisionConstants;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.subsystems.drive.Drive;
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
 *   <li>Triple camera MegaTag pose estimation processing (Front, Left, Up)
 *   <li>Inverse-variance weighting fusion of multi-camera estimates
 *   <li>Single-tag gyro fusion for improved accuracy (fuseWithGyro)
 *   <li>Dynamic standard deviation adjustment
 *   <li>Vision measurement rejection logic
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

  // We removed turret dynamic camera processing, so no longer need
  // mTurretAngleHistory

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
          public void onStart(double timestamp) {}

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
  }

  @Override
  public void writePeriodicOutputs() {
    logCameraInputs("Vision/CameraFront", inputs.cameraFront, inputs.cameraFrontConnected);
    logCameraInputs("Vision/CameraLeft", inputs.cameraLeft, inputs.cameraLeftConnected);
    logCameraInputs("Vision/CameraUp", inputs.cameraUp, inputs.cameraUpConnected);

    updatePipelineAndCrop(VisionConstants.kFrontLimelightName, inputs.cameraFront);
    updatePipelineAndCrop(VisionConstants.kLeftLimelightName, inputs.cameraLeft);
    updatePipelineAndCrop(VisionConstants.kUpLimelightName, inputs.cameraUp);

    if (!useVision || frc.robot.DashboardState.getInstance().isApriltagDisabled()) {
      return;
    }

    // 靜態相機（前、左）：Limelight Web UI 已配置 camera offset，
    // 因此 MegaTag1 的 botpose 已經是真正的機器人場座標 Pose，不需任何轉換。
    List<VisionFieldPoseEstimate> validEstimates = new ArrayList<>();

    processStaticCamera(inputs.cameraFront, "CameraFront").ifPresent(validEstimates::add);
    processStaticCamera(inputs.cameraLeft, "CameraLeft").ifPresent(validEstimates::add);
    processStaticCamera(inputs.cameraUp, "CameraUp").ifPresent(validEstimates::add);

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
          // frc.robot.DashboardState.getInstance().lastVisionTimestamp =
          // est.getTimestampSeconds();
        });

    frc.robot.DashboardState.getInstance().frontLLOK = inputs.cameraFrontConnected;
  }

  private void updatePipelineAndCrop(String cameraName, VisionIO.CameraInputs cam) {
    if (!cam.seesTarget) {
      io.setPipelineIndex(cameraName, VisionConstants.kPipelineSearch);
      io.setCropWindow(cameraName, -1.0, 1.0, -1.0, 1.0);
      return;
    }
    io.setPipelineIndex(cameraName, VisionConstants.kPipelineTrack);

    double latencyMargin = cam.latencyPipelineMs * VisionConstants.kCropMarginPerMsLatency;
    double halfX = VisionConstants.kCropHalfSizeHoriz + latencyMargin;
    double halfY = VisionConstants.kCropHalfSizeVert + latencyMargin;

    double xCenter =
        (cam.txDeg + VisionConstants.kFovHorizDeg / 2.0) / VisionConstants.kFovHorizDeg;
    double yCenter = (cam.tyDeg + VisionConstants.kFovVertDeg / 2.0) / VisionConstants.kFovVertDeg;

    mCropXMin = xCenter - halfX;
    mCropXMax = xCenter + halfX;
    mCropYMin = yCenter - halfY;
    mCropYMax = yCenter + halfY;

    expandCropForNeighborTags(cam);

    double xMin = MathUtil.clamp(mCropXMin * 2.0 - 1.0, -1.0, 1.0);
    double xMax = MathUtil.clamp(mCropXMax * 2.0 - 1.0, -1.0, 1.0);
    double yMin = MathUtil.clamp(mCropYMin * 2.0 - 1.0, -1.0, 1.0);
    double yMax = MathUtil.clamp(mCropYMax * 2.0 - 1.0, -1.0, 1.0);
    io.setCropWindow(cameraName, xMin, xMax, yMin, yMax);
  }

  private double mCropXMin, mCropXMax, mCropYMin, mCropYMax;

  /**
   * Expands the crop bounding box toward neighbor tags that the field layout predicts are nearby.
   * Uses the robot's current estimated pose and the detected tag's known position to project
   * neighbor locations into normalized camera coordinates.
   */
  private void expandCropForNeighborTags(VisionIO.CameraInputs cam) {
    if (cam.fiducialObservations == null || cam.fiducialObservations.length == 0) return;

    int seenId = cam.fiducialObservations[0].id;
    var layout = frc.robot.FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
    if (layout == null) return;

    var seenTagPoseOpt = layout.getTagPose(seenId);
    if (seenTagPoseOpt.isEmpty()) return;

    Translation2d seenTag2d = seenTagPoseOpt.get().toPose2d().getTranslation();

    Pose2d robotPose;
    try {
      robotPose = frc.robot.subsystems.RobotStateEstimator.getInstance().getEstimatedPose();
    } catch (Exception e) {
      return;
    }

    double marginNormX = VisionConstants.kCropNeighborMarginDeg / VisionConstants.kFovHorizDeg;

    for (var tag : layout.getTags()) {
      if (tag.ID == seenId) continue;
      Translation2d neighbor2d = tag.pose.toPose2d().getTranslation();
      if (seenTag2d.getDistance(neighbor2d) > VisionConstants.kNeighborSearchRadiusMeters) continue;

      Translation2d robotToNeighbor = neighbor2d.minus(robotPose.getTranslation());
      double angleToNeighbor =
          Math.toDegrees(Math.atan2(robotToNeighbor.getY(), robotToNeighbor.getX()))
              - robotPose.getRotation().getDegrees();
      double neighborTxDeg = MathUtil.inputModulus(angleToNeighbor, -180, 180);

      if (Math.abs(neighborTxDeg) > VisionConstants.kFovHorizDeg / 2.0) continue;

      double neighborXNorm =
          (neighborTxDeg + VisionConstants.kFovHorizDeg / 2.0) / VisionConstants.kFovHorizDeg;

      mCropXMin = Math.min(mCropXMin, neighborXNorm - marginNormX);
      mCropXMax = Math.max(mCropXMax, neighborXNorm + marginNormX);
    }
  }

  /**
   * Fuses multiple vision pose estimates using inverse-variance weighting, but only if the
   * estimates agree within their combined uncertainty. When a close-tag camera and a far-tag camera
   * disagree significantly, the far estimate is discarded to prevent oscillation.
   */
  private VisionFieldPoseEstimate fuseMultipleEstimates(List<VisionFieldPoseEstimate> estimates) {
    if (estimates.size() == 0) return null;
    if (estimates.size() == 1) return estimates.get(0);

    // Find the estimate with the lowest XY stdDev (most trustworthy)
    VisionFieldPoseEstimate best = estimates.get(0);
    double bestStd = best.getVisionMeasurementStdDevs().get(0, 0);
    for (int i = 1; i < estimates.size(); i++) {
      double std = estimates.get(i).getVisionMeasurementStdDevs().get(0, 0);
      if (std < bestStd) {
        best = estimates.get(i);
        bestStd = std;
      }
    }

    // Only fuse estimates that agree with the best one within their combined
    // uncertainty. Disagreeing far-tag estimates are dropped.
    List<VisionFieldPoseEstimate> compatible = new ArrayList<>();
    compatible.add(best);

    for (var est : estimates) {
      if (est == best) continue;

      double dist =
          best.getVisionRobotPoseMeters()
              .getTranslation()
              .getDistance(est.getVisionRobotPoseMeters().getTranslation());
      double combinedStd =
          best.getVisionMeasurementStdDevs().get(0, 0)
              + est.getVisionMeasurementStdDevs().get(0, 0);

      if (dist < combinedStd * 2.0) {
        compatible.add(est);
      }
    }

    VisionFieldPoseEstimate fused = compatible.get(0);
    for (int i = 1; i < compatible.size(); i++) {
      fused = fuseEstimates(fused, compatible.get(i));
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

    boolean aHasHeading = varianceA.get(2, 0) < VisionConstants.kLargeVariance;
    boolean bHasHeading = varianceB.get(2, 0) < VisionConstants.kLargeVariance;

    Rotation2d fusedHeading;
    double fusedRotVariance;

    if (aHasHeading && bHasHeading) {
      double wA = 1.0 / varianceA.get(2, 0);
      double wB = 1.0 / varianceB.get(2, 0);
      fusedHeading =
          new Rotation2d(
              poseA.getRotation().getCos() * wA + poseB.getRotation().getCos() * wB,
              poseA.getRotation().getSin() * wA + poseB.getRotation().getSin() * wB);
      fusedRotVariance = 1.0 / (wA + wB);
    } else if (aHasHeading) {
      fusedHeading = poseA.getRotation();
      fusedRotVariance = varianceA.get(2, 0);
    } else if (bHasHeading) {
      fusedHeading = poseB.getRotation();
      fusedRotVariance = varianceB.get(2, 0);
    } else {
      fusedHeading = poseB.getRotation();
      fusedRotVariance = VisionConstants.kLargeVariance;
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
            Math.sqrt(fusedRotVariance));

    int numTags = a.getNumTags() + b.getNumTags();
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
    if (!VisionConstants.kEnableGyroReset) {
      return;
    }

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

  /**
   * Processes a chassis-mounted static camera.
   *
   * <p>⚠️ 由於 Limelight Web UI 已配置好 camera offset， 因此 botpose 即為真正的機器人場座標位姿，不需額外的 inverse() 轉換。
   *
   * <p>Validation gates (254 pattern): Z-height, position norm, field bounds.
   */
  private Optional<VisionFieldPoseEstimate> processStaticCamera(
      VisionIO.CameraInputs cam, String label) {

    if (!cam.seesTarget || cam.megatagPoseEstimate == null || cam.pose3d == null) {
      return Optional.empty();
    }

    if (Math.abs(cam.pose3d.getZ()) > VisionConstants.kMaxZHeightMeters) {
      return Optional.empty();
    }

    MegatagPoseEstimate poseEstimate = cam.megatagPoseEstimate;

    // 💡 反直覺工程觀點 (Counter-Intuitive Engineering Insight):
    // 直覺上，多重感測器融合 (EKF) “有訊號總比沒訊號好”，因為可以藉由調高大距離測量的 Standard Deviation (測量標準差)
    // 來降低權重。
    // 但在 FRC 物理現實中：單一 AprilTag 在遠距離時，光學畸變與微小像素抖動都會被阿貝誤差 (Abbe Error) 放大為巨大的位移跳變。
    // 這種非高斯分佈的跳變訊號會強烈拉扯 EKF，導致幽靈旋轉。多標籤雖然能消除 Ambiguity，但也仍有硬性物理極限。
    // 在 1.8m (單標籤) 或 2.5m (多標籤) 之外，Swerve 高頻 (250Hz) 的四輪硬體計數純死區推算 (Dead-Reckoning)
    // 精度，
    // 實際上遠遠優於受噪聲污染的視覺測量。我們採用「嚴格硬性切斷 (Hard Cutoff)」捨棄不良訊號，讓里程計安靜地發揮它的作用。
    double maxAllowedDistance =
        poseEstimate.fiducialIds().length == 1
            ? VisionConstants.kMaxSingleTagDistanceMeters
            : VisionConstants.kMaxMultiTagDistanceMeters;

    if (poseEstimate.avgTagDist() > maxAllowedDistance) {
      return Optional.empty();
    }

    Pose2d visionPose = cam.pose3d.toPose2d();

    if (visionPose.getTranslation().getNorm() < VisionConstants.kMinPositionNormMeters) {
      return Optional.empty();
    }

    if (visionPose.getX() < 0
        || visionPose.getX() > VisionConstants.kFieldLengthMeters
        || visionPose.getY() < 0
        || visionPose.getY() > VisionConstants.kFieldWidthMeters) {
      return Optional.empty();
    }

    attemptGyroReset(poseEstimate, visionPose);

    Optional<VisionFieldPoseEstimate> mtEstimate =
        processMegatagPoseEstimate(poseEstimate, cam, visionPose, false);

    Optional<VisionFieldPoseEstimate> gyroEstimate = fuseWithGyro(poseEstimate, cam, visionPose);

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

    if (Double.isNaN(visionPose.getX()) || Double.isNaN(visionPose.getY())) {
      return Optional.empty();
    }

    // 💡 反直覺工程觀點 (Counter-Intuitive Engineering Insight): 二次衰減權重
    // 原本直接沿用 Limelight 算出的 StdDev 與 Quality 做融合信任度 (Inverse-Variance Weighting)。
    // 但 Limelight 的 Quality 對「阿貝誤差 (Abbe Error)」造成的側向位移放大非常不敏感。
    // 如果把 0.5m (精準) 跟 2.0m (受畸變污染) 的相機訊號「平均融合」，結果會是毀滅性的晃動 (Jitter)。
    // 因此，我們改採 Team 254 的物理幾何退化模型：信任懲罰必須隨「距離的平方 (Distance Squared)」成正比。
    // 這創造了一個極端陡峭的信任函數，在多相機融合時實質上強制變成了「Winner-Takes-All (距離最近者全拿)」。

    double dist = poseEstimate.avgTagDist();
    int numTags = poseEstimate.fiducialIds().length;

    // 二次方程式幾何退化模型
    double xyStdDev =
        VisionConstants.kBaseXYStdDev
            + (dist * VisionConstants.kXYStdDevPerMeter)
            + (Math.pow(dist, 2) * VisionConstants.kXYStdDevPerMeterSq);

    // 單標籤的 Heading (Yaw) 在遠距離極度不可靠，給予專屬懲罰
    double thetaStdDev =
        (numTags == 1) ? VisionConstants.kSingleTagThetaStdDev : VisionConstants.kBaseThetaStdDev;

    // If it is dynamic OR kForceGyroHeading is active, we completely discard
    // vision Heading by setting variance to INFINITY.
    double rotStd =
        (isDynamic || VisionConstants.kForceGyroHeading)
            ? VisionConstants.kLargeVariance
            : thetaStdDev;

    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(xyStdDev, xyStdDev, rotStd);

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
    return inputs.cameraFrontConnected || inputs.cameraLeftConnected || inputs.cameraUpConnected;
  }

  @Override
  public boolean checkConnectionPassive() {
    return inputs.cameraFrontConnected || inputs.cameraLeftConnected || inputs.cameraUpConnected;
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

  public boolean isCameraUpConnected() {
    return inputs.cameraUpConnected;
  }
}
