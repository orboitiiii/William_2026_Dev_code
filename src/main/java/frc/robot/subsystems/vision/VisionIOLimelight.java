package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.VisionConstants;

/**
 * VisionIO Limelight implementation.
 *
 * <p>Uses LimelightHelpers v1.14 to read vision data from dual Limelights.
 *
 * <p>Design decisions (Reference: 254 2025):
 *
 * <ul>
 *   <li>Uses MegaTag1 instead of MegaTag2 (LL4 internal IMU not accurate enough)
 *   <li>Timestamps use NT server time with latency correction
 *   <li>Connection status detected via heartbeat changes
 * </ul>
 */
public class VisionIOLimelight implements VisionIO {

  private final String cameraFrontName;
  private final String cameraLeftName;
  private final String cameraUpName;

  private double lastHeartbeatFront = 0.0;
  private double lastHeartbeatLeft = 0.0;
  private double lastHeartbeatUp = 0.0;
  private double lastUpdateTimeFront = 0.0;
  private double lastUpdateTimeLeft = 0.0;
  private double lastUpdateTimeUp = 0.0;

  private static final double CONNECTION_TIMEOUT_SEC = 1.0;

  private final double[] mSmoothedStdDevFront = {1.0, 1.0, VisionConstants.kLargeVariance};
  private final double[] mSmoothedStdDevLeft = {1.0, 1.0, VisionConstants.kLargeVariance};
  private final double[] mSmoothedStdDevUp = {1.0, 1.0, VisionConstants.kLargeVariance};

  /** Creates triple Limelight vision IO with default names. */
  public VisionIOLimelight() {
    this(
        VisionConstants.kFrontLimelightName,
        VisionConstants.kLeftLimelightName,
        VisionConstants.kUpLimelightName);
  }

  /**
   * Creates triple Limelight vision IO.
   *
   * @param cameraFrontName Front Camera NetworkTables name
   * @param cameraLeftName Left Camera NetworkTables name
   * @param cameraUpName Up Camera NetworkTables name
   */
  public VisionIOLimelight(String cameraFrontName, String cameraLeftName, String cameraUpName) {
    this.cameraFrontName = cameraFrontName;
    this.cameraLeftName = cameraLeftName;
    this.cameraUpName = cameraUpName;
  }

  @Override
  public void readInputs(VisionIOInputs inputs) {
    double currentTime = Timer.getFPGATimestamp();

    // Read Camera Front
    readCameraInputs(inputs.cameraFront, cameraFrontName, lastHeartbeatFront);
    double heartbeatFront = LimelightHelpers.getHeartbeat(cameraFrontName);
    if (heartbeatFront != lastHeartbeatFront) {
      lastHeartbeatFront = heartbeatFront;
      lastUpdateTimeFront = currentTime;
    }
    inputs.cameraFrontConnected = (currentTime - lastUpdateTimeFront) < CONNECTION_TIMEOUT_SEC;

    // Read Camera Left
    readCameraInputs(inputs.cameraLeft, cameraLeftName, lastHeartbeatLeft);
    double heartbeatLeft = LimelightHelpers.getHeartbeat(cameraLeftName);
    if (heartbeatLeft != lastHeartbeatLeft) {
      lastHeartbeatLeft = heartbeatLeft;
      lastUpdateTimeLeft = currentTime;
    }
    inputs.cameraLeftConnected = (currentTime - lastUpdateTimeLeft) < CONNECTION_TIMEOUT_SEC;

    // Read Camera Up
    readCameraInputs(inputs.cameraUp, cameraUpName, lastHeartbeatUp);
    double heartbeatUp = LimelightHelpers.getHeartbeat(cameraUpName);
    if (heartbeatUp != lastHeartbeatUp) {
      lastHeartbeatUp = heartbeatUp;
      lastUpdateTimeUp = currentTime;
    }
    inputs.cameraUpConnected = (currentTime - lastUpdateTimeUp) < CONNECTION_TIMEOUT_SEC;
  }

  /** Reads single camera inputs. */
  private void readCameraInputs(CameraInputs inputs, String cameraName, double lastHeartbeat) {
    inputs.seesTarget = LimelightHelpers.getTV(cameraName);
    inputs.heartbeat = LimelightHelpers.getHeartbeat(cameraName);
    inputs.lastHeartbeat = lastHeartbeat;

    if (!inputs.seesTarget) {
      inputs.megatagCount = 0;
      inputs.pose3d = null;
      inputs.megatagPoseEstimate = null;
      inputs.fiducialObservations = null;
      inputs.txDeg = 0.0;
      inputs.tyDeg = 0.0;
      inputs.latencyPipelineMs = 0.0;
      return;
    }

    inputs.txDeg = LimelightHelpers.getTX(cameraName);
    inputs.tyDeg = LimelightHelpers.getTY(cameraName);
    inputs.latencyPipelineMs = LimelightHelpers.getLatency_Pipeline(cameraName);

    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (poseEstimate != null && LimelightHelpers.validPoseEstimate(poseEstimate)) {
      inputs.megatagCount = poseEstimate.tagCount;
      inputs.pose3d = new Pose3d(poseEstimate.pose);

      int[] fiducialIds = new int[poseEstimate.rawFiducials.length];
      for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
        fiducialIds[i] = poseEstimate.rawFiducials[i].id;
      }

      double quality = calculateQuality(poseEstimate);

      inputs.megatagPoseEstimate =
          new MegatagPoseEstimate(
              poseEstimate.pose,
              poseEstimate.timestampSeconds,
              fiducialIds,
              quality,
              poseEstimate.avgTagArea,
              poseEstimate.avgTagDist,
              poseEstimate.tagSpan);

      inputs.fiducialObservations = poseEstimate.rawFiducials;

      double[] rawStd = calculateStandardDeviations(poseEstimate, cameraName);
      double[] buf = getSmoothedBuffer(cameraName);
      double a = VisionConstants.kStdDevSmoothingAlpha;
      for (int j = 0; j < 3; j++) {
        buf[j] = a * rawStd[j] + (1.0 - a) * buf[j];
      }
      inputs.standardDeviations = new double[] {buf[0], buf[1], buf[2]};
    } else {
      inputs.megatagCount = 0;
      inputs.pose3d = null;
      inputs.megatagPoseEstimate = null;
      inputs.fiducialObservations = null;
    }
  }

  /**
   * Calculates estimate quality (0-1).
   *
   * <p>Multi-tag = 1.0 (fully constrained). Single-tag = weighted combination of area, distance,
   * and worst-case ambiguity across observed fiducials.
   */
  private double calculateQuality(PoseEstimate estimate) {
    if (estimate.tagCount >= 2) {
      double spanBonus = Math.min(estimate.tagSpan / 2.0, 1.0) * 0.1;
      return Math.min(0.9 + spanBonus, 1.0);
    }

    double worstAmbiguity = 0.0;
    for (var fid : estimate.rawFiducials) {
      worstAmbiguity = Math.max(worstAmbiguity, fid.ambiguity);
    }
    double ambiguityContrib = Math.max(0, 1.0 - worstAmbiguity / 0.3) * 0.4;
    double areaContrib = Math.min(estimate.avgTagArea / 3.0, 1.0) * 0.3;
    double distContrib = Math.max(0, 1.0 - estimate.avgTagDist / 5.0) * 0.3;

    return Math.min(ambiguityContrib + areaContrib + distContrib, 1.0);
  }

  private double[] getSmoothedBuffer(String cameraName) {
    if (cameraName.equals(cameraFrontName)) return mSmoothedStdDevFront;
    if (cameraName.equals(cameraLeftName)) return mSmoothedStdDevLeft;
    return mSmoothedStdDevUp;
  }

  /** Returns the measured reprojection error for the given camera. */
  private double getReprojectionError(String cameraName) {
    if (cameraName.equals(VisionConstants.kFrontLimelightName)) {
      return VisionConstants.kFrontReprojectionError;
    } else if (cameraName.equals(VisionConstants.kLeftLimelightName)) {
      return VisionConstants.kLeftReprojectionError;
    } else if (cameraName.equals(VisionConstants.kUpLimelightName)) {
      return VisionConstants.kUpReprojectionError;
    }
    return VisionConstants.kReferenceReprojectionError;
  }

  /**
   * Calculates measurement standard deviations with per-camera reprojection-error scaling and
   * quadratic distance term (PnP error grows with distance²).
   */
  private double[] calculateStandardDeviations(PoseEstimate estimate, String cameraName) {
    double dist = estimate.avgTagDist;
    double linearTerm = dist * VisionConstants.kXYStdDevPerMeter;
    double quadraticTerm = dist * dist * VisionConstants.kXYStdDevPerMeterSq;

    double tagCountFactor = 1.0 / (1.0 + 0.8 * (estimate.tagCount - 1));
    double reprojScale =
        getReprojectionError(cameraName) / VisionConstants.kReferenceReprojectionError;

    double xyStdDev =
        (VisionConstants.kBaseXYStdDev + linearTerm + quadraticTerm) * tagCountFactor * reprojScale;
    double thetaStdDev =
        estimate.tagCount < 2
            ? VisionConstants.kSingleTagThetaStdDev
            : VisionConstants.kBaseThetaStdDev * tagCountFactor * reprojScale;

    // Reduce trust for single-tag at long range to avoid PnP jitter (pixel noise amplifies with
    // distance)
    if (estimate.tagCount < 2
        && dist > VisionConstants.kSingleTagFarStdDevDistanceThresholdMeters) {
      double extra =
          1.0
              + (dist - VisionConstants.kSingleTagFarStdDevDistanceThresholdMeters)
                  * VisionConstants.kSingleTagFarStdDevScalePerMeter;
      xyStdDev *= extra;
      thetaStdDev *= extra;
    }

    return new double[] {xyStdDev, xyStdDev, thetaStdDev};
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
    // Set robot orientation for MegaTag2 (keeping functionality even though we use
    // MegaTag1)
    LimelightHelpers.SetRobotOrientation_NoFlush(
        cameraFrontName, yawDegrees, yawRateDegPerSec, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation_NoFlush(
        cameraLeftName, yawDegrees, yawRateDegPerSec, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation_NoFlush(
        cameraUpName, yawDegrees, yawRateDegPerSec, 0, 0, 0, 0);
  }

  @Override
  public void setLEDMode(int mode) {
    switch (mode) {
      case 0 -> {
        LimelightHelpers.setLEDMode_PipelineControl(cameraFrontName);
        LimelightHelpers.setLEDMode_PipelineControl(cameraLeftName);
        LimelightHelpers.setLEDMode_PipelineControl(cameraUpName);
      }
      case 1 -> {
        LimelightHelpers.setLEDMode_ForceOff(cameraFrontName);
        LimelightHelpers.setLEDMode_ForceOff(cameraLeftName);
        LimelightHelpers.setLEDMode_ForceOff(cameraUpName);
      }
      case 2 -> {
        LimelightHelpers.setLEDMode_ForceBlink(cameraFrontName);
        LimelightHelpers.setLEDMode_ForceBlink(cameraLeftName);
        LimelightHelpers.setLEDMode_ForceBlink(cameraUpName);
      }
      case 3 -> {
        LimelightHelpers.setLEDMode_ForceOn(cameraFrontName);
        LimelightHelpers.setLEDMode_ForceOn(cameraLeftName);
        LimelightHelpers.setLEDMode_ForceOn(cameraUpName);
      }
    }
  }

  @Override
  public void setPipelineIndex(int index) {
    LimelightHelpers.setPipelineIndex(cameraFrontName, index);
    LimelightHelpers.setPipelineIndex(cameraLeftName, index);
    LimelightHelpers.setPipelineIndex(cameraUpName, index);
  }

  @Override
  public void setPipelineIndex(String cameraName, int index) {
    LimelightHelpers.setPipelineIndex(cameraName, index);
  }

  @Override
  public void setCropWindow(String cameraName, double xMin, double xMax, double yMin, double yMax) {
    LimelightHelpers.setCropWindow(cameraName, xMin, xMax, yMin, yMax);
  }
}
