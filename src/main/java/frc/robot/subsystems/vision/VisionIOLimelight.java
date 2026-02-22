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
  private final String cameraRightName;

  private double lastHeartbeatFront = 0.0;
  private double lastHeartbeatLeft = 0.0;
  private double lastHeartbeatRight = 0.0;
  private double lastUpdateTimeFront = 0.0;
  private double lastUpdateTimeLeft = 0.0;
  private double lastUpdateTimeRight = 0.0;

  // Connection detection timeout (seconds)
  private static final double CONNECTION_TIMEOUT_SEC = 1.0;

  /** Creates triple Limelight vision IO with default names. */
  public VisionIOLimelight() {
    this(
        VisionConstants.kFrontLimelightName,
        VisionConstants.kLeftLimelightName,
        VisionConstants.kRightLimelightName);
  }

  /**
   * Creates triple Limelight vision IO.
   *
   * @param cameraFrontName Front Camera NetworkTables name
   * @param cameraLeftName Left Camera NetworkTables name
   * @param cameraRightName Right Camera NetworkTables name
   */
  public VisionIOLimelight(String cameraFrontName, String cameraLeftName, String cameraRightName) {
    this.cameraFrontName = cameraFrontName;
    this.cameraLeftName = cameraLeftName;
    this.cameraRightName = cameraRightName;
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

    // Read Camera Right
    readCameraInputs(inputs.cameraRight, cameraRightName, lastHeartbeatRight);
    double heartbeatRight = LimelightHelpers.getHeartbeat(cameraRightName);
    if (heartbeatRight != lastHeartbeatRight) {
      lastHeartbeatRight = heartbeatRight;
      lastUpdateTimeRight = currentTime;
    }
    inputs.cameraRightConnected = (currentTime - lastUpdateTimeRight) < CONNECTION_TIMEOUT_SEC;
  }

  /** Reads single camera inputs. */
  private void readCameraInputs(CameraInputs inputs, String cameraName, double lastHeartbeat) {
    // Basic target detection
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

    // Get MegaTag1 estimate (using wpiBlue coordinate system)
    PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (poseEstimate != null && LimelightHelpers.validPoseEstimate(poseEstimate)) {
      inputs.megatagCount = poseEstimate.tagCount;
      inputs.pose3d = new Pose3d(poseEstimate.pose);

      // Build fiducial IDs array
      int[] fiducialIds = new int[poseEstimate.rawFiducials.length];
      for (int i = 0; i < poseEstimate.rawFiducials.length; i++) {
        fiducialIds[i] = poseEstimate.rawFiducials[i].id;
      }

      // Calculate quality (based on tag area and count)
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

      // Calculate standard deviations (based on distance and tag count)
      inputs.standardDeviations = calculateStandardDeviations(poseEstimate);
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
   * <p>Quality based on: tag count, area, distance
   */
  private double calculateQuality(PoseEstimate estimate) {
    double quality = 0.0;

    // Tag count contribution
    quality += Math.min(estimate.tagCount / 4.0, 1.0) * 0.4;

    // Area contribution (larger area = higher quality)
    quality += Math.min(estimate.avgTagArea / 5.0, 1.0) * 0.3;

    // Distance contribution (closer distance = higher quality)
    double distFactor = Math.max(0, 1.0 - estimate.avgTagDist / 6.0);
    quality += distFactor * 0.3;

    return Math.min(quality, 1.0);
  }

  /**
   * Calculates measurement standard deviations.
   *
   * <p>Reference 254 2025: std devs increase with distance, decrease with tag count
   */
  private double[] calculateStandardDeviations(PoseEstimate estimate) {
    double baseFactor = VisionConstants.kBaseXYStdDev;
    double distFactor = estimate.avgTagDist * VisionConstants.kXYStdDevPerMeter;

    // Reduce std dev for multi-tag
    double tagCountFactor = estimate.tagCount >= 2 ? 0.5 : 1.0;

    double xyStdDev = (baseFactor + distFactor) * tagCountFactor;
    double thetaStdDev = VisionConstants.kBaseThetaStdDev * tagCountFactor;

    // Increase theta std dev for single tag (heading uncertainty is higher)
    if (estimate.tagCount < 2) {
      thetaStdDev = VisionConstants.kLargeVariance;
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
        cameraRightName, yawDegrees, yawRateDegPerSec, 0, 0, 0, 0);
  }

  @Override
  public void setLEDMode(int mode) {
    switch (mode) {
      case 0 -> {
        LimelightHelpers.setLEDMode_PipelineControl(cameraFrontName);
        LimelightHelpers.setLEDMode_PipelineControl(cameraLeftName);
        LimelightHelpers.setLEDMode_PipelineControl(cameraRightName);
      }
      case 1 -> {
        LimelightHelpers.setLEDMode_ForceOff(cameraFrontName);
        LimelightHelpers.setLEDMode_ForceOff(cameraLeftName);
        LimelightHelpers.setLEDMode_ForceOff(cameraRightName);
      }
      case 2 -> {
        LimelightHelpers.setLEDMode_ForceBlink(cameraFrontName);
        LimelightHelpers.setLEDMode_ForceBlink(cameraLeftName);
        LimelightHelpers.setLEDMode_ForceBlink(cameraRightName);
      }
      case 3 -> {
        LimelightHelpers.setLEDMode_ForceOn(cameraFrontName);
        LimelightHelpers.setLEDMode_ForceOn(cameraLeftName);
        LimelightHelpers.setLEDMode_ForceOn(cameraRightName);
      }
    }
  }

  @Override
  public void setPipelineIndex(int index) {
    LimelightHelpers.setPipelineIndex(cameraFrontName, index);
    LimelightHelpers.setPipelineIndex(cameraLeftName, index);
    LimelightHelpers.setPipelineIndex(cameraRightName, index);
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
