package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers.RawFiducial;

/**
 * Vision IO interface - Hardware abstraction layer.
 *
 * <p>Defines the interface between vision subsystem and Limelight hardware, allowing switching
 * between real hardware and simulation.
 *
 * <p>Reference: AdvantageKit IO layer design pattern.
 */
public interface VisionIO {

  /** Single camera input data. */
  class CameraInputs {
    /** Whether a valid target is detected */
    public boolean seesTarget = false;

    /** Heartbeat value - for connection status detection */
    public double heartbeat = 0.0;

    /** Previous heartbeat value - for update detection */
    public double lastHeartbeat = 0.0;

    /** MegaTag estimated tag count */
    public int megatagCount = 0;

    /** 3D robot pose (if available) */
    public Pose3d pose3d = null;

    /** MegaTag pose estimate */
    public MegatagPoseEstimate megatagPoseEstimate = null;

    /** Raw Fiducial observation data */
    public RawFiducial[] fiducialObservations = null;

    /** Standard deviation array [x, y, theta] */
    public double[] standardDeviations = new double[] {1.0, 1.0, 1.0};

    /** Target X angle (deg) - for dynamic crop when target visible */
    public double txDeg = 0.0;

    /** Target Y angle (deg) - for dynamic crop */
    public double tyDeg = 0.0;

    /** Pipeline latency (ms) - for crop margin */
    public double latencyPipelineMs = 0.0;
  }

  /** All vision input data. */
  class VisionIOInputs {
    /** Front camera inputs */
    public CameraInputs cameraFront = new CameraInputs();

    /** Left camera (base) inputs */
    public CameraInputs cameraLeft = new CameraInputs();

    /** Right camera (turret) inputs */
    public CameraInputs cameraRight = new CameraInputs();

    /** Front camera connection status */
    public boolean cameraFrontConnected = false;

    /** Left camera connection status */
    public boolean cameraLeftConnected = false;

    /** Right camera connection status */
    public boolean cameraRightConnected = false;
  }

  /**
   * Reads vision inputs from hardware.
   *
   * @param inputs The input object to populate
   */
  void readInputs(VisionIOInputs inputs);

  /**
   * Sets robot orientation (for MegaTag2).
   *
   * @param yawDegrees Robot yaw angle (degrees)
   * @param yawRateDegPerSec Yaw angular velocity (degrees/sec)
   */
  default void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {}

  /**
   * Sets LED mode.
   *
   * @param mode LED mode (0=pipeline, 1=off, 2=blink, 3=on)
   */
  default void setLEDMode(int mode) {}

  /**
   * Sets pipeline index for both cameras.
   *
   * @param index Pipeline index (0-9)
   */
  default void setPipelineIndex(int index) {}

  /**
   * Sets pipeline index for one camera (Orbit2: per-camera pipeline for search vs track).
   *
   * @param cameraName NetworkTables name of the Limelight
   * @param index Pipeline index (0-9)
   */
  default void setPipelineIndex(String cameraName, int index) {}

  /**
   * Sets crop window for one camera (Orbit2: dynamic crop around target).
   *
   * @param cameraName NetworkTables name of the Limelight
   * @param xMin xMin normalized 0-1
   * @param xMax xMax normalized 0-1
   * @param yMin yMin normalized 0-1
   * @param yMax yMax normalized 0-1
   */
  default void setCropWindow(
      String cameraName, double xMin, double xMax, double yMin, double yMax) {}
}
