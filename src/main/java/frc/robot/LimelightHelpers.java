// LimelightHelpers v1.14 (REQUIRES LLOS 2026.0 OR LATER)

package frc.robot;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * LimelightHelpers v1.14 provides static methods and classes for interfacing with Limelight vision
 * cameras in FRC.
 *
 * <p>This library supports all Limelight features including AprilTag tracking, Neural Networks, and
 * standard color/retroreflective tracking.
 *
 * <p>REQUIRES LLOS 2026.0 OR LATER
 */
public class LimelightHelpers {

  private static final Map<String, DoubleArrayEntry> doubleArrayEntries = new ConcurrentHashMap<>();
  private static ObjectMapper mapper;
  static boolean profileJSON = false;

  // ============================================================
  // TARGET RESULT CLASSES
  // ============================================================

  /** Represents a Color/Retroreflective Target Result extracted from JSON Output */
  public static class LimelightTarget_Retro {
    @JsonProperty("t6c_ts")
    private double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    private double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    private double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    private double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    private double[] targetPose_RobotSpace;

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("typ")
    public double ty_pixels;

    @JsonProperty("tx_nocross")
    public double tx_nocrosshair;

    @JsonProperty("ty_nocross")
    public double ty_nocrosshair;

    @JsonProperty("ts")
    public double ts;

    public LimelightTarget_Retro() {
      cameraPose_TargetSpace = new double[6];
      robotPose_FieldSpace = new double[6];
      robotPose_TargetSpace = new double[6];
      targetPose_CameraSpace = new double[6];
      targetPose_RobotSpace = new double[6];
    }

    public Pose3d getCameraPose_TargetSpace() {
      return toPose3D(cameraPose_TargetSpace);
    }

    public Pose3d getRobotPose_FieldSpace() {
      return toPose3D(robotPose_FieldSpace);
    }

    public Pose3d getRobotPose_TargetSpace() {
      return toPose3D(robotPose_TargetSpace);
    }

    public Pose3d getTargetPose_CameraSpace() {
      return toPose3D(targetPose_CameraSpace);
    }

    public Pose3d getTargetPose_RobotSpace() {
      return toPose3D(targetPose_RobotSpace);
    }

    public Pose2d getCameraPose_TargetSpace2D() {
      return toPose2D(cameraPose_TargetSpace);
    }

    public Pose2d getRobotPose_FieldSpace2D() {
      return toPose2D(robotPose_FieldSpace);
    }

    public Pose2d getRobotPose_TargetSpace2D() {
      return toPose2D(robotPose_TargetSpace);
    }

    public Pose2d getTargetPose_CameraSpace2D() {
      return toPose2D(targetPose_CameraSpace);
    }

    public Pose2d getTargetPose_RobotSpace2D() {
      return toPose2D(targetPose_RobotSpace);
    }
  }

  /** Represents an AprilTag/Fiducial Target Result extracted from JSON Output */
  public static class LimelightTarget_Fiducial {
    @JsonProperty("fID")
    public double fiducialID;

    @JsonProperty("fam")
    public String fiducialFamily;

    @JsonProperty("t6c_ts")
    private double[] cameraPose_TargetSpace;

    @JsonProperty("t6r_fs")
    private double[] robotPose_FieldSpace;

    @JsonProperty("t6r_ts")
    private double[] robotPose_TargetSpace;

    @JsonProperty("t6t_cs")
    private double[] targetPose_CameraSpace;

    @JsonProperty("t6t_rs")
    private double[] targetPose_RobotSpace;

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("typ")
    public double ty_pixels;

    @JsonProperty("tx_nocross")
    public double tx_nocrosshair;

    @JsonProperty("ty_nocross")
    public double ty_nocrosshair;

    @JsonProperty("ts")
    public double ts;

    public LimelightTarget_Fiducial() {
      cameraPose_TargetSpace = new double[6];
      robotPose_FieldSpace = new double[6];
      robotPose_TargetSpace = new double[6];
      targetPose_CameraSpace = new double[6];
      targetPose_RobotSpace = new double[6];
    }

    public Pose3d getCameraPose_TargetSpace() {
      return toPose3D(cameraPose_TargetSpace);
    }

    public Pose3d getRobotPose_FieldSpace() {
      return toPose3D(robotPose_FieldSpace);
    }

    public Pose3d getRobotPose_TargetSpace() {
      return toPose3D(robotPose_TargetSpace);
    }

    public Pose3d getTargetPose_CameraSpace() {
      return toPose3D(targetPose_CameraSpace);
    }

    public Pose3d getTargetPose_RobotSpace() {
      return toPose3D(targetPose_RobotSpace);
    }

    public Pose2d getCameraPose_TargetSpace2D() {
      return toPose2D(cameraPose_TargetSpace);
    }

    public Pose2d getRobotPose_FieldSpace2D() {
      return toPose2D(robotPose_FieldSpace);
    }

    public Pose2d getRobotPose_TargetSpace2D() {
      return toPose2D(robotPose_TargetSpace);
    }

    public Pose2d getTargetPose_CameraSpace2D() {
      return toPose2D(targetPose_CameraSpace);
    }

    public Pose2d getTargetPose_RobotSpace2D() {
      return toPose2D(targetPose_RobotSpace);
    }
  }

  /** Represents a Barcode Target Result extracted from JSON Output */
  public static class LimelightTarget_Barcode {
    @JsonProperty("fam")
    public String family;

    @JsonProperty("data")
    public String data;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("typ")
    public double ty_pixels;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("tx_nocross")
    public double tx_nocrosshair;

    @JsonProperty("ty_nocross")
    public double ty_nocrosshair;

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("pts")
    public double[][] corners;

    public String getFamily() {
      return family;
    }
  }

  /** Represents a Neural Classifier Pipeline Result extracted from JSON Output */
  public static class LimelightTarget_Classifier {
    @JsonProperty("class")
    public String className;

    @JsonProperty("classID")
    public double classID;

    @JsonProperty("conf")
    public double confidence;

    @JsonProperty("zone")
    public double zone;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("typ")
    public double ty_pixels;
  }

  /** Represents a Neural Detector Pipeline Result extracted from JSON Output */
  public static class LimelightTarget_Detector {
    @JsonProperty("class")
    public String className;

    @JsonProperty("classID")
    public double classID;

    @JsonProperty("conf")
    public double confidence;

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("txp")
    public double tx_pixels;

    @JsonProperty("typ")
    public double ty_pixels;

    @JsonProperty("tx_nocross")
    public double tx_nocrosshair;

    @JsonProperty("ty_nocross")
    public double ty_nocrosshair;
  }

  /** Represents hardware statistics from the Limelight. */
  public static class HardwareReport {
    @JsonProperty("cid")
    public String cameraId;

    @JsonProperty("cpu")
    public double cpuUsage;

    @JsonProperty("dfree")
    public double diskFree;

    @JsonProperty("dtot")
    public double diskTotal;

    @JsonProperty("ram")
    public double ramUsage;

    @JsonProperty("temp")
    public double temperature;
  }

  /** Represents IMU data from the JSON results. */
  public static class IMUResults {
    @JsonProperty("data")
    public double[] data;

    @JsonProperty("quat")
    public double[] quaternion;

    @JsonProperty("yaw")
    public double yaw;

    public double robotYaw, roll, pitch, rawYaw, gyroZ, gyroX, gyroY, accelZ, accelX, accelY;

    public IMUResults() {
      data = new double[0];
      quaternion = new double[4];
    }

    public void parseDataArray() {
      if (data != null && data.length >= 10) {
        robotYaw = data[0];
        roll = data[1];
        pitch = data[2];
        rawYaw = data[3];
        gyroZ = data[4];
        gyroX = data[5];
        gyroY = data[6];
        accelZ = data[7];
        accelX = data[8];
        accelY = data[9];
      }
    }
  }

  /** Represents capture rewind buffer statistics. */
  public static class RewindStats {
    @JsonProperty("bufferUsage")
    public double bufferUsage;

    @JsonProperty("enabled")
    public int enabled;

    @JsonProperty("flushing")
    public int flushing;

    @JsonProperty("frameCount")
    public int frameCount;

    @JsonProperty("latpen")
    public int latencyPenalty;

    @JsonProperty("storedSeconds")
    public double storedSeconds;
  }

  /** Limelight Results object, parsed from a Limelight's JSON results output. */
  public static class LimelightResults {
    public String error;

    @JsonProperty("pID")
    public double pipelineID;

    @JsonProperty("tl")
    public double latency_pipeline;

    @JsonProperty("cl")
    public double latency_capture;

    public double latency_jsonParse;

    @JsonProperty("ts")
    public double timestamp_LIMELIGHT_publish;

    @JsonProperty("ts_rio")
    public double timestamp_RIOFPGA_capture;

    @JsonProperty("ts_nt")
    public long timestamp_nt;

    @JsonProperty("ts_sys")
    public long timestamp_sys;

    @JsonProperty("ts_us")
    public long timestamp_us;

    @JsonProperty("v")
    @JsonFormat(shape = Shape.NUMBER)
    public boolean valid;

    @JsonProperty("pTYPE")
    public String pipelineType;

    @JsonProperty("tx")
    public double tx;

    @JsonProperty("ty")
    public double ty;

    @JsonProperty("txnc")
    public double tx_nocrosshair;

    @JsonProperty("tync")
    public double ty_nocrosshair;

    @JsonProperty("ta")
    public double ta;

    @JsonProperty("botpose")
    public double[] botpose;

    @JsonProperty("botpose_wpired")
    public double[] botpose_wpired;

    @JsonProperty("botpose_wpiblue")
    public double[] botpose_wpiblue;

    @JsonProperty("botpose_tagcount")
    public double botpose_tagcount;

    @JsonProperty("botpose_span")
    public double botpose_span;

    @JsonProperty("botpose_avgdist")
    public double botpose_avgdist;

    @JsonProperty("botpose_avgarea")
    public double botpose_avgarea;

    @JsonProperty("botpose_orb")
    public double[] botpose_orb;

    @JsonProperty("botpose_orb_wpiblue")
    public double[] botpose_orb_wpiblue;

    @JsonProperty("botpose_orb_wpired")
    public double[] botpose_orb_wpired;

    @JsonProperty("t6c_rs")
    public double[] camerapose_robotspace;

    @JsonProperty("hw")
    public HardwareReport hardware;

    @JsonProperty("imu")
    public IMUResults imuResults;

    @JsonProperty("rewind")
    public RewindStats rewindStats;

    @JsonProperty("PythonOut")
    public double[] pythonOutput;

    @JsonProperty("Retro")
    public LimelightTarget_Retro[] targets_Retro;

    @JsonProperty("Fiducial")
    public LimelightTarget_Fiducial[] targets_Fiducials;

    @JsonProperty("Classifier")
    public LimelightTarget_Classifier[] targets_Classifier;

    @JsonProperty("Detector")
    public LimelightTarget_Detector[] targets_Detector;

    @JsonProperty("Barcode")
    public LimelightTarget_Barcode[] targets_Barcode;

    public Pose3d getBotPose3d() {
      return toPose3D(botpose);
    }

    public Pose3d getBotPose3d_wpiRed() {
      return toPose3D(botpose_wpired);
    }

    public Pose3d getBotPose3d_wpiBlue() {
      return toPose3D(botpose_wpiblue);
    }

    public Pose2d getBotPose2d() {
      return toPose2D(botpose);
    }

    public Pose2d getBotPose2d_wpiRed() {
      return toPose2D(botpose_wpired);
    }

    public Pose2d getBotPose2d_wpiBlue() {
      return toPose2D(botpose_wpiblue);
    }

    public LimelightResults() {
      botpose = new double[6];
      botpose_wpired = new double[6];
      botpose_wpiblue = new double[6];
      botpose_orb = new double[6];
      botpose_orb_wpiblue = new double[6];
      botpose_orb_wpired = new double[6];
      camerapose_robotspace = new double[6];
      targets_Retro = new LimelightTarget_Retro[0];
      targets_Fiducials = new LimelightTarget_Fiducial[0];
      targets_Classifier = new LimelightTarget_Classifier[0];
      targets_Detector = new LimelightTarget_Detector[0];
      targets_Barcode = new LimelightTarget_Barcode[0];
      pythonOutput = new double[0];
      pipelineType = "";
    }
  }

  // ============================================================
  // RAW DATA CLASSES
  // ============================================================

  /** Represents a Limelight Raw Fiducial result from NetworkTables. */
  public static class RawFiducial {
    public int id;
    public double txnc, tync, ta, distToCamera, distToRobot, ambiguity;

    public RawFiducial(
        int id,
        double txnc,
        double tync,
        double ta,
        double distToCamera,
        double distToRobot,
        double ambiguity) {
      this.id = id;
      this.txnc = txnc;
      this.tync = tync;
      this.ta = ta;
      this.distToCamera = distToCamera;
      this.distToRobot = distToRobot;
      this.ambiguity = ambiguity;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) return true;
      if (obj == null || getClass() != obj.getClass()) return false;
      RawFiducial other = (RawFiducial) obj;
      return id == other.id
          && Double.compare(txnc, other.txnc) == 0
          && Double.compare(tync, other.tync) == 0
          && Double.compare(ta, other.ta) == 0
          && Double.compare(distToCamera, other.distToCamera) == 0
          && Double.compare(distToRobot, other.distToRobot) == 0
          && Double.compare(ambiguity, other.ambiguity) == 0;
    }
  }

  /** Represents a Limelight Raw Target/Contour result from NetworkTables. */
  public static class RawTarget {
    public double txnc, tync, ta;

    public RawTarget(double txnc, double tync, double ta) {
      this.txnc = txnc;
      this.tync = tync;
      this.ta = ta;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) return true;
      if (obj == null || getClass() != obj.getClass()) return false;
      RawTarget other = (RawTarget) obj;
      return Double.compare(txnc, other.txnc) == 0
          && Double.compare(tync, other.tync) == 0
          && Double.compare(ta, other.ta) == 0;
    }
  }

  /** Represents a Limelight Raw Neural Detector result from NetworkTables. */
  public static class RawDetection {
    public int classId;
    public double txnc, tync, ta;
    public double corner0_X, corner0_Y, corner1_X, corner1_Y;
    public double corner2_X, corner2_Y, corner3_X, corner3_Y;

    public RawDetection(
        int classId,
        double txnc,
        double tync,
        double ta,
        double c0x,
        double c0y,
        double c1x,
        double c1y,
        double c2x,
        double c2y,
        double c3x,
        double c3y) {
      this.classId = classId;
      this.txnc = txnc;
      this.tync = tync;
      this.ta = ta;
      corner0_X = c0x;
      corner0_Y = c0y;
      corner1_X = c1x;
      corner1_Y = c1y;
      corner2_X = c2x;
      corner2_Y = c2y;
      corner3_X = c3x;
      corner3_Y = c3y;
    }
  }

  /** Represents a 3D Pose Estimate. */
  public static class PoseEstimate {
    public Pose2d pose;
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;
    public RawFiducial[] rawFiducials;
    public boolean isMegaTag2;

    public PoseEstimate() {
      this.pose = new Pose2d();
      this.timestampSeconds = 0;
      this.latency = 0;
      this.tagCount = 0;
      this.tagSpan = 0;
      this.avgTagDist = 0;
      this.avgTagArea = 0;
      this.rawFiducials = new RawFiducial[] {};
      this.isMegaTag2 = false;
    }

    public PoseEstimate(
        Pose2d pose,
        double timestampSeconds,
        double latency,
        int tagCount,
        double tagSpan,
        double avgTagDist,
        double avgTagArea,
        RawFiducial[] rawFiducials,
        boolean isMegaTag2) {
      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.latency = latency;
      this.tagCount = tagCount;
      this.tagSpan = tagSpan;
      this.avgTagDist = avgTagDist;
      this.avgTagArea = avgTagArea;
      this.rawFiducials = rawFiducials;
      this.isMegaTag2 = isMegaTag2;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) return true;
      if (obj == null || getClass() != obj.getClass()) return false;
      PoseEstimate that = (PoseEstimate) obj;
      return Double.compare(that.latency, latency) == 0
          && tagCount == that.tagCount
          && Double.compare(that.tagSpan, tagSpan) == 0
          && Double.compare(that.avgTagDist, avgTagDist) == 0
          && Double.compare(that.avgTagArea, avgTagArea) == 0
          && pose.equals(that.pose)
          && Arrays.equals(rawFiducials, that.rawFiducials);
    }
  }

  /** Encapsulates the state of an internal Limelight IMU. */
  public static class IMUData {
    public double robotYaw, Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

    public IMUData() {}

    public IMUData(double[] imuData) {
      if (imuData != null && imuData.length >= 10) {
        robotYaw = imuData[0];
        Roll = imuData[1];
        Pitch = imuData[2];
        Yaw = imuData[3];
        gyroX = imuData[4];
        gyroY = imuData[5];
        gyroZ = imuData[6];
        accelX = imuData[7];
        accelY = imuData[8];
        accelZ = imuData[9];
      }
    }
  }

  // ============================================================
  // UTILITY METHODS
  // ============================================================

  static final String sanitizeName(String name) {
    if ("".equals(name) || name == null) return "limelight";
    return name;
  }

  public static Pose3d toPose3D(double[] inData) {
    if (inData.length < 6) return new Pose3d();
    return new Pose3d(
        new Translation3d(inData[0], inData[1], inData[2]),
        new Rotation3d(
            Units.degreesToRadians(inData[3]),
            Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5])));
  }

  public static Pose2d toPose2D(double[] inData) {
    if (inData.length < 6) return new Pose2d();
    return new Pose2d(
        new Translation2d(inData[0], inData[1]), new Rotation2d(Units.degreesToRadians(inData[5])));
  }

  public static double[] pose3dToArray(Pose3d pose) {
    return new double[] {
      pose.getTranslation().getX(),
      pose.getTranslation().getY(),
      pose.getTranslation().getZ(),
      Units.radiansToDegrees(pose.getRotation().getX()),
      Units.radiansToDegrees(pose.getRotation().getY()),
      Units.radiansToDegrees(pose.getRotation().getZ())
    };
  }

  public static double[] pose2dToArray(Pose2d pose) {
    return new double[] {
      pose.getTranslation().getX(),
      pose.getTranslation().getY(),
      0,
      0,
      0,
      Units.radiansToDegrees(pose.getRotation().getRadians())
    };
  }

  private static double extractArrayEntry(double[] inData, int position) {
    return inData.length < position + 1 ? 0 : inData[position];
  }

  // ============================================================
  // NETWORKTABLES METHODS
  // ============================================================

  public static NetworkTable getLimelightNTTable(String tableName) {
    return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName));
  }

  public static void Flush() {
    NetworkTableInstance.getDefault().flush();
  }

  public static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
    return getLimelightNTTable(tableName).getEntry(entryName);
  }

  public static DoubleArrayEntry getLimelightDoubleArrayEntry(String tableName, String entryName) {
    String key = tableName + "/" + entryName;
    return doubleArrayEntries.computeIfAbsent(
        key,
        k -> getLimelightNTTable(tableName).getDoubleArrayTopic(entryName).getEntry(new double[0]));
  }

  public static double getLimelightNTDouble(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
  }

  public static void setLimelightNTDouble(String tableName, String entryName, double val) {
    getLimelightNTTableEntry(tableName, entryName).setDouble(val);
  }

  public static void setLimelightNTDoubleArray(String tableName, String entryName, double[] val) {
    getLimelightNTTableEntry(tableName, entryName).setDoubleArray(val);
  }

  public static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
  }

  public static String getLimelightNTString(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getString("");
  }

  public static String[] getLimelightNTStringArray(String tableName, String entryName) {
    return getLimelightNTTableEntry(tableName, entryName).getStringArray(new String[0]);
  }

  // ============================================================
  // TARGET DATA GETTERS
  // ============================================================

  public static boolean getTV(String limelightName) {
    return 1.0 == getLimelightNTDouble(limelightName, "tv");
  }

  public static double getTX(String limelightName) {
    return getLimelightNTDouble(limelightName, "tx");
  }

  public static double getTY(String limelightName) {
    return getLimelightNTDouble(limelightName, "ty");
  }

  public static double getTXNC(String limelightName) {
    return getLimelightNTDouble(limelightName, "txnc");
  }

  public static double getTYNC(String limelightName) {
    return getLimelightNTDouble(limelightName, "tync");
  }

  public static double getTA(String limelightName) {
    return getLimelightNTDouble(limelightName, "ta");
  }

  public static double[] getT2DArray(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "t2d");
  }

  public static int getTargetCount(String limelightName) {
    double[] t2d = getT2DArray(limelightName);
    return t2d.length == 17 ? (int) t2d[1] : 0;
  }

  public static double getLatency_Pipeline(String limelightName) {
    return getLimelightNTDouble(limelightName, "tl");
  }

  public static double getLatency_Capture(String limelightName) {
    return getLimelightNTDouble(limelightName, "cl");
  }

  public static double getCurrentPipelineIndex(String limelightName) {
    return getLimelightNTDouble(limelightName, "getpipe");
  }

  public static String getCurrentPipelineType(String limelightName) {
    return getLimelightNTString(limelightName, "getpipetype");
  }

  public static double getFiducialID(String limelightName) {
    return getLimelightNTDouble(limelightName, "tid");
  }

  public static double getHeartbeat(String limelightName) {
    return getLimelightNTDouble(limelightName, "hb");
  }

  // ============================================================
  // POSE GETTERS
  // ============================================================

  public static double[] getBotPose(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose");
  }

  public static double[] getBotPose_wpiRed(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
  }

  public static double[] getBotPose_wpiBlue(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
  }

  public static Pose3d getBotPose3d(String limelightName) {
    return toPose3D(getBotPose(limelightName));
  }

  public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
    return toPose3D(getBotPose_wpiBlue(limelightName));
  }

  public static Pose3d getBotPose3d_wpiRed(String limelightName) {
    return toPose3D(getBotPose_wpiRed(limelightName));
  }

  public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
    return toPose2D(getBotPose_wpiBlue(limelightName));
  }

  public static Pose2d getBotPose2d_wpiRed(String limelightName) {
    return toPose2D(getBotPose_wpiRed(limelightName));
  }

  public static Pose2d getBotPose2d(String limelightName) {
    return toPose2D(getBotPose(limelightName));
  }

  // ============================================================
  // POSE ESTIMATE METHODS
  // ============================================================

  private static PoseEstimate getBotPoseEstimate(
      String limelightName, String entryName, boolean isMegaTag2) {
    DoubleArrayEntry poseEntry = getLimelightDoubleArrayEntry(limelightName, entryName);
    TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    double[] poseArray = tsValue.value;
    long timestamp = tsValue.timestamp;

    if (poseArray.length == 0) return new PoseEstimate();

    var pose = toPose2D(poseArray);
    double latency = extractArrayEntry(poseArray, 6);
    int tagCount = (int) extractArrayEntry(poseArray, 7);
    double tagSpan = extractArrayEntry(poseArray, 8);
    double tagDist = extractArrayEntry(poseArray, 9);
    double tagArea = extractArrayEntry(poseArray, 10);
    double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

    int valsPerFiducial = 7;
    int expectedTotalVals = 11 + valsPerFiducial * tagCount;
    RawFiducial[] rawFiducials;

    if (poseArray.length != expectedTotalVals) {
      rawFiducials = new RawFiducial[0];
    } else {
      rawFiducials = new RawFiducial[tagCount];
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        rawFiducials[i] =
            new RawFiducial(
                (int) poseArray[baseIndex],
                poseArray[baseIndex + 1],
                poseArray[baseIndex + 2],
                poseArray[baseIndex + 3],
                poseArray[baseIndex + 4],
                poseArray[baseIndex + 5],
                poseArray[baseIndex + 6]);
      }
    }
    return new PoseEstimate(
        pose,
        adjustedTimestamp,
        latency,
        tagCount,
        tagSpan,
        tagDist,
        tagArea,
        rawFiducials,
        isMegaTag2);
  }

  /** Gets MegaTag1 Pose2d for WPILib Blue alliance coordinate system. */
  public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
    return getBotPoseEstimate(limelightName, "botpose_wpiblue", false);
  }

  /** Gets MegaTag2 Pose2d for WPILib Blue alliance. Call setRobotOrientation() first. */
  public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
    return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true);
  }

  /** Gets MegaTag1 Pose2d for WPILib Red alliance coordinate system. */
  public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
    return getBotPoseEstimate(limelightName, "botpose_wpired", false);
  }

  /** Gets MegaTag2 Pose2d for WPILib Red alliance. Call setRobotOrientation() first. */
  public static PoseEstimate getBotPoseEstimate_wpiRed_MegaTag2(String limelightName) {
    return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true);
  }

  public static Boolean validPoseEstimate(PoseEstimate pose) {
    return pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0;
  }

  // ============================================================
  // RAW DATA GETTERS
  // ============================================================

  public static RawFiducial[] getRawFiducials(String limelightName) {
    var entry = getLimelightNTTableEntry(limelightName, "rawfiducials");
    var rawFiducialArray = entry.getDoubleArray(new double[0]);
    int valsPerEntry = 7;
    if (rawFiducialArray.length % valsPerEntry != 0) return new RawFiducial[0];

    int numFiducials = rawFiducialArray.length / valsPerEntry;
    RawFiducial[] rawFiducials = new RawFiducial[numFiducials];
    for (int i = 0; i < numFiducials; i++) {
      int baseIndex = i * valsPerEntry;
      rawFiducials[i] =
          new RawFiducial(
              (int) extractArrayEntry(rawFiducialArray, baseIndex),
              extractArrayEntry(rawFiducialArray, baseIndex + 1),
              extractArrayEntry(rawFiducialArray, baseIndex + 2),
              extractArrayEntry(rawFiducialArray, baseIndex + 3),
              extractArrayEntry(rawFiducialArray, baseIndex + 4),
              extractArrayEntry(rawFiducialArray, baseIndex + 5),
              extractArrayEntry(rawFiducialArray, baseIndex + 6));
    }
    return rawFiducials;
  }

  public static RawDetection[] getRawDetections(String limelightName) {
    var entry = getLimelightNTTableEntry(limelightName, "rawdetections");
    var rawDetectionArray = entry.getDoubleArray(new double[0]);
    int valsPerEntry = 12;
    if (rawDetectionArray.length % valsPerEntry != 0) return new RawDetection[0];

    int numDetections = rawDetectionArray.length / valsPerEntry;
    RawDetection[] rawDetections = new RawDetection[numDetections];
    for (int i = 0; i < numDetections; i++) {
      int b = i * valsPerEntry;
      rawDetections[i] =
          new RawDetection(
              (int) extractArrayEntry(rawDetectionArray, b),
              extractArrayEntry(rawDetectionArray, b + 1),
              extractArrayEntry(rawDetectionArray, b + 2),
              extractArrayEntry(rawDetectionArray, b + 3),
              extractArrayEntry(rawDetectionArray, b + 4),
              extractArrayEntry(rawDetectionArray, b + 5),
              extractArrayEntry(rawDetectionArray, b + 6),
              extractArrayEntry(rawDetectionArray, b + 7),
              extractArrayEntry(rawDetectionArray, b + 8),
              extractArrayEntry(rawDetectionArray, b + 9),
              extractArrayEntry(rawDetectionArray, b + 10),
              extractArrayEntry(rawDetectionArray, b + 11));
    }
    return rawDetections;
  }

  public static RawTarget[] getRawTargets(String limelightName) {
    var entry = getLimelightNTTableEntry(limelightName, "rawtargets");
    var rawTargetArray = entry.getDoubleArray(new double[0]);
    int valsPerEntry = 3;
    if (rawTargetArray.length % valsPerEntry != 0) return new RawTarget[0];

    int numTargets = rawTargetArray.length / valsPerEntry;
    RawTarget[] rawTargets = new RawTarget[numTargets];
    for (int i = 0; i < numTargets; i++) {
      int b = i * valsPerEntry;
      rawTargets[i] =
          new RawTarget(
              extractArrayEntry(rawTargetArray, b),
              extractArrayEntry(rawTargetArray, b + 1),
              extractArrayEntry(rawTargetArray, b + 2));
    }
    return rawTargets;
  }

  public static double[] getCornerCoordinates(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "tcornxy");
  }

  public static IMUData getIMUData(String limelightName) {
    double[] imuData = getLimelightNTDoubleArray(limelightName, "imu");
    if (imuData == null || imuData.length < 10) return new IMUData();
    return new IMUData(imuData);
  }

  // ============================================================
  // SETTERS
  // ============================================================

  public static void setPipelineIndex(String limelightName, int pipelineIndex) {
    setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
  }

  public static void setPriorityTagID(String limelightName, int ID) {
    setLimelightNTDouble(limelightName, "priorityid", ID);
  }

  public static void setLEDMode_PipelineControl(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 0);
  }

  public static void setLEDMode_ForceOff(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 1);
  }

  public static void setLEDMode_ForceBlink(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 2);
  }

  public static void setLEDMode_ForceOn(String limelightName) {
    setLimelightNTDouble(limelightName, "ledMode", 3);
  }

  public static void setStreamMode_Standard(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 0);
  }

  public static void setStreamMode_PiPMain(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 1);
  }

  public static void setStreamMode_PiPSecondary(String limelightName) {
    setLimelightNTDouble(limelightName, "stream", 2);
  }

  public static void setCropWindow(String ll, double xMin, double xMax, double yMin, double yMax) {
    setLimelightNTDoubleArray(ll, "crop", new double[] {xMin, xMax, yMin, yMax});
  }

  public static void setKeystone(String limelightName, double horizontal, double vertical) {
    setLimelightNTDoubleArray(limelightName, "keystone_set", new double[] {horizontal, vertical});
  }

  public static void setFiducial3DOffset(String ll, double x, double y, double z) {
    setLimelightNTDoubleArray(ll, "fiducial_offset_set", new double[] {x, y, z});
  }

  /** Sets robot orientation for MegaTag2 localization. */
  public static void SetRobotOrientation(
      String ll,
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    setLimelightNTDoubleArray(
        ll, "robot_orientation_set", new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate});
    Flush();
  }

  public static void SetRobotOrientation_NoFlush(
      String ll,
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    setLimelightNTDoubleArray(
        ll, "robot_orientation_set", new double[] {yaw, yawRate, pitch, pitchRate, roll, rollRate});
  }

  public static void SetIMUMode(String limelightName, int mode) {
    setLimelightNTDouble(limelightName, "imumode_set", mode);
  }

  public static void SetIMUAssistAlpha(String limelightName, double alpha) {
    setLimelightNTDouble(limelightName, "imuassistalpha_set", alpha);
  }

  public static void SetThrottle(String limelightName, int throttle) {
    setLimelightNTDouble(limelightName, "throttle_set", throttle);
  }

  public static void SetFiducialIDFiltersOverride(String limelightName, int[] validIDs) {
    double[] validIDsDouble = new double[validIDs.length];
    for (int i = 0; i < validIDs.length; i++) validIDsDouble[i] = validIDs[i];
    setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble);
  }

  public static void SetFiducialDownscalingOverride(String limelightName, float downscale) {
    int d = 0;
    if (downscale == 1.0) d = 1;
    if (downscale == 1.5) d = 2;
    if (downscale == 2) d = 3;
    if (downscale == 3) d = 4;
    if (downscale == 4) d = 5;
    setLimelightNTDouble(limelightName, "fiducial_downscale_set", d);
  }

  public static void setCameraPose_RobotSpace(
      String ll, double fw, double side, double up, double roll, double pitch, double yaw) {
    setLimelightNTDoubleArray(
        ll, "camerapose_robotspace_set", new double[] {fw, side, up, roll, pitch, yaw});
  }

  public static void setPythonScriptData(String limelightName, double[] outgoingPythonData) {
    setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData);
  }

  public static double[] getPythonScriptData(String limelightName) {
    return getLimelightNTDoubleArray(limelightName, "llpython");
  }

  public static void triggerSnapshot(String limelightName) {
    double current = getLimelightNTDouble(limelightName, "snapshot");
    setLimelightNTDouble(limelightName, "snapshot", current + 1);
  }

  public static void setRewindEnabled(String limelightName, boolean enabled) {
    setLimelightNTDouble(limelightName, "rewind_enable_set", enabled ? 1 : 0);
  }

  public static void triggerRewindCapture(String limelightName, double durationSeconds) {
    double[] currentArray = getLimelightNTDoubleArray(limelightName, "capture_rewind");
    double counter = (currentArray.length > 0) ? currentArray[0] : 0;
    setLimelightNTDoubleArray(
        limelightName,
        "capture_rewind",
        new double[] {counter + 1, Math.min(durationSeconds, 165)});
  }

  public static String getJSONDump(String limelightName) {
    return getLimelightNTString(limelightName, "json");
  }

  public static LimelightResults getLatestResults(String limelightName) {
    long start = System.nanoTime();
    LimelightResults results = new LimelightResults();
    if (mapper == null) {
      mapper =
          new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    }
    try {
      String jsonString = getJSONDump(limelightName);
      if (jsonString == null || jsonString.isEmpty() || jsonString.isBlank()) {
        results.error = "lljson error: empty json";
      } else {
        results = mapper.readValue(jsonString, LimelightResults.class);
        if (results.imuResults != null) results.imuResults.parseDataArray();
      }
    } catch (Exception e) {
      results.error = "lljson error: " + e.getMessage();
    }
    results.latency_jsonParse = (System.nanoTime() - start) * .000001;
    return results;
  }

  public static void setupPortForwardingUSB(int usbIndex) {
    String ip = "172.29." + usbIndex + ".1";
    int basePort = 5800 + (usbIndex * 10);
    for (int i = 0; i < 10; i++) PortForwarder.add(basePort + i, ip, 5800 + i);
  }
}
