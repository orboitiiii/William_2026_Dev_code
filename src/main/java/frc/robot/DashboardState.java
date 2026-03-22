package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.struct.Struct;

/**
 * Global telemetry state container for NetworkTables publication.
 *
 * <p>
 * This class aggregates robot telemetry into a single struct for efficient NT4
 * transmission.
 * Using a struct reduces overhead compared to multiple individual topic
 * publishes.
 *
 * <p>
 * <strong>Protocol</strong>: NT4 Struct serialization with manual pack/unpack
 * for maximum
 * control over the binary format. This approach is faster than reflection-based
 * serialization.
 *
 * <p>
 * <strong>Usage</strong>:
 *
 * <pre>{@code
 * DashboardState.getInstance().robotPose = drive.getPose();
 * DashboardState.getInstance().publish();
 * }</pre>
 *
 * <p>
 * <strong>Extension</strong>: To add new fields, update both the public field
 * declaration AND
 * the ManualStruct inner class (pack, unpack, getSize, getSchema).
 */
public class DashboardState {
  private static DashboardState mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global DashboardState instance.
   */
  public static synchronized DashboardState getInstance() {
    if (mInstance == null) {
      mInstance = new DashboardState();
    }
    return mInstance;
  }

  // ==========================================
  // Telemetry Fields - Add new fields here
  // ==========================================

  /** Remaining match time in seconds. */
  public double matchTime = 0.0;

  /** Current estimated robot pose in field coordinates. */
  public Pose2d robotPose = new Pose2d();

  /**
   * Game-specific data from FMS indicating which alliance's goal goes inactive
   * first.
   *
   * <p>
   * Values: 0 = unknown/not yet received, 1 = Red ('R'), 2 = Blue ('B').
   */
  public byte gameData = 0;

  /** True if the robot is on the Red Alliance, false otherwise (Blue). */
  public boolean isRedAlliance = false;

  // --- Robot State & Mechanism Data ---
  /** Robot state string (e.g., "IDLE", "SHOOTING", "INTAKING"). */
  public String robotState = "IDLE";

  /** Climb state: 0=Stowed, 1=Extending, 2=Pre-Climb. */
  public int climbState = 0;

  /** Turret angle in degrees for display. */
  public double turretAngle = 0.0;

  /** Elevation (hood) angle in degrees for display. */
  public double elevationAngle = 0.0;

  // --- Alerts & EKF Trust Metrics ---
  /** Odometry drift from dead reckoning (meters). */
  public double odometryDrift = 0.0;

  /** True if the robot's pose history is stable without jumping. */
  public boolean poseStable = false;

  /** True if the EKF pose is trusted (valid and recent vision). */
  public boolean isTrusted = false;

  /** Continuous trust score for auto/auto-aim routines. */
  public double trustScore = 0.0;

  // --- Subsystem Health (Slot 10) ---
  public boolean turretOK = true;
  public boolean hoodOK = true;
  public boolean shooterOK = true;
  public boolean intakeWheelsOK = true;
  public boolean intakePivotOK = true;
  public boolean indexerOK = true;
  public boolean driveOK = true;
  public boolean frontLLOK = true;

  // ==========================================

  private final edu.wpi.first.networktables.StructPublisher<DashboardState> mPublisher;

  // Subscribers for Dashboard -> Robot topics
  private final edu.wpi.first.networktables.IntegerSubscriber mSelectedLevelSub;
  private final edu.wpi.first.networktables.BooleanSubscriber mShootOnMoveDisabledSub;
  private final edu.wpi.first.networktables.BooleanSubscriber mPointShootDisabledSub;
  private final edu.wpi.first.networktables.BooleanSubscriber mAutoPassingDisabledSub;
  private final edu.wpi.first.networktables.BooleanSubscriber mApriltagDisabledSub;
  private final edu.wpi.first.networktables.BooleanSubscriber mDriveProtectDisabledSub;
  private final edu.wpi.first.networktables.DoubleArraySubscriber mRobotFieldPoseSub;

  private DashboardState() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    mPublisher = inst.getStructTopic("/SmartDashboard/DashboardState", DashboardState.struct).publish();

    // Dashboard -> Robot Subscribers
    mSelectedLevelSub = inst.getIntegerTopic("/SmartDashboard/SelectedLevel").subscribe(1);
    mShootOnMoveDisabledSub = inst.getBooleanTopic("/SmartDashboard/ShootOnMoveDisabled").subscribe(false);
    mPointShootDisabledSub = inst.getBooleanTopic("/SmartDashboard/PointShootDisabled").subscribe(false);
    mAutoPassingDisabledSub = inst.getBooleanTopic("/SmartDashboard/AutoPassingDisabled").subscribe(false);
    mApriltagDisabledSub = inst.getBooleanTopic("/SmartDashboard/ApriltagDisabled").subscribe(false);
    mDriveProtectDisabledSub = inst.getBooleanTopic("/SmartDashboard/DriveProtectDisabled").subscribe(false);
    mRobotFieldPoseSub = inst.getDoubleArrayTopic("/SmartDashboard/Field/Robot")
        .subscribe(new double[] { 0.0, 0.0, 0.0 });
  }

  /**
   * Publishes the current state to NetworkTables as a single struct.
   *
   * <p>
   * Call this once per robot periodic loop after updating all fields.
   */
  public void publish() {
    this.robotState = GlobalData.robotState.name();
    mPublisher.set(this);
  }

  // --- Subscriber Getters ---

  public int getSelectedLevel() {
    return (int) mSelectedLevelSub.get();
  }

  public boolean isShootOnMoveDisabled() {
    // return mShootOnMoveDisabledSub.get();
    return true;
  }

  public boolean isPointShootDisabled() {
    return mPointShootDisabledSub.get();
  }

  public boolean isAutoPassingDisabled() {
    return mAutoPassingDisabledSub.get();
  }

  public boolean isApriltagDisabled() {
    return mApriltagDisabledSub.get();
  }

  public boolean isDriveProtectDisabled() {
    return mDriveProtectDisabledSub.get();
  }

  /** Gets the robot pose set by the dashboard (e.g. for auto preview/init). */
  // public double[] getDashboardRobotPose() {
  // return mRobotFieldPoseSub.get();
  // }

  // --- Manual Struct Implementation ---

  /** The struct serializer for DashboardState. */
  public static final Struct<DashboardState> struct = new ManualStruct();

  /**
   * Manual struct implementation for DashboardState.
   *
   * <p>
   * Implements NT4 binary serialization without reflection for performance. The
   * schema string
   * must match the pack/unpack order exactly.
   *
   * <p>
   * <strong>Schema Layout</strong> (total ~70 bytes):
   *
   * <ul>
   * <li>matchTime (double, 8 bytes)
   * <li>robotPose (Pose2d, 24 bytes)
   * <li>gameData (uint8, 1 byte)
   * <li>isRedAlliance (bool, 1 byte)
   * <li>climbState (int32, 4 bytes)
   * <li>turretAngle (double, 8 bytes)
   * <li>elevationAngle (double, 8 bytes)
   * <li>robotSlipping (bool, 1 byte)
   * <li>robotImpacting (bool, 1 byte)
   * <li>odometryStale (bool, 1 byte)
   * <li>lastVisionTimestamp (double, 8 bytes)
   * <li>turretOK..frontLLOK (8 bools, 8 bytes)
   * </ul>
   *
   * <p>
   * <strong>Note</strong>: robotState (String) is NOT serialized via struct due
   * to variable
   * length. Use a separate StringPublisher if needed.
   */
  private static class ManualStruct implements Struct<DashboardState> {
    // Fixed size:
    // 8 (matchTime) + 24 (robotPose) + 1 (gameData) + 1 (isRedAlliance)
    // + 4 (climbState) + 8 (turretAngle) + 8 (elevationAngle)
    // + 8 (odometryDrift) + 1 (poseStable) + 1 (isTrusted) + 8 (trustScore)
    // + 16 (robotState, char[16])
    // + 8 (OK flags: turret, hood, shooter, intakeW, intakeP, indexer, drive,
    // frontLL)
    // Total = 8+24+1+1+4+8+8 + 8+1+1+8 + 16 + 8 = 96 bytes
    private static final int STRUCT_SIZE = 96;

    @Override
    public Class<DashboardState> getTypeClass() {
      return DashboardState.class;
    }

    @Override
    public String getTypeName() {
      return "DashboardState";
    }

    @Override
    public String getTypeString() {
      return "struct:DashboardState";
    }

    @Override
    public int getSize() {
      return STRUCT_SIZE;
    }

    @Override
    public String getSchema() {
      return "double matchTime; Pose2d robotPose; uint8 gameData; bool isRedAlliance; "
          + "int32 climbState; double turretAngle; double elevationAngle; "
          + "double odometryDrift; bool poseStable; bool isTrusted; double trustScore; "
          + "char[16] robotState; " // String represented as fixed-length char array
          + "bool turretOK; bool hoodOK; bool shooterOK; bool intakeWheelsOK; "
          + "bool intakePivotOK; bool indexerOK; bool driveOK; bool frontLLOK";
    }

    @Override
    public DashboardState unpack(ByteBuffer bb) {
      DashboardState state = new DashboardState();
      state.matchTime = bb.getDouble();
      state.robotPose = Pose2d.struct.unpack(bb);
      state.gameData = bb.get();
      state.isRedAlliance = bb.get() != 0;
      state.climbState = bb.getInt();
      state.turretAngle = bb.getDouble();
      state.elevationAngle = bb.getDouble();
      state.odometryDrift = bb.getDouble();
      state.poseStable = bb.get() != 0;
      state.isTrusted = bb.get() != 0;
      state.trustScore = bb.getDouble();

      // Unpack string (char[16])
      byte[] strBytes = new byte[16];
      bb.get(strBytes);
      state.robotState = new String(strBytes, java.nio.charset.StandardCharsets.US_ASCII).trim();

      state.turretOK = bb.get() != 0;
      state.hoodOK = bb.get() != 0;
      state.shooterOK = bb.get() != 0;
      state.intakeWheelsOK = bb.get() != 0;
      state.intakePivotOK = bb.get() != 0;
      state.indexerOK = bb.get() != 0;
      state.driveOK = bb.get() != 0;
      state.frontLLOK = bb.get() != 0;
      return state;
    }

    @Override
    public void pack(ByteBuffer bb, DashboardState value) {
      bb.putDouble(value.matchTime);
      Pose2d.struct.pack(bb, value.robotPose);
      bb.put(value.gameData);
      bb.put((byte) (value.isRedAlliance ? 1 : 0));
      bb.putInt(value.climbState);
      bb.putDouble(value.turretAngle);
      bb.putDouble(value.elevationAngle);
      bb.putDouble(value.odometryDrift);
      bb.put((byte) (value.poseStable ? 1 : 0));
      bb.put((byte) (value.isTrusted ? 1 : 0));
      bb.putDouble(value.trustScore);

      // Pack string into exactly 16 bytes (pad with space or truncate)
      byte[] strBytes = value.robotState.getBytes(java.nio.charset.StandardCharsets.US_ASCII);
      byte[] paddedBytes = new byte[16];
      System.arraycopy(strBytes, 0, paddedBytes, 0, Math.min(strBytes.length, 16));
      bb.put(paddedBytes);

      bb.put((byte) (value.turretOK ? 1 : 0));
      bb.put((byte) (value.hoodOK ? 1 : 0));
      bb.put((byte) (value.shooterOK ? 1 : 0));
      bb.put((byte) (value.intakeWheelsOK ? 1 : 0));
      bb.put((byte) (value.intakePivotOK ? 1 : 0));
      bb.put((byte) (value.indexerOK ? 1 : 0));
      bb.put((byte) (value.driveOK ? 1 : 0));
      bb.put((byte) (value.frontLLOK ? 1 : 0));
    }
  }
}
