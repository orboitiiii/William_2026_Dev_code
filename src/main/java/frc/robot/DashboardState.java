package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

/**
 * Global telemetry state container for NetworkTables publication.
 *
 * <p>This class aggregates robot telemetry into a single struct for efficient NT4 transmission.
 * Using a struct reduces overhead compared to multiple individual topic publishes.
 *
 * <p><strong>Protocol</strong>: NT4 Struct serialization with manual pack/unpack for maximum
 * control over the binary format. This approach is faster than reflection-based serialization.
 *
 * <p><strong>Usage</strong>:
 *
 * <pre>{@code
 * DashboardState.getInstance().robotPose = drive.getPose();
 * DashboardState.getInstance().publish();
 * }</pre>
 *
 * <p><strong>Extension</strong>: To add new fields, update both the public field declaration AND
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
   * Game-specific data from FMS indicating which alliance's goal goes inactive first.
   *
   * <p>Values: 0 = unknown/not yet received, 1 = Red ('R'), 2 = Blue ('B').
   *
   * <p>Data becomes available ~3 seconds after Auto ends. Before that, value is 0.
   */
  public byte gameData = 0;

  /**
   * True if the robot is on the Red Alliance, false otherwise (Blue).
   *
   * <p>Derived from DriverStation.getAlliance(). Defaults to false (Blue) if unknown.
   */
  public boolean isRedAlliance = false;

  // --- Subsystem Health Status (Updated by HealthCheckLooper) ---
  // Value: 0 = Error (any check failed), 1 = Healthy (all checks pass)

  /** Drive subsystem health status. */
  public byte driveHealth = 1;

  /** IntakeWheel subsystem health status. */
  public byte intakeHealth = 1;

  // ==========================================

  private final StructPublisher<DashboardState> mPublisher;

  private DashboardState() {
    mPublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("/SmartDashboard/DashboardState", DashboardState.struct)
            .publish();
  }

  /**
   * Publishes the current state to NetworkTables.
   *
   * <p>Call this once per robot periodic loop after updating all fields.
   */
  public void publish() {
    mPublisher.set(this);
  }

  // --- Manual Struct Implementation ---

  /** The struct serializer for DashboardState. */
  public static final Struct<DashboardState> struct = new ManualStruct();

  /**
   * Manual struct implementation for DashboardState.
   *
   * <p>Implements NT4 binary serialization without reflection for performance. The schema string
   * must match the pack/unpack order exactly.
   */
  private static class ManualStruct implements Struct<DashboardState> {
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

    /**
     * Returns the serialized size in bytes.
     *
     * <p>Layout: matchTime(8) + robotPose(24) + gameData(1) + isRedAlliance(1) + driveHealth(1) +
     * intakeHealth(1) = 36 bytes.
     */
    @Override
    public int getSize() {
      return 8 + Pose2d.struct.getSize() + 1 + 1 + 1 + 1;
    }

    /**
     * Returns the schema string for dashboard clients.
     *
     * <p>Must match pack/unpack order for correct deserialization.
     */
    @Override
    public String getSchema() {
      return "double matchTime; Pose2d robotPose; uint8 gameData; bool isRedAlliance; uint8 driveHealth; uint8 intakeHealth";
    }

    @Override
    public DashboardState unpack(ByteBuffer bb) {
      DashboardState state = new DashboardState();
      state.matchTime = bb.getDouble();
      state.robotPose = Pose2d.struct.unpack(bb);
      state.gameData = bb.get();
      state.isRedAlliance = bb.get() != 0;
      state.driveHealth = bb.get();
      state.intakeHealth = bb.get();
      return state;
    }

    @Override
    public void pack(ByteBuffer bb, DashboardState value) {
      bb.putDouble(value.matchTime);
      Pose2d.struct.pack(bb, value.robotPose);
      bb.put(value.gameData);
      bb.put((byte) (value.isRedAlliance ? 1 : 0));
      bb.put(value.driveHealth);
      bb.put(value.intakeHealth);
    }
  }
}
