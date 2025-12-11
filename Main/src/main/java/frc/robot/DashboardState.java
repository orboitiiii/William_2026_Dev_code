package frc.robot;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;

/**
 * Global telemetry state holder. Replaces SmartDashboard.
 *
 * <p>Automagical Struct Generation: You only need to add public fields to this class. The inner
 * ReflectiveStruct will automatically discover, sort (alphabetically), and serialize them.
 * Supported types: double, boolean, int, float, long, Pose2d.
 */
public class DashboardState {
  private static DashboardState mInstance;

  public static synchronized DashboardState getInstance() {
    if (mInstance == null) {
      mInstance = new DashboardState();
    }
    return mInstance;
  }

  // ==========================================
  //  USER DATA AREA - JUST ADD FIELDS HERE
  // ==========================================
  public double driveHeadingDegrees = 0.0;
  public int driveMode = 0; // 0: Manual, 1: Auto
  public boolean isRedAlliance = false;
  public double looperDt = 0.0;
  public double matchTime = 0.0;
  public Pose2d robotPose = new Pose2d();
  // Add more fields here... e.g. public double shooterSpeed;
  // ==========================================

  private final StructPublisher<DashboardState> mPublisher;

  private DashboardState() {
    mPublisher =
        NetworkTableInstance.getDefault()
            .getStructTopic("DashboardState", DashboardState.struct)
            .publish();
  }

  public void publish() {
    mPublisher.set(this);
  }

  // --- Automatic Struct Implementation ---
  public static final Struct<DashboardState> struct = new ReflectiveStruct();

  private static class ReflectiveStruct implements Struct<DashboardState> {
    private final Field[] mFields;
    private final int mSize;
    private final String mSchema;

    public ReflectiveStruct() {
      // 1. Get all public fields
      Field[] allFields = DashboardState.class.getFields();
      List<Field> validFields = new ArrayList<>();

      // 2. Filter and Sort (Alphabetical for determinism)
      Arrays.sort(allFields, Comparator.comparing(Field::getName));

      StringBuilder schemaBuilder = new StringBuilder();
      int calcSize = 0;

      for (Field f : allFields) {
        // Skip static/final fields (like 'struct' or 'mInstance')
        if (java.lang.reflect.Modifier.isStatic(f.getModifiers())
            || java.lang.reflect.Modifier.isFinal(f.getModifiers())) {
          continue;
        }

        validFields.add(f);
        Class<?> type = f.getType();
        String name = f.getName();

        if (schemaBuilder.length() > 0) schemaBuilder.append("; ");

        if (type == double.class) {
          calcSize += 8;
          schemaBuilder.append("double ").append(name);
        } else if (type == boolean.class) {
          calcSize += 1;
          schemaBuilder.append("bool ").append(name);
        } else if (type == int.class) {
          calcSize += 4;
          schemaBuilder.append("int32 ").append(name);
        } else if (type == float.class) {
          calcSize += 4;
          schemaBuilder.append("float ").append(name);
        } else if (type == long.class) {
          calcSize += 8;
          schemaBuilder.append("int64 ").append(name);
        } else if (type == Pose2d.class) {
          calcSize += Pose2d.struct.getSize();
          schemaBuilder.append("Pose2d ").append(name);
        } else {
          System.err.println(
              "DashboardState: Unsupported type " + type.getName() + " for field " + name);
        }
      }

      mFields = validFields.toArray(new Field[0]);
      mSize = calcSize;
      mSchema = schemaBuilder.toString();
    }

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
      return mSize;
    }

    @Override
    public String getSchema() {
      return mSchema;
    }

    @Override
    public DashboardState unpack(ByteBuffer bb) {
      DashboardState state = new DashboardState();
      try {
        for (Field f : mFields) {
          Class<?> type = f.getType();
          if (type == double.class) f.setDouble(state, bb.getDouble());
          else if (type == boolean.class) f.setBoolean(state, bb.get() != 0);
          else if (type == int.class) f.setInt(state, bb.getInt());
          else if (type == float.class) f.setFloat(state, bb.getFloat());
          else if (type == long.class) f.setLong(state, bb.getLong());
          else if (type == Pose2d.class) f.set(state, Pose2d.struct.unpack(bb));
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
      return state;
    }

    @Override
    public void pack(ByteBuffer bb, DashboardState value) {
      try {
        for (Field f : mFields) {
          Class<?> type = f.getType();
          if (type == double.class) bb.putDouble(f.getDouble(value));
          else if (type == boolean.class) bb.put((byte) (f.getBoolean(value) ? 1 : 0));
          else if (type == int.class) bb.putInt(f.getInt(value));
          else if (type == float.class) bb.putFloat(f.getFloat(value));
          else if (type == long.class) bb.putLong(f.getLong(value));
          else if (type == Pose2d.class) Pose2d.struct.pack(bb, (Pose2d) f.get(value));
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }
}
