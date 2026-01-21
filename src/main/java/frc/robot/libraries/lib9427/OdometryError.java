package frc.robot.libraries.lib9427;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/**
 * Represents the final pose error after a trial returns to origin.
 *
 * <p><strong>DataLog Struct</strong>: Implements WPILib StructSerializable for binary logging.
 *
 * @param dx X position error (meters).
 * @param dy Y position error (meters).
 * @param dTheta Heading error (radians).
 */
public record OdometryError(double dx, double dy, double dTheta) implements StructSerializable {

  /** Binary struct definition. */
  public static final Struct<OdometryError> struct = new OdometryErrorStruct();

  /**
   * Computes the Euclidean translation error.
   *
   * @return sqrt(dx² + dy²) in meters.
   */
  public double getTranslationError() {
    return Math.hypot(dx, dy);
  }

  /**
   * Formats error as a human-readable string.
   *
   * @return Formatted error string.
   */
  @Override
  public String toString() {
    return String.format(
        "OdometryError(dx=%.4fm, dy=%.4fm, dθ=%.2f°, trans=%.4fm)",
        dx, dy, Math.toDegrees(dTheta), getTranslationError());
  }

  /**
   * Returns CSV row for this error.
   *
   * @return Comma-separated values: dx, dy, dTheta, translationError.
   */
  public String toCsvRow() {
    return String.format("%.6f,%.6f,%.6f,%.6f", dx, dy, dTheta, getTranslationError());
  }

  /** CSV header for error columns. */
  public static String csvHeader() {
    return "error_x,error_y,error_theta,error_trans";
  }

  /** Internal Struct definition. */
  private static class OdometryErrorStruct implements Struct<OdometryError> {
    @Override
    public Class<OdometryError> getTypeClass() {
      return OdometryError.class;
    }

    @Override
    public String getTypeString() {
      return "struct:OdometryError";
    }

    // @Override
    // public String getTypeString() {
    // return "struct:OdometryError";
    // }

    // @Override
    // public String getTypeName() {
    // return "OdometryError";
    // }

    @Override
    public String getTypeName() {
      return "OdometryDataPoint";
    }

    @Override
    public int getSize() {
      return 24; // 3 doubles * 8
    }

    @Override
    public String getSchema() {
      return "double dx;double dy;double dTheta";
    }

    @Override
    public OdometryError unpack(ByteBuffer bb) {
      double dx = bb.getDouble();
      double dy = bb.getDouble();
      double dTheta = bb.getDouble();
      return new OdometryError(dx, dy, dTheta);
    }

    @Override
    public void pack(ByteBuffer bb, OdometryError value) {
      bb.putDouble(value.dx);
      bb.putDouble(value.dy);
      bb.putDouble(value.dTheta);
    }
  }
}
