package frc.robot.libraries.lib9427;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import java.nio.ByteBuffer;

/**
 * Single time-point snapshot of complete odometry data.
 *
 * <p>Contains robot-level and wheel-level kinematic data for post-hoc analysis of odometry error
 * sources. Data granularity supports causal inference and advanced statistical analysis.
 *
 * <p><strong>Coordinate Frame</strong>: All values are in NWU (North-West-Up) field-relative or
 * robot-relative as noted. Angles use radians, distances use meters.
 *
 * <p><strong>DataLog Struct</strong>: Implements WPILib StructSerializable for high-performance
 * binary logging (10-100x faster than CSV text format).
 *
 * @see OdometryTrial
 * @see OdometryCharacterizer
 */
public record OdometryDataPoint(
    // --- Timestamp ---
    /** FPGA timestamp in seconds. */
    double timestamp,
    /** Actual time interval from previous data point (seconds). Used for jitter detection. */
    double dtActual,

    // --- Robot-Level Kinematics ---
    /** Odometry X position (meters). */
    double poseX,
    /** Odometry Y position (meters). */
    double poseY,
    /** Heading angle (radians, CCW positive). */
    double heading,
    /** Robot X velocity (m/s). */
    double vx,
    /** Robot Y velocity (m/s). */
    double vy,
    /** Robot angular velocity (rad/s, CCW positive). */
    double omega,
    /** Robot X acceleration from IMU (m/s²). */
    double ax,
    /** Robot Y acceleration from IMU (m/s²). */
    double ay,
    /** Robot angular acceleration (rad/s²), computed via d(omega)/dt. */
    double alpha,

    // --- Per-Wheel Kinematics ---
    // Index: [0]=FL, [1]=FR, [2]=BL, [3]=BR

    /** Wheel drive velocity (tangential, m/s). */
    double[] wheelDriveVelocity,
    /** Wheel drive acceleration (m/s²). */
    double[] wheelDriveAccel,
    /** Wheel steer angle (radians, 0=forward). */
    double[] wheelSteerAngle,
    /** Wheel steer angular velocity (rad/s). */
    double[] wheelSteerVelocity,
    /** Wheel steer angular acceleration (rad/s²). */
    double[] wheelSteerAccel,
    /** Wheel X velocity component in robot frame: v*cos(θ). */
    double[] wheelVx,
    /** Wheel Y velocity component in robot frame: v*sin(θ). */
    double[] wheelVy,

    // --- Electrical ---
    /** Battery voltage (V). */
    double batteryVoltage,
    /** Drive motor current (A) per module. */
    double[] driveMotorCurrent,
    /** Steer motor current (A) per module. */
    double[] steerMotorCurrent,
    /** Drive motor supply voltage (V) per module. */
    double[] driveMotorSupplyVoltage,

    // --- Control ---
    /** Drive velocity setpoint (m/s) per module. */
    double[] wheelDriveSetpoint,
    /** Steer angle setpoint (rad) per module. */
    double[] wheelSteerSetpoint,
    /** Drive velocity error (setpoint - actual, m/s) per module. */
    double[] wheelDriveError,
    /** Steer angle error (setpoint - actual, rad) per module. */
    double[] wheelSteerError,

    // --- Derived Metrics ---
    /** |v_FL - v_FR|. */
    double wheelSpeedDiff_FL_FR,
    /** |v_BL - v_BR|. */
    double wheelSpeedDiff_BL_BR,
    /** |v_FL - v_BL|. */
    double wheelSpeedDiff_FL_BL,
    /** |v_FR - v_BR|. */
    double wheelSpeedDiff_FR_BR,
    /** max(|v_FL - v_BR|, |v_FR - v_BL|). */
    double wheelSpeedDiff_Diagonal,
    /** Variance of [v_FL, v_FR, v_BL, v_BR]. */
    double wheelSpeedVariance,
    /** Estimated slip ratio based on wheel speed variance and acceleration. */
    double slipRatio)
    implements StructSerializable {

  /** Number of modules (always 4 for swerve). */
  public static final int NUM_MODULES = 4;

  /**
   * The binary struct definition for high-performance logging.
   *
   * <p>Total Size: 85 doubles * 8 bytes = 680 bytes per record.
   */
  public static final Struct<OdometryDataPoint> struct = new OdometryDataPointStruct();

  /**
   * Creates a builder for constructing OdometryDataPoint with mutable state.
   *
   * @return A new Builder instance.
   */
  public static Builder builder() {
    return new Builder();
  }

  /** Mutable builder for OdometryDataPoint. */
  public static class Builder {
    private double timestamp;
    private double dtActual;
    private double poseX, poseY, heading;
    private double vx, vy, omega;
    private double ax, ay, alpha;
    private double[] wheelDriveVelocity = new double[NUM_MODULES];
    private double[] wheelDriveAccel = new double[NUM_MODULES];
    private double[] wheelSteerAngle = new double[NUM_MODULES];
    private double[] wheelSteerVelocity = new double[NUM_MODULES];
    private double[] wheelSteerAccel = new double[NUM_MODULES];
    private double[] wheelVx = new double[NUM_MODULES];
    private double[] wheelVy = new double[NUM_MODULES];
    private double batteryVoltage;
    private double[] driveMotorCurrent = new double[NUM_MODULES];
    private double[] steerMotorCurrent = new double[NUM_MODULES];
    private double[] driveMotorSupplyVoltage = new double[NUM_MODULES];
    private double[] wheelDriveSetpoint = new double[NUM_MODULES];
    private double[] wheelSteerSetpoint = new double[NUM_MODULES];
    private double[] wheelDriveError = new double[NUM_MODULES];
    private double[] wheelSteerError = new double[NUM_MODULES];
    private double wheelSpeedDiff_FL_FR;
    private double wheelSpeedDiff_BL_BR;
    private double wheelSpeedDiff_FL_BL;
    private double wheelSpeedDiff_FR_BR;
    private double wheelSpeedDiff_Diagonal;
    private double wheelSpeedVariance;
    private double slipRatio;

    public Builder timestamp(double v) {
      this.timestamp = v;
      return this;
    }

    public Builder dtActual(double v) {
      this.dtActual = v;
      return this;
    }

    public Builder pose(double x, double y, double heading) {
      this.poseX = x;
      this.poseY = y;
      this.heading = heading;
      return this;
    }

    public Builder robotVelocity(double vx, double vy, double omega) {
      this.vx = vx;
      this.vy = vy;
      this.omega = omega;
      return this;
    }

    public Builder robotAcceleration(double ax, double ay, double alpha) {
      this.ax = ax;
      this.ay = ay;
      this.alpha = alpha;
      return this;
    }

    public Builder wheelDriveVelocity(double[] v) {
      this.wheelDriveVelocity = v.clone();
      return this;
    }

    public Builder wheelDriveAccel(double[] v) {
      this.wheelDriveAccel = v.clone();
      return this;
    }

    public Builder wheelSteerAngle(double[] v) {
      this.wheelSteerAngle = v.clone();
      return this;
    }

    public Builder wheelSteerVelocity(double[] v) {
      this.wheelSteerVelocity = v.clone();
      return this;
    }

    public Builder wheelSteerAccel(double[] v) {
      this.wheelSteerAccel = v.clone();
      return this;
    }

    public Builder wheelVxVy(double[] vx, double[] vy) {
      this.wheelVx = vx.clone();
      this.wheelVy = vy.clone();
      return this;
    }

    public Builder electrical(
        double batteryV, double[] driveCurrent, double[] steerCurrent, double[] supplyVoltage) {
      this.batteryVoltage = batteryV;
      this.driveMotorCurrent = driveCurrent.clone();
      this.steerMotorCurrent = steerCurrent.clone();
      this.driveMotorSupplyVoltage = supplyVoltage.clone();
      return this;
    }

    public Builder controlSetpoints(double[] driveSetpoint, double[] steerSetpoint) {
      this.wheelDriveSetpoint = driveSetpoint.clone();
      this.wheelSteerSetpoint = steerSetpoint.clone();
      return this;
    }

    public Builder controlErrors(double[] driveError, double[] steerError) {
      this.wheelDriveError = driveError.clone();
      this.wheelSteerError = steerError.clone();
      return this;
    }

    public Builder wheelSpeedDiffs(
        double flFr, double blBr, double flBl, double frBr, double diag) {
      this.wheelSpeedDiff_FL_FR = flFr;
      this.wheelSpeedDiff_BL_BR = blBr;
      this.wheelSpeedDiff_FL_BL = flBl;
      this.wheelSpeedDiff_FR_BR = frBr;
      this.wheelSpeedDiff_Diagonal = diag;
      return this;
    }

    public Builder wheelSpeedVariance(double v) {
      this.wheelSpeedVariance = v;
      return this;
    }

    public Builder slipRatio(double v) {
      this.slipRatio = v;
      return this;
    }

    /** Computes derived metrics from current wheel velocities. */
    public Builder computeDerivedMetrics() {
      double[] v = wheelDriveVelocity;
      wheelSpeedDiff_FL_FR = Math.abs(v[0] - v[1]);
      wheelSpeedDiff_BL_BR = Math.abs(v[2] - v[3]);
      wheelSpeedDiff_FL_BL = Math.abs(v[0] - v[2]);
      wheelSpeedDiff_FR_BR = Math.abs(v[1] - v[3]);
      wheelSpeedDiff_Diagonal = Math.max(Math.abs(v[0] - v[3]), Math.abs(v[1] - v[2]));

      double mean = (v[0] + v[1] + v[2] + v[3]) / 4.0;
      double sumSq = 0;
      for (double vel : v) {
        sumSq += (vel - mean) * (vel - mean);
      }
      wheelSpeedVariance = sumSq / 4.0;

      // Simple slip ratio estimate: high variance + high acceleration suggests slip
      double robotAccel = Math.hypot(ax, ay);
      slipRatio = wheelSpeedVariance * (1 + robotAccel);
      return this;
    }

    public OdometryDataPoint build() {
      return new OdometryDataPoint(
          timestamp,
          dtActual,
          poseX,
          poseY,
          heading,
          vx,
          vy,
          omega,
          ax,
          ay,
          alpha,
          wheelDriveVelocity,
          wheelDriveAccel,
          wheelSteerAngle,
          wheelSteerVelocity,
          wheelSteerAccel,
          wheelVx,
          wheelVy,
          batteryVoltage,
          driveMotorCurrent,
          steerMotorCurrent,
          driveMotorSupplyVoltage,
          wheelDriveSetpoint,
          wheelSteerSetpoint,
          wheelDriveError,
          wheelSteerError,
          wheelSpeedDiff_FL_FR,
          wheelSpeedDiff_BL_BR,
          wheelSpeedDiff_FL_BL,
          wheelSpeedDiff_FR_BR,
          wheelSpeedDiff_Diagonal,
          wheelSpeedVariance,
          slipRatio);
    }
  }

  /** Internal Struct definition for binary serialization. */
  private static class OdometryDataPointStruct implements Struct<OdometryDataPoint> {
    @Override
    public Class<OdometryDataPoint> getTypeClass() {
      return OdometryDataPoint.class;
    }

    @Override
    public String getTypeString() {
      return "struct:OdometryDataPoint";
    }

    @Override
    public String getTypeName() {
      return "OdometryDataPoint";
    }

    @Override
    public int getSize() {
      // 85 doubles * 8 bytes = 680 bytes
      return 680;
    }

    @Override
    public String getSchema() {
      // Schema string for DataLog layout
      return "double timestamp;double dtActual;"
          + "double poseX;double poseY;double heading;"
          + "double vx;double vy;double omega;"
          + "double ax;double ay;double alpha;"
          + "double wheelDriveVelocity[4];double wheelDriveAccel[4];"
          + "double wheelSteerAngle[4];double wheelSteerVelocity[4];double wheelSteerAccel[4];"
          + "double wheelVx[4];double wheelVy[4];"
          + "double batteryVoltage;"
          + "double driveMotorCurrent[4];double steerMotorCurrent[4];double driveSupplyVoltage[4];"
          + "double wheelDriveSetpoint[4];double wheelSteerSetpoint[4];"
          + "double wheelDriveError[4];double wheelSteerError[4];"
          + "double wheelSpeedDiff_FL_FR;double wheelSpeedDiff_BL_BR;"
          + "double wheelSpeedDiff_FL_BL;double wheelSpeedDiff_FR_BR;"
          + "double wheelSpeedDiff_Diagonal;double wheelSpeedVariance;double slipRatio";
    }

    @Override
    public OdometryDataPoint unpack(ByteBuffer bb) {
      // Unpack logic is rarely used on robot, but required by interface.
      // We implement it for completeness or PC-side deserialization.
      double timestamp = bb.getDouble();
      double dtActual = bb.getDouble();
      double poseX = bb.getDouble();
      double poseY = bb.getDouble();
      double heading = bb.getDouble();
      double vx = bb.getDouble();
      double vy = bb.getDouble();
      double omega = bb.getDouble();
      double ax = bb.getDouble();
      double ay = bb.getDouble();
      double alpha = bb.getDouble();

      double[] wheelDriveVelocity = Struct.unpackDoubleArray(bb, 4);
      double[] wheelDriveAccel = Struct.unpackDoubleArray(bb, 4);
      double[] wheelSteerAngle = Struct.unpackDoubleArray(bb, 4);
      double[] wheelSteerVelocity = Struct.unpackDoubleArray(bb, 4);
      double[] wheelSteerAccel = Struct.unpackDoubleArray(bb, 4);
      double[] wheelVx = Struct.unpackDoubleArray(bb, 4);
      double[] wheelVy = Struct.unpackDoubleArray(bb, 4);

      double batteryVoltage = bb.getDouble();
      double[] driveMotorCurrent = Struct.unpackDoubleArray(bb, 4);
      double[] steerMotorCurrent = Struct.unpackDoubleArray(bb, 4);
      double[] driveMotorSupplyVoltage = Struct.unpackDoubleArray(bb, 4);

      double[] wheelDriveSetpoint = Struct.unpackDoubleArray(bb, 4);
      double[] wheelSteerSetpoint = Struct.unpackDoubleArray(bb, 4);
      double[] wheelDriveError = Struct.unpackDoubleArray(bb, 4);
      double[] wheelSteerError = Struct.unpackDoubleArray(bb, 4);

      double wheelSpeedDiff_FL_FR = bb.getDouble();
      double wheelSpeedDiff_BL_BR = bb.getDouble();
      double wheelSpeedDiff_FL_BL = bb.getDouble();
      double wheelSpeedDiff_FR_BR = bb.getDouble();
      double wheelSpeedDiff_Diagonal = bb.getDouble();
      double wheelSpeedVariance = bb.getDouble();
      double slipRatio = bb.getDouble();

      return new OdometryDataPoint(
          timestamp,
          dtActual,
          poseX,
          poseY,
          heading,
          vx,
          vy,
          omega,
          ax,
          ay,
          alpha,
          wheelDriveVelocity,
          wheelDriveAccel,
          wheelSteerAngle,
          wheelSteerVelocity,
          wheelSteerAccel,
          wheelVx,
          wheelVy,
          batteryVoltage,
          driveMotorCurrent,
          steerMotorCurrent,
          driveMotorSupplyVoltage,
          wheelDriveSetpoint,
          wheelSteerSetpoint,
          wheelDriveError,
          wheelSteerError,
          wheelSpeedDiff_FL_FR,
          wheelSpeedDiff_BL_BR,
          wheelSpeedDiff_FL_BL,
          wheelSpeedDiff_FR_BR,
          wheelSpeedDiff_Diagonal,
          wheelSpeedVariance,
          slipRatio);
    }

    @Override
    public void pack(ByteBuffer bb, OdometryDataPoint value) {
      bb.putDouble(value.timestamp);
      bb.putDouble(value.dtActual);
      bb.putDouble(value.poseX);
      bb.putDouble(value.poseY);
      bb.putDouble(value.heading);
      bb.putDouble(value.vx);
      bb.putDouble(value.vy);
      bb.putDouble(value.omega);
      bb.putDouble(value.ax);
      bb.putDouble(value.ay);
      bb.putDouble(value.alpha);

      for (double v : value.wheelDriveVelocity) bb.putDouble(v);
      for (double v : value.wheelDriveAccel) bb.putDouble(v);
      for (double v : value.wheelSteerAngle) bb.putDouble(v);
      for (double v : value.wheelSteerVelocity) bb.putDouble(v);
      for (double v : value.wheelSteerAccel) bb.putDouble(v);
      for (double v : value.wheelVx) bb.putDouble(v);
      for (double v : value.wheelVy) bb.putDouble(v);

      bb.putDouble(value.batteryVoltage);
      for (double v : value.driveMotorCurrent) bb.putDouble(v);
      for (double v : value.steerMotorCurrent) bb.putDouble(v);
      for (double v : value.driveMotorSupplyVoltage) bb.putDouble(v);

      for (double v : value.wheelDriveSetpoint) bb.putDouble(v);
      for (double v : value.wheelSteerSetpoint) bb.putDouble(v);
      for (double v : value.wheelDriveError) bb.putDouble(v);
      for (double v : value.wheelSteerError) bb.putDouble(v);

      bb.putDouble(value.wheelSpeedDiff_FL_FR);
      bb.putDouble(value.wheelSpeedDiff_BL_BR);
      bb.putDouble(value.wheelSpeedDiff_FL_BL);
      bb.putDouble(value.wheelSpeedDiff_FR_BR);
      bb.putDouble(value.wheelSpeedDiff_Diagonal);
      bb.putDouble(value.wheelSpeedVariance);
      bb.putDouble(value.slipRatio);
    }
  }
}
