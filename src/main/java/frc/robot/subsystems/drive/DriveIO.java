package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Hardware abstraction interface for the Drive subsystem.
 *
 * <p>This interface defines the contract between the Drive logic layer and the physical hardware.
 * Implementations may target real hardware (DriveIOReal) or simulation environments.
 *
 * <p><strong>Design Pattern</strong>: IO Layer abstraction separates control logic from hardware
 * specifics, enabling unit testing without physical hardware.
 *
 * <p><strong>Module Indexing Convention</strong>:
 *
 * <ul>
 *   <li>0 = Front Left (FL)
 *   <li>1 = Front Right (FR)
 *   <li>2 = Back Left (BL)
 *   <li>3 = Back Right (BR)
 * </ul>
 */
public interface DriveIO {

  /**
   * Container for all drive subsystem sensor inputs.
   *
   * <p>Updated by {@link #updateInputs(DriveIOInputs)} each control cycle. All arrays use the
   * standard module indexing (FL, FR, BL, BR).
   */
  public static class DriveIOInputs {
    /** FPGA timestamp when this data was captured. */
    public double timestamp;

    /** Robot heading from the gyroscope. */
    public Rotation2d gyroYaw = new Rotation2d();

    /** Gyroscope yaw rate in radians per second. */
    public double gyroYawVelocityRadPerSec;

    /** Linear acceleration in m/s² [x, y, z] from the IMU. */
    public double[] accelMetersPerSec2 = new double[3];

    /** Drive motor position in rotor rotations (after gear ratio). */
    public double[] drivePositionRotations = new double[4];

    /** Drive motor velocity in rotations per second. */
    public double[] driveVelocityRotationsPerSec = new double[4];

    /** Drive motor applied voltage. */
    public double[] driveAppliedVolts = new double[4];

    /** Drive motor supply current in Amps. */
    public double[] driveCurrentAmps = new double[4];

    /** Drive motor supply voltage in Volts. */
    public double[] driveSupplyVoltage = new double[4];

    /** Steer motor position in rotations (mechanism angle). */
    public double[] steerPositionRotations = new double[4];

    /** Steer motor velocity in rotations per second. */
    public double[] steerVelocityRotationsPerSec = new double[4];

    /** Steer motor applied voltage. */
    public double[] steerAppliedVolts = new double[4];

    /** Steer motor supply current in Amps. */
    public double[] steerCurrentAmps = new double[4];

    /** CANcoder absolute position in rotations (with MagnetOffset applied). */
    public double[] steerAbsolutePositionRotations = new double[4];

    // --- Connection Status (Passive Check) ---

    /** Drive motor CAN connection status per module (true = OK). */
    public boolean[] driveMotorConnected = new boolean[] {true, true, true, true};

    /** Steer motor CAN connection status per module (true = OK). */
    public boolean[] steerMotorConnected = new boolean[] {true, true, true, true};

    /** CANcoder connection status per module (true = OK). */
    public boolean[] cancoderConnected = new boolean[] {true, true, true, true};

    /** Pigeon IMU connection status (true = OK). */
    public boolean pigeonConnected = true;
  }

  /**
   * Updates the inputs object with the latest hardware data.
   *
   * <p>Called once per control cycle. Implementation should refresh all CAN status signals and
   * populate the inputs struct.
   *
   * @param inputs The inputs container to populate.
   */
  public default void updateInputs(DriveIOInputs inputs) {}

  /**
   * Sets the voltage output of a drive motor.
   *
   * @param moduleIndex Module index (0-3).
   * @param volts Voltage command (-12 to +12).
   */
  public default void setDriveVoltage(int moduleIndex, double volts) {}

  /**
   * Sets the voltage output of a steer motor.
   *
   * @param moduleIndex Module index (0-3).
   * @param volts Voltage command (-12 to +12).
   */
  public default void setSteerVoltage(int moduleIndex, double volts) {}

  /**
   * Sets the velocity setpoint of a drive motor (closed-loop).
   *
   * @param moduleIndex Module index (0-3).
   * @param velocityRadPerSec Desired velocity in rotations per second.
   */
  public default void setDriveVelocity(int moduleIndex, double velocityRadPerSec) {}

  /**
   * Sets the position setpoint of a steer motor (closed-loop).
   *
   * @param moduleIndex Module index (0-3).
   * @param rotations Desired angle in rotations (mechanism angle, not rotor).
   */
  public default void setSteerPosition(int moduleIndex, double rotations) {}

  /**
   * Enables or disables brake mode for all drive motors.
   *
   * @param enable True for brake mode, false for coast.
   */
  public default void setDriveBrakeMode(boolean enable) {}

  /**
   * Sets the gyroscope yaw to a specified value.
   *
   * @param degrees The new yaw value in degrees.
   */
  public default void setGyroYaw(double degrees) {}

  // --- Calibration Methods ---

  /**
   * Reads the current absolute positions from all CANcoders.
   *
   * <p>The returned values include the currently configured MagnetOffset. Used during calibration
   * to determine the offset correction.
   *
   * @return Array of 4 absolute positions in rotations.
   */
  public default double[] getAbsolutePositionsRotations() {
    return new double[4];
  }

  /**
   * Reads the currently configured MagnetOffset from all CANcoders.
   *
   * @return Array of 4 MagnetOffset values in rotations.
   */
  public default double[] getCurrentMagnetOffsets() {
    return new double[4];
  }

  /**
   * Sets the MagnetOffset for a specific CANcoder.
   *
   * <p>This value is persisted in the CANcoder's flash memory and automatically applied to all
   * future absolute position readings.
   *
   * @param moduleIndex Module index (0-3).
   * @param offset New MagnetOffset value in rotations.
   */
  public default void setMagnetOffset(int moduleIndex, double offset) {}

  /**
   * Plays a tone on the drive motors for audible feedback.
   *
   * <p>Used to confirm calibration success. Set frequency to 0 for silence.
   *
   * @param frequencyHz Tone frequency in Hz (10-20000), or 0 to stop.
   */
  public default void playTone(double frequencyHz) {}

  // --- Active Connection Check Methods ---

  /**
   * Reads firmware versions from all drive motors.
   *
   * <p>Active query - performs blocking CAN read. Only call during init.
   *
   * @return Array of 4 firmware version integers (0 = read failed).
   */
  public default int[] getDriveMotorFirmwareVersions() {
    return new int[4];
  }

  /**
   * Reads firmware versions from all steer motors.
   *
   * @return Array of 4 firmware version integers (0 = read failed).
   */
  public default int[] getSteerMotorFirmwareVersions() {
    return new int[4];
  }

  /**
   * Reads firmware version from Pigeon IMU.
   *
   * @return Firmware version integer (0 = read failed).
   */
  public default int getPigeonFirmwareVersion() {
    return 0;
  }
}
