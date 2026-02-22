package frc.robot.subsystems.hood;

/**
 * Hardware abstraction interface for the Hood subsystem.
 *
 * <p>This interface defines the contract between the Hood logic layer and the physical hardware.
 * The hood is a linear actuator or servo mechanism that adjusts the projectile launch angle.
 *
 * <p><strong>Design Pattern</strong>: IO Layer abstraction separates control logic from hardware
 * specifics, enabling unit testing without physical hardware.
 *
 * <p><strong>Angle Definition</strong>: Hood angle is measured from the horizontal plane. Higher
 * angles produce a steeper trajectory for longer shots.
 *
 * @see HoodIOReal
 */
public interface HoodIO {

  /**
   * Container for all hood sensor inputs.
   *
   * <p>Updated by {@link #updateInputs(HoodIOInputs)} each control cycle.
   */
  public static class HoodIOInputs {
    /** Current hood angle in radians. */
    public double positionRads = 0.0;

    /** Angular velocity in radians per second. */
    public double velocityRadsPerSec = 0.0;

    /** Applied voltage to the hood motor. */
    public double appliedVolts = 0.0;

    /** Supply current in Amps. */
    public double currentAmps = 0.0;

    /** Motor CAN connection status (true = OK, false = error). */
    public boolean motorConnected = true;

    /** FPGA timestamp of the last input update. */
    public double timestamp = 0.0;
  }

  /** Hood output mode enumeration. */
  public enum HoodOutputMode {
    /** Motor in brake mode (hold position when stopped). */
    BRAKE,
    /** Motor in coast mode (free spin when stopped). */
    COAST,
    /** Closed-loop position control. */
    CLOSED_LOOP
  }

  /**
   * Updates the inputs object with the latest hardware data.
   *
   * <p>Called once per control cycle. Implementation should refresh CAN status signals and populate
   * the inputs struct.
   *
   * @param inputs The inputs container to populate.
   */
  public default void updateInputs(HoodIOInputs inputs) {}

  /**
   * Commands the hood to a target position using closed-loop control.
   *
   * @param positionRads Target hood angle in radians.
   * @param velocityRadsPerSec Feedforward velocity in radians per second.
   * @param feedforwardVolts Arbitrary feedforward voltage to add to the output.
   */
  public default void setPositionSetpoint(
      double positionRads, double velocityRadsPerSec, double feedforwardVolts) {}

  /**
   * Commands the hood to a target position using MotionMagic profiled control.
   *
   * <p>Use this for large moves (≥20°) to get smooth acceleration/deceleration profiles.
   *
   * @param positionRads Target hood angle in radians.
   */
  public default void setMotionMagicSetpoint(double positionRads) {}

  /**
   * Sets the voltage output of the hood motor directly (open-loop).
   *
   * @param volts Voltage command (-12 to +12).
   */
  public default void setVoltage(double volts) {}

  /** Stops the hood motor by setting output to zero. */
  public default void stop() {}

  /**
   * Sets the motor neutral mode.
   *
   * @param mode The desired output mode (BRAKE, COAST, or CLOSED_LOOP).
   */
  public default void setOutputMode(HoodOutputMode mode) {}

  /**
   * Sets the closed-loop PID gains.
   *
   * @param kP Proportional gain.
   * @param kD Derivative gain.
   */
  public default void setPIDGains(double kP, double kD) {}

  /**
   * Enables or disables the software position limits.
   *
   * <p><strong>Warning</strong>: Disabling soft limits allows the mechanism to travel beyond safe
   * ranges. Use only for testing and calibration with operator supervision.
   *
   * @param enabled True to enable soft limits, false to disable.
   */
  public default void setSoftLimitsEnabled(boolean enabled) {}

  /**
   * Reads firmware version from the hood motor.
   *
   * @return Firmware version integer (0 = read failed).
   */
  public default int getMotorFirmwareVersion() {
    return 0;
  }
}
