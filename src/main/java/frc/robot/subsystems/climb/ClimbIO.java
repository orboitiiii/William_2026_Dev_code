package frc.robot.subsystems.climb;

/**
 * Hardware abstraction interface for the Climb subsystem.
 *
 * <p>This interface defines the contract between the Climb logic layer and the physical hardware.
 * The climb system uses a Kraken X44 motor through a 1:125 gearbox to pull a string.
 */
public interface ClimbIO {

  /**
   * Container for all climb sensor inputs.
   *
   * <p>Updated by {@link #updateInputs(ClimbIOInputs)} each control cycle.
   */
  public static class ClimbIOInputs {
    /** Current spool position in rotations. */
    public double positionRotations = 0.0;

    /** Motor velocity in rotations per second (mechanism units). */
    public double velocityRotSec = 0.0;

    /** Applied voltage to the climb motor. */
    public double appliedVolts = 0.0;

    /** Supply current in Amps. */
    public double currentAmps = 0.0;

    /** Motor CAN connection status (true = OK, false = error). */
    public boolean motorConnected = true;

    /** FPGA timestamp of the last input update. */
    public double timestamp = 0.0;
  }

  /**
   * Updates the inputs object with the latest hardware data.
   *
   * @param inputs The inputs container to populate.
   */
  public default void updateInputs(ClimbIOInputs inputs) {}

  /**
   * Commands the motor to a target position using closed-loop control.
   *
   * @param positionRotations Target spool position in rotations.
   * @param feedforwardVolts Arbitrary feedforward voltage to add to the output.
   */
  public default void setPositionSetpoint(double positionRotations, double feedforwardVolts) {}

  /**
   * Sets the voltage output of the motor directly (open-loop).
   *
   * <p><strong>Warning:</strong> Use carefully as this ignores position limits!
   *
   * @param volts Voltage command (-12 to +12).
   */
  public default void setVoltage(double volts) {}

  /** Stops the motor by setting voltage to zero. */
  public default void stop() {}

  /**
   * Enables or disables brake mode on the motor.
   *
   * @param enable True for Brake mode, False for Coast mode.
   */
  public default void setBrakeMode(boolean enable) {}
}
