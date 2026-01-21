package frc.robot.subsystems.intake;

/**
 * Hardware abstraction interface for the Intake Wheel subsystem.
 *
 * <p>This interface defines the contract between the IntakeWheel logic layer and the physical
 * hardware. Implementations may target real hardware (IntakeWheelIOReal) or simulation.
 *
 * <p><strong>Design Pattern</strong>: IO Layer abstraction separates control logic from hardware
 * specifics, enabling unit testing without physical hardware.
 *
 * @see IntakeWheelIOReal
 */
public interface IntakeWheelIO {

  /**
   * Container for all intake wheel sensor inputs.
   *
   * <p>Updated by {@link #updateInputs(IntakeWheelIOInputs)} each control cycle.
   */
  public static class IntakeWheelIOInputs {
    /** Motor velocity in rotations per second. */
    public double velocityRotationsPerSec;

    /** Motor applied voltage. */
    public double appliedVolts;

    /** Motor supply current in Amps. */
    public double currentAmps;

    /** Motor CAN connection status (true = OK, false = error). */
    public boolean motorConnected = true;
  }

  /**
   * Updates the inputs object with the latest hardware data.
   *
   * <p>Called once per control cycle. Implementation should refresh CAN status signals and populate
   * the inputs struct.
   *
   * @param inputs The inputs container to populate.
   */
  public default void updateInputs(IntakeWheelIOInputs inputs) {}

  /**
   * Sets the voltage output of the intake motor.
   *
   * <p>Using voltage control ensures consistent output regardless of battery state.
   *
   * @param volts Voltage command (-12 to +12).
   */
  public default void setVoltage(double volts) {}

  /** Stops the intake motor by setting output to zero. */
  public default void stop() {}

  // --- Active Connection Check Methods ---

  /**
   * Reads firmware version from the intake motor.
   *
   * <p>Active query - performs blocking CAN read. Only call during init.
   *
   * @return Firmware version integer (0 = read failed).
   */
  public default int getMotorFirmwareVersion() {
    return 0;
  }
}
