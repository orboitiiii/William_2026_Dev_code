package frc.robot.subsystems.shooter;

/**
 * Hardware abstraction interface for the Shooter subsystem.
 *
 * <p>This interface defines the contract between the Shooter logic layer and the physical hardware.
 * The shooter uses a dual-KrakenX60 configuration with leader-follower opposed alignment for
 * synchronized, mirrored torque output.
 *
 * <p><strong>Design Pattern</strong>: IO Layer abstraction separates control logic from hardware
 * specifics, enabling unit testing without physical hardware.
 *
 * <p><strong>Mechanical Configuration</strong>: Two KrakenX60 motors drive the shooter flywheel.
 * The right motor acts as the leader, while the left motor follows in opposed direction to produce
 * synchronized, mirrored rotation suitable for dual-flywheel shooter designs.
 *
 * @see ShooterIOReal
 */
public interface ShooterIO {

  /**
   * Container for all shooter sensor inputs.
   *
   * <p>Updated by {@link #updateInputs(ShooterIOInputs)} each control cycle.
   */
  public static class ShooterIOInputs {
    /** Right motor (leader) position in rotations. Used for SysId data collection. */
    public double rightPositionRot;

    /** Right motor (leader) angular velocity in rotations per second. */
    public double rightVelocityRotPerSec;

    /** Left motor (follower) angular velocity in rotations per second. */
    public double leftVelocityRotPerSec;

    /** Right motor (leader) applied voltage. */
    public double rightAppliedVolts;

    /** Left motor (follower) applied voltage. */
    public double leftAppliedVolts;

    /** Right motor supply current in Amps. */
    public double rightCurrentAmps;

    /** Left motor supply current in Amps. */
    public double leftCurrentAmps;

    /** Right motor CAN connection status (true = OK, false = error). */
    public boolean rightMotorConnected = true;

    /** Left motor CAN connection status (true = OK, false = error). */
    public boolean leftMotorConnected = true;
  }

  /**
   * Updates the inputs object with the latest hardware data.
   *
   * <p>Called once per control cycle. Implementation should refresh CAN status signals and populate
   * the inputs struct.
   *
   * @param inputs The inputs container to populate.
   */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Sets the voltage output of the shooter motors directly (open-loop).
   *
   * <p>Positive voltage spins the shooter in the "firing" direction. The left motor will
   * automatically apply inverted voltage due to opposed follower alignment.
   *
   * @param volts Voltage command (-12 to +12).
   */
  public default void setVoltage(double volts) {}

  /**
   * Sets the target velocity for the shooter motors (closed-loop).
   *
   * <p>Uses the hardware-level PID controller (Slot 0).
   *
   * @param rotationsPerSec Target velocity in rotations per second.
   */
  public default void setTargetVelocity(double rotationsPerSec) {}

  /** Stops both shooter motors by setting output to zero and engaging brake mode. */
  public default void stop() {}

  // --- Active Connection Check Methods ---

  /**
   * Reads firmware version from the right (leader) motor.
   *
   * <p>Active query - performs blocking CAN read. Only call during init.
   *
   * @return Firmware version integer (0 = read failed).
   */
  public default int getRightMotorFirmwareVersion() {
    return 0;
  }

  /**
   * Reads firmware version from the left (follower) motor.
   *
   * <p>Active query - performs blocking CAN read. Only call during init.
   *
   * @return Firmware version integer (0 = read failed).
   */
  public default int getLeftMotorFirmwareVersion() {
    return 0;
  }
}
