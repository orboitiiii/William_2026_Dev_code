package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Hardware abstraction interface for the Intake Pivot (Four-Bar Linkage) subsystem.
 *
 * <p>This interface defines the contract between the IntakePivot logic layer and the physical
 * hardware. The pivot mechanism uses a dual-motor configuration (leader-follower) for robust
 * four-bar linkage control.
 *
 * <p><strong>Design Pattern</strong>: IO Layer abstraction separates control logic from hardware
 * specifics, enabling unit testing without physical hardware.
 *
 * <p><strong>Mechanical Configuration</strong>: Two KrakenX60 motors drive the pivot mechanism. The
 * right motor (ID 30) acts as the position reference, while the left motor (ID 31) is inverted and
 * follows to produce synchronized, opposing torque for the four-bar linkage.
 *
 * @see IntakePivotIOReal
 */
public interface IntakePivotIO {

  /**
   * Container for all intake pivot sensor inputs.
   *
   * <p>Updated by {@link #updateInputs(IntakePivotIOInputs)} each control cycle.
   */
  public static class IntakePivotIOInputs {
    /** Current pivot angle measured from the right (leader) motor encoder. */
    public Rotation2d position = new Rotation2d();

    /** Angular velocity in rotations per second (after gear reduction). */
    public double velocityRotationsPerSec;

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
  public default void updateInputs(IntakePivotIOInputs inputs) {}

  /**
   * Commands the pivot to a target position using closed-loop control.
   *
   * <p>The position reference is the right motor encoder. The left motor follows in opposing
   * direction to maintain synchronized four-bar linkage motion.
   *
   * @param angle Target pivot angle.
   * @param feedforwardVolts Arbitrary feedforward voltage to apply.
   * @param velocity Motion Magic cruise velocity (rot/s).
   * @param acceleration Motion Magic acceleration (rot/s^2).
   */
  public default void setPosition(
      Rotation2d angle, double feedforwardVolts, double velocity, double acceleration) {}

  /**
   * Sets the voltage output of the pivot motors directly (open-loop).
   *
   * <p>Positive voltage moves the pivot in the "positive" direction (as defined by the right motor
   * convention). The left motor will automatically apply inverted voltage.
   *
   * @param volts Voltage command (-12 to +12).
   */
  public default void setVoltage(double volts) {}

  /** Stops both pivot motors by setting output to zero and engaging brake mode. */
  public default void stop() {}

  /**
   * Resets the motor encoder positions to the specified angle.
   *
   * <p>Used for calibration when the physical position is known (e.g., at a hard stop).
   *
   * @param angle The current physical angle to set as the encoder reference.
   */
  public default void resetPosition(Rotation2d angle) {}

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
