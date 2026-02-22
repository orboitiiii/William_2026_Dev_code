package frc.robot.subsystems.turret;

/**
 * Hardware abstraction interface for the Turret subsystem.
 *
 * <p>This interface defines the contract between the Turret logic layer and the physical hardware.
 * The turret is a single-motor continuous rotation mechanism with CANcoder absolute position
 * feedback.
 *
 * <p><strong>Design Pattern</strong>: IO Layer abstraction separates control logic from hardware
 * specifics, enabling unit testing without physical hardware.
 *
 * <p><strong>Coordinate Frame</strong>: Positive angle is counter-clockwise when viewed from above.
 * Zero is defined as the turret facing forward relative to the robot chassis.
 *
 * @see TurretIOReal
 */
public interface TurretIO {

  /**
   * Container for all turret sensor inputs.
   *
   * <p>Updated by {@link #updateInputs(TurretIOInputs)} each control cycle.
   */
  public static class TurretIOInputs {
    /** Current turret angle in radians (robot-relative, CCW positive). */
    public double positionRads = 0.0;

    /** Angular velocity in radians per second. */
    public double velocityRadsPerSec = 0.0;

    /** Applied voltage to the turret motor. */
    public double appliedVolts = 0.0;

    /** Supply current in Amps. */
    public double currentAmps = 0.0;

    /** Motor CAN connection status (true = OK, false = error). */
    public boolean motorConnected = true;

    /** Motor axis CANcoder (ID 41) connection status. */
    public boolean encoder1Connected = true;

    /** Auxiliary axis CANcoder (ID 42) connection status. */
    public boolean encoder2Connected = true;

    /** Absolute position of motor encoder in rotations (0-1). */
    public double motorEncoderAbsPosRotations = 0.0;

    /** Absolute position of auxiliary encoder in rotations (0-1). */
    public double auxEncoderAbsPosRotations = 0.0;

    /** FPGA timestamp of the last input update. */
    public double timestamp = 0.0;
  }

  /**
   * Updates the inputs object with the latest hardware data.
   *
   * <p>Called once per control cycle. Implementation should refresh CAN status signals and populate
   * the inputs struct.
   *
   * @param inputs The inputs container to populate.
   */
  public default void updateInputs(TurretIOInputs inputs) {}

  /**
   * Commands the turret to a target position using closed-loop control.
   *
   * <p>The position is in radians relative to the robot chassis. The turret will use the internal
   * motion profile to reach the target smoothly.
   *
   * @param positionRads Target turret angle in radians.
   * @param velocityRadsPerSec Feedforward velocity in radians per second.
   * @param feedforwardVolts Open-loop voltage to apply on top of the PID and standard velocity FF.
   */
  public default void setPositionSetpoint(
      double positionRads, double velocityRadsPerSec, double feedforwardVolts) {}

  /**
   * Sets the voltage output of the turret motor directly (open-loop).
   *
   * <p>Positive voltage rotates the turret counter-clockwise (when viewed from above).
   *
   * @param volts Voltage command (-12 to +12).
   */
  public default void setVoltage(double volts) {}

  /** Stops the turret motor by setting output to zero. */
  public default void stop() {}

  /**
   * Sets the motor neutral mode to brake or coast.
   *
   * @param brake True for brake mode, false for coast mode.
   */
  public default void setBrakeMode(boolean brake) {}

  /**
   * Sets the closed-loop PID gains.
   *
   * <p>Used for dynamic gain tuning via SmartDashboard.
   *
   * @param kP Proportional gain.
   * @param kD Derivative gain.
   */
  public default void setPIDGains(double kP, double kD) {}

  /**
   * Enables or disables software position limits.
   *
   * <p><strong>Warning</strong>: Disabling soft limits can allow travel beyond safe ranges.
   *
   * @param enabled True to enable limits, false to disable.
   */
  public default void setSoftLimitsEnabled(boolean enabled) {}

  /**
   * Initializes absolute turret angle using CRT at startup, and writes it to the motor encoder.
   *
   * <p>Reads absolute positions from both CANcoders, solves for the unique turret tooth position
   * using the Chinese Remainder Theorem, converts it to angle, and sets the motor encoder's initial
   * position.
   *
   * <p><strong>Postconditions</strong>:
   *
   * <ul>
   *   <li>Success: Motor encoder set to correct turret angle, returns that angle (rad).
   *   <li>Failure (Disconnect/Inconsistent): Returns NaN, motor encoder remains unchanged.
   * </ul>
   *
   * <p><strong>Fail-Safe</strong>: On failure, upper layer (Turret) will not set mZeroed = true, so
   * writePeriodicOutputs will continue holding position.
   *
   * @return Calculated turret angle (rad), range [0, 2π); Returns NaN on failure.
   */
  public default double initializeAbsolutePosition() {
    return Double.NaN;
  }

  // --- Active Connection Check Methods ---

  /**
   * Reads firmware version from the turret motor.
   *
   * <p>Active query - performs blocking CAN read. Only call during init.
   *
   * @return Firmware version integer (0 = read failed).
   */
  public default int getMotorFirmwareVersion() {
    return 0;
  }
}
