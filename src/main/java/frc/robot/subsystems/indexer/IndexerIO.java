package frc.robot.subsystems.indexer;

/**
 * Hardware abstraction interface for the Indexer subsystem.
 *
 * <p>
 * The Indexer consists of two Krakon X44 motors: one driving the Side Roller
 * and one driving
 * the Straight Roller. Both rollers work in concert to transfer game pieces
 * through the internal
 * path.
 *
 * <p>
 * <strong>Design Pattern</strong>: IO Layer abstraction separates control logic
 * from hardware
 * specifics, enabling unit testing without physical hardware.
 *
 * @see IndexerIOReal
 */
public interface IndexerIO {

    /**
     * Container for all indexer sensor inputs.
     *
     * <p>
     * Updated by {@link #updateInputs(IndexerIOInputs)} each control cycle.
     */
    public static class IndexerIOInputs {
        // --- Side Roller Motor ---
        /** Side roller motor velocity in rotations per second. */
        public double sideRollerVelocityRotPerSec;

        /** Side roller motor applied voltage. */
        public double sideRollerAppliedVolts;

        /** Side roller motor supply current in Amps. */
        public double sideRollerCurrentAmps;

        /** Side roller motor CAN connection status (true = OK, false = error). */
        public boolean sideRollerConnected = true;

        // --- Straight Roller Motor ---
        /** Straight roller motor velocity in rotations per second. */
        public double straightRollerVelocityRotPerSec;

        /** Straight roller motor applied voltage. */
        public double straightRollerAppliedVolts;

        /** Straight roller motor supply current in Amps. */
        public double straightRollerCurrentAmps;

        /** Straight roller motor CAN connection status (true = OK, false = error). */
        public boolean straightRollerConnected = true;
    }

    /**
     * Updates the inputs object with the latest hardware data.
     *
     * <p>
     * Called once per control cycle. Implementation should refresh CAN status
     * signals and populate
     * the inputs struct.
     *
     * @param inputs The inputs container to populate.
     */
    public default void updateInputs(IndexerIOInputs inputs) {
    }

    /**
     * Sets the voltage output for both indexer motors.
     *
     * <p>
     * Both the Side Roller and Straight Roller will receive the same voltage
     * command to ensure
     * synchronized operation.
     *
     * @param volts Voltage command (-12 to +12).
     */
    public default void setVoltage(double volts) {
    }

    /** Stops both indexer motors by setting output to zero. */
    public default void stop() {
    }

    // --- Active Connection Check Methods ---

    /**
     * Reads firmware version from the side roller motor.
     *
     * <p>
     * Active query - performs blocking CAN read. Only call during init.
     *
     * @return Firmware version integer (0 = read failed).
     */
    public default int getSideRollerFirmwareVersion() {
        return 0;
    }

    /**
     * Reads firmware version from the straight roller motor.
     *
     * <p>
     * Active query - performs blocking CAN read. Only call during init.
     *
     * @return Firmware version integer (0 = read failed).
     */
    public default int getStraightRollerFirmwareVersion() {
        return 0;
    }
}
