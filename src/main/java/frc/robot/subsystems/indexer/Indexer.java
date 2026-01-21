package frc.robot.subsystems.indexer;

import frc.robot.Constants;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;

/**
 * Indexer Subsystem - Dual-motor game piece transfer mechanism.
 *
 * <p>
 * This subsystem controls the internal indexer used to transfer game pieces
 * from the intake to
 * the shooter/scoring mechanism. It consists of two Krakon X44 motors:
 *
 * <ul>
 * <li><strong>Side Roller</strong>: Transfers game pieces laterally.
 * <li><strong>Straight Roller</strong>: Transfers game pieces in a linear path.
 * </ul>
 *
 * <p>
 * <strong>Control Strategy</strong>: Uses open-loop voltage control for
 * consistent torque output
 * regardless of battery state. Both motors receive the same voltage command to
 * ensure synchronized
 * operation.
 *
 * <p>
 * <strong>Thread Safety</strong>: All state access is synchronized as the
 * Looper runs on a
 * separate thread from the main robot loop.
 *
 * @see IndexerIO
 */
public class Indexer extends Subsystem {
    private static Indexer mInstance;

    /**
     * Returns the singleton instance.
     *
     * @return The global Indexer instance.
     */
    public static Indexer getInstance() {
        if (mInstance == null) {
            mInstance = new Indexer();
        }
        return mInstance;
    }

    private final IndexerIO mIO;
    private final IndexerIO.IndexerIOInputs mInputs = new IndexerIO.IndexerIOInputs();

    /**
     * Running state: true = rollers running, false = stopped.
     *
     * <p>
     * Synchronized access required as state changes may originate from the main
     * robot thread while
     * the Looper reads state from a separate thread.
     */
    private boolean mIsRunning = false;

    private Indexer() {
        // Use real IO implementation; swap to simulation IO if needed
        mIO = new IndexerIOReal();
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        enabledLooper.register(
                new ILoop() {
                    @Override
                    public void onStart(double timestamp) {
                        synchronized (Indexer.this) {
                            mIsRunning = false;
                        }
                    }

                    @Override
                    public void onLoop(double timestamp) {
                        // State machine logic runs here if needed
                    }

                    @Override
                    public void onStop(double timestamp) {
                        stop();
                    }
                });
    }

    // --- Public API ---

    /**
     * Sets the running state of the indexer.
     *
     * @param running True to run the indexer, false to stop.
     */
    public synchronized void setRunning(boolean running) {
        mIsRunning = running;
    }

    /**
     * Toggles the indexer state.
     *
     * <p>
     * If currently running, stops the indexer. If stopped, starts the indexer.
     */
    public synchronized void toggle() {
        mIsRunning = !mIsRunning;
        System.out.println("[Indexer] Toggle -> " + (mIsRunning ? "RUNNING" : "STOPPED"));
    }

    /**
     * Returns the current running state.
     *
     * @return True if the indexer is running.
     */
    public synchronized boolean isRunning() {
        return mIsRunning;
    }

    // --- Subsystem Interface ---

    @Override
    public void readPeriodicInputs() {
        mIO.updateInputs(mInputs);
    }

    @Override
    public void writePeriodicOutputs() {
        synchronized (this) {
            if (mIsRunning) {
                mIO.setVoltage(Constants.Indexer.kIndexerVoltage);
            } else {
                mIO.stop();
            }
        }
    }

    @Override
    public boolean checkConnectionActive() {
        // Active Query: Read Firmware Version from both motors
        System.out.println("[Indexer] Running Active Connection Check...");

        int sideRollerFw = mIO.getSideRollerFirmwareVersion();
        int straightRollerFw = mIO.getStraightRollerFirmwareVersion();

        boolean allConnected = true;

        if (sideRollerFw == 0) {
            System.err.println("[Indexer] Side Roller Motor Firmware Read Failed!");
            allConnected = false;
        } else {
            System.out.println("[Indexer] Side Roller Motor FW: " + sideRollerFw);
        }

        if (straightRollerFw == 0) {
            System.err.println("[Indexer] Straight Roller Motor Firmware Read Failed!");
            allConnected = false;
        } else {
            System.out.println("[Indexer] Straight Roller Motor FW: " + straightRollerFw);
        }

        return allConnected;
    }

    @Override
    public boolean checkConnectionPassive() {
        // Passive: Check cached status from IO layer
        return mInputs.sideRollerConnected && mInputs.straightRollerConnected;
    }

    @Override
    public boolean checkSanityPassive() {
        // Check for current spike when running (stall detection)
        double totalCurrent = mInputs.sideRollerCurrentAmps + mInputs.straightRollerCurrentAmps;

        if (totalCurrent > 80.0) {
            System.err.println("[Indexer] Total Current Spike (Stall): " + totalCurrent + "A");
            return false;
        }

        // Check individual motor stall conditions
        if (mIsRunning) {
            boolean sideRollerStalled = mInputs.sideRollerVelocityRotPerSec < 1 && mInputs.sideRollerAppliedVolts > 6;
            boolean straightRollerStalled = mInputs.straightRollerVelocityRotPerSec < 1
                    && mInputs.straightRollerAppliedVolts > 6;

            if (sideRollerStalled) {
                System.err.println(
                        "[Indexer] Side Roller Stalled! Current: " + mInputs.sideRollerCurrentAmps + "A");
                return false;
            }
            if (straightRollerStalled) {
                System.err.println(
                        "[Indexer] Straight Roller Stalled! Current: " + mInputs.straightRollerCurrentAmps + "A");
                return false;
            }
        }

        return true;
    }

    @Override
    public void outputTelemetry() {
        // TODO: Publish indexer state to NetworkTables if needed
    }

    @Override
    public void stop() {
        synchronized (this) {
            mIsRunning = false;
        }
        mIO.stop();
    }

    @Override
    public void zeroSensors() {
        // No sensors to zero for this simple mechanism
    }
}
