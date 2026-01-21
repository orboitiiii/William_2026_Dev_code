package frc.robot.framework;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * System Identification (SysId) test routine executor.
 *
 * <p>This class implements the two standard SysId test types:
 *
 * <ul>
 *   <li><strong>Quasistatic</strong>: Voltage ramps slowly from 0 to max, measuring the system's
 *       response to find kS (static friction) and kV (velocity gain).
 *   <li><strong>Dynamic</strong>: Step input to a fixed voltage, measuring acceleration to find kA
 *       (acceleration gain).
 * </ul>
 *
 * <p><strong>Data Output</strong>: Test results are saved to CSV files via {@link CSVLogWriter} for
 * offline analysis with WPILib's SysId tool.
 *
 * <p><strong>Usage Pattern</strong>:
 *
 * <pre>{@code
 * // Setup
 * SysIdRoutine routine = new SysIdRoutine(
 *     new SysIdRoutine.Config()
 *         .setSubsystemName("Drive")
 *         .setRampRate(Volts.of(1.0))
 *         .setTimeout(Seconds.of(10.0)));
 *
 * // In test mode
 * if (startTest) {
 *     routine.start(TestType.QUASISTATIC, Direction.FORWARD);
 * }
 *
 * // In periodic
 * routine.update(timestamp, position, velocity);
 * motor.setVoltage(routine.getOutputVoltage());
 *
 * // On button release
 * routine.stop();
 * }</pre>
 *
 * <p><strong>Safety</strong>: A hard timeout automatically stops the test to prevent mechanism
 * damage.
 *
 * @see CSVLogWriter
 */
public class SysIdRoutine {

  /** Type of system identification test. */
  public enum TestType {
    /** Slow voltage ramp to measure static friction and velocity gain. */
    QUASISTATIC,
    /** Step input to measure acceleration gain. */
    DYNAMIC
  }

  /** Test direction. */
  public enum Direction {
    /** Positive voltage/direction. */
    FORWARD(1.0),
    /** Negative voltage/direction. */
    REVERSE(-1.0);

    /** Multiplier applied to output voltage. */
    public final double multiplier;

    Direction(double multiplier) {
      this.multiplier = multiplier;
    }
  }

  /** Container for a single data sample. */
  public static class DataPoint {
    /** FPGA timestamp in seconds. */
    public final double timestamp;

    /** Applied voltage. */
    public final double voltage;

    /** Mechanism position (units depend on subsystem). */
    public final double position;

    /** Mechanism velocity (units depend on subsystem). */
    public final double velocity;

    /**
     * Creates a data point.
     *
     * @param t Timestamp.
     * @param v Voltage.
     * @param p Position.
     * @param vel Velocity.
     */
    public DataPoint(double t, double v, double p, double vel) {
      this.timestamp = t;
      this.voltage = v;
      this.position = p;
      this.velocity = vel;
    }
  }

  /**
   * Configuration for a SysId routine.
   *
   * <p>Uses a fluent builder pattern for easy configuration.
   */
  public static class Config {
    /** Name of the subsystem being characterized. */
    public String subsystemName = "Unknown";

    /** Voltage ramp rate for quasistatic tests (V/s). */
    public double rampRateVoltsPerSec = 1.0;

    /** Fixed voltage for dynamic tests. */
    public double stepVoltageVolts = 4.0;

    /** Maximum test duration (safety limit). */
    public double timeoutSeconds = 10.0;

    /** Logger for output data. */
    public CSVLogWriter logWriter;

    /**
     * Sets the subsystem name.
     *
     * @param name Subsystem identifier.
     * @return This config for chaining.
     */
    public Config setSubsystemName(String name) {
      this.subsystemName = name;
      return this;
    }

    /**
     * Sets the quasistatic ramp rate.
     *
     * @param voltsPerSecond Ramp rate in volts per second.
     * @return This config for chaining.
     */
    public Config setRampRate(Voltage voltsPerSecond) {
      this.rampRateVoltsPerSec = voltsPerSecond.in(Volts);
      return this;
    }

    /**
     * Sets the dynamic step voltage.
     *
     * @param volts Step voltage magnitude.
     * @return This config for chaining.
     */
    public Config setStepVoltage(Voltage volts) {
      this.stepVoltageVolts = volts.in(Volts);
      return this;
    }

    /**
     * Sets the safety timeout.
     *
     * @param time Maximum test duration.
     * @return This config for chaining.
     */
    public Config setTimeout(Time time) {
      this.timeoutSeconds = time.in(Seconds);
      return this;
    }

    /**
     * Sets the log writer for data output.
     *
     * @param writer CSV log writer.
     * @return This config for chaining.
     */
    public Config setLogWriter(CSVLogWriter writer) {
      this.logWriter = writer;
      return this;
    }
  }

  // State
  private boolean mIsActive = false;
  private TestType mCurrentType = TestType.QUASISTATIC;
  private Direction mCurrentDirection = Direction.FORWARD;
  private double mStartTime = 0.0;
  private double mOutputVoltage = 0.0;

  private final List<DataPoint> mDataBuffer = new ArrayList<>(2000);
  private final Config mConfig;

  /**
   * Creates a SysId routine with the given configuration.
   *
   * @param config Test configuration.
   */
  public SysIdRoutine(Config config) {
    this.mConfig = config;
    if (mConfig.logWriter == null) {
      mConfig.logWriter = new CSVLogWriter("sysid_" + mConfig.subsystemName);
    }
  }

  /**
   * Starts a characterization test.
   *
   * @param type Test type (quasistatic or dynamic).
   * @param direction Positive or negative direction.
   */
  public void start(TestType type, Direction direction) {
    mIsActive = true;
    mCurrentType = type;
    mCurrentDirection = direction;
    mStartTime = Timer.getFPGATimestamp();
    mOutputVoltage = 0.0;
    mDataBuffer.clear();

    System.out.println(
        "[SysId] Started " + type + " " + direction + " for " + mConfig.subsystemName);
  }

  /**
   * Updates the test and records data.
   *
   * <p>Call this every control cycle while the test is active.
   *
   * @param timestamp FPGA timestamp.
   * @param position Current mechanism position.
   * @param velocity Current mechanism velocity.
   */
  public void update(double timestamp, double position, double velocity) {
    if (!mIsActive) return;

    double timeElapsed = timestamp - mStartTime;

    // Safety timeout
    if (timeElapsed > mConfig.timeoutSeconds) {
      stop();
      return;
    }

    // Calculate target voltage
    double targetVolts = 0.0;
    if (mCurrentType == TestType.QUASISTATIC) {
      targetVolts = timeElapsed * mConfig.rampRateVoltsPerSec;
    } else if (mCurrentType == TestType.DYNAMIC) {
      targetVolts = mConfig.stepVoltageVolts;
    }

    // Apply direction
    mOutputVoltage = targetVolts * mCurrentDirection.multiplier;

    // Record data
    mDataBuffer.add(new DataPoint(timestamp, mOutputVoltage, position, velocity));
  }

  /**
   * Returns the current voltage command.
   *
   * <p>Apply this to the motor controller.
   *
   * @return Output voltage, or 0 if not active.
   */
  public double getOutputVoltage() {
    return mIsActive ? mOutputVoltage : 0.0;
  }

  /**
   * Returns whether a test is currently running.
   *
   * @return True if active.
   */
  public boolean isActive() {
    return mIsActive;
  }

  /** Stops the test and saves data to CSV. */
  public void stop() {
    if (!mIsActive) return;

    mIsActive = false;
    mOutputVoltage = 0.0;
    System.out.println(
        "[SysId] Stopping "
            + mConfig.subsystemName
            + "... Saving "
            + mDataBuffer.size()
            + " points.");

    flushToDisk();
  }

  /** Writes collected data to a CSV file. */
  private void flushToDisk() {
    if (mConfig.logWriter == null) return;

    String uniqueName =
        String.format(
            "sysid_%s_%s_%s",
            mConfig.subsystemName, mCurrentType.toString(), mCurrentDirection.toString());

    mConfig.logWriter.writeBatch(uniqueName, mDataBuffer);
  }
}
