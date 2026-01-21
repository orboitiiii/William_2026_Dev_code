package frc.robot.framework;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * CSV logging utility for telemetry and System Identification data.
 *
 * <p>This class provides two logging modes:
 *
 * <ul>
 *   <li><strong>Streaming Mode</strong>: Appends rows incrementally during operation.
 *   <li><strong>Batch Mode</strong>: Writes complete datasets at once (used by SysId).
 * </ul>
 *
 * <p><strong>File Location</strong>: All files are written to the Operating Directory ({@link
 * edu.wpi.first.wpilibj.Filesystem#getOperatingDirectory()}).
 *
 * <p><strong>SysId Data Format</strong>:
 *
 * <pre>
 * Timestamp,Voltage,Position,Velocity
 * 0.000000,0.500000,0.001234,0.012345
 * ...
 * </pre>
 *
 * @see SysIdRoutine
 */
public class CSVLogWriter {
  private PrintWriter mWriter;
  private final String mFileName;
  private final List<String> mLogFields = new ArrayList<>();

  /**
   * Creates a CSV log writer.
   *
   * @param fileName Base file name (without extension).
   */
  public CSVLogWriter(String fileName) {
    mFileName = fileName;
  }

  /**
   * Adds a column header field.
   *
   * <p>Call this before {@link #init()} to define the CSV schema.
   *
   * @param fieldName The column header name.
   */
  public void addField(String fieldName) {
    mLogFields.add(fieldName);
  }

  /**
   * Initializes the log file and writes the header row.
   *
   * <p>Opens the file in append mode. Multiple sessions will be appended.
   */
  public void init() {
    try {
      mWriter = new PrintWriter(new FileWriter(mFileName + ".csv", true));
      mWriter.println(String.join(",", mLogFields));
      mWriter.flush();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /**
   * Writes a data row to the log file.
   *
   * @param values List of values (must match number of fields).
   */
  public void log(List<Object> values) {
    if (mWriter != null) {
      List<String> stringValues = new ArrayList<>();
      for (Object o : values) {
        stringValues.add(o.toString());
      }
      mWriter.println(String.join(",", stringValues));
      mWriter.flush();
    }
  }

  /** Closes the log file. */
  public void close() {
    if (mWriter != null) {
      mWriter.close();
    }
  }

  /**
   * Writes a batch of SysId data points to a timestamped CSV file.
   *
   * <p>Used by {@link SysIdRoutine} to save characterization data.
   *
   * @param fileNamePrefix Prefix for the output file (e.g., "sysid_drive").
   * @param data List of data points to write.
   */
  public void writeBatch(String fileNamePrefix, List<SysIdRoutine.DataPoint> data) {
    long timestamp = System.currentTimeMillis();
    String fullFileName = fileNamePrefix + "_" + timestamp + ".csv";
    File file = new File(Filesystem.getOperatingDirectory(), fullFileName);

    try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
      // Header
      writer.write("Timestamp,Voltage,Position,Velocity");
      writer.newLine();

      // Data rows
      for (SysIdRoutine.DataPoint point : data) {
        String line =
            String.format(
                "%.6f,%.6f,%.6f,%.6f",
                point.timestamp, point.voltage, point.position, point.velocity);
        writer.write(line);
        writer.newLine();
      }

      System.out.println(
          "[CSVLogWriter] Successfully wrote " + data.size() + " rows to " + fullFileName);

    } catch (IOException e) {
      System.err.println("[CSVLogWriter] Error writing batch CSV: " + e.getMessage());
    }
  }

  /**
   * Deletes all SysId log files from the operating directory.
   *
   * <p>Matches files with pattern {@code sysid_*.csv}.
   */
  public static void deleteAllLogs() {
    File dir = Filesystem.getOperatingDirectory();
    File[] files = dir.listFiles((d, name) -> name.startsWith("sysid_") && name.endsWith(".csv"));
    if (files != null) {
      for (File f : files) {
        if (f.delete()) {
          System.out.println("[CSVLogWriter] Deleted " + f.getName());
        } else {
          System.err.println("[CSVLogWriter] Failed to delete " + f.getName());
        }
      }
    }
  }
}
