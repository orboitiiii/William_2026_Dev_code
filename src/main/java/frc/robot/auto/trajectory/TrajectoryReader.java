package frc.robot.auto.trajectory;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * CSV trajectory file reader.
 *
 * <p>Loads pre-generated trajectory data from CSV files stored in the deploy directory.
 *
 * <p><strong>Expected CSV Format</strong>:
 *
 * <pre>
 * time_s,x_m,y_m,yaw_rad,vx_mps,vy_mps,omega_rads
 * 0.0000,2.0000,2.0000,0.0000,0.0000,0.0000,0.0000
 * 0.0200,2.0063,2.0000,0.0000,0.0800,0.0000,0.0000
 * ...
 * </pre>
 *
 * <p><strong>Column Definitions</strong>:
 *
 * <ul>
 *   <li><strong>time_s</strong>: Time in seconds from trajectory start
 *   <li><strong>x_m</strong>: X position in meters (field coordinates)
 *   <li><strong>y_m</strong>: Y position in meters
 *   <li><strong>yaw_rad</strong>: Heading in radians
 *   <li><strong>vx_mps</strong>: X velocity in m/s
 *   <li><strong>vy_mps</strong>: Y velocity in m/s
 *   <li><strong>omega_rads</strong>: Angular velocity in rad/s
 * </ul>
 *
 * <p><strong>File Location</strong>:
 *
 * <ul>
 *   <li>RoboRIO: {@code /home/lvuser/deploy/paths/}
 *   <li>Development/Simulation: {@code src/main/deploy/paths/}
 * </ul>
 */
public class TrajectoryReader {

  /** Subfolder within deploy directory for trajectory files. */
  private static final String PATHS_FOLDER = "paths";

  private TrajectoryReader() {
    // Utility class, no instantiation
  }

  /**
   * Loads a trajectory from the deploy directory.
   *
   * @param filename CSV file name (e.g., "RealTest.csv").
   * @return The parsed Trajectory.
   * @throws IOException If the file cannot be read or has invalid format.
   */
  public static Trajectory fromDeploy(String filename) throws IOException {
    Path filePath =
        Filesystem.getDeployDirectory().toPath().resolve(PATHS_FOLDER).resolve(filename);
    return fromPath(filePath);
  }

  /**
   * Loads a trajectory from an absolute path.
   *
   * @param filePath Full path to the CSV file.
   * @return The parsed Trajectory.
   * @throws IOException If the file cannot be read or has invalid format.
   */
  public static Trajectory fromPath(Path filePath) throws IOException {
    List<TrajectoryPoint> points = new ArrayList<>();

    try (BufferedReader reader = Files.newBufferedReader(filePath)) {
      String line;
      boolean headerSkipped = false;

      while ((line = reader.readLine()) != null) {
        line = line.trim();

        // Skip empty lines
        if (line.isEmpty()) {
          continue;
        }

        // Skip header row
        if (!headerSkipped) {
          headerSkipped = true;
          // Validate header format
          if (!line.startsWith("time")) {
            throw new IOException("Invalid CSV header: expected 'time_s,...', got: " + line);
          }
          continue;
        }

        // Parse data row
        TrajectoryPoint point = parseLine(line);
        points.add(point);
      }
    }

    if (points.isEmpty()) {
      throw new IOException("CSV file contains no data points: " + filePath);
    }

    System.out.println(
        "[TrajectoryReader] Loaded trajectory with "
            + points.size()
            + " points, duration: "
            + points.get(points.size() - 1).timeSeconds()
            + "s");

    return Trajectory.fromPoints(points);
  }

  /**
   * Parses a single CSV data line.
   *
   * @param line The CSV row string.
   * @return The parsed TrajectoryPoint.
   * @throws IOException If the format is invalid.
   */
  private static TrajectoryPoint parseLine(String line) throws IOException {
    String[] parts = line.split(",");

    if (parts.length < 7) {
      throw new IOException("Invalid CSV line (expected 7 columns): " + line);
    }

    try {
      double time = Double.parseDouble(parts[0].trim());
      double x = Double.parseDouble(parts[1].trim());
      double y = Double.parseDouble(parts[2].trim());
      double yaw = Double.parseDouble(parts[3].trim());
      double vx = Double.parseDouble(parts[4].trim());
      double vy = Double.parseDouble(parts[5].trim());
      double omega = Double.parseDouble(parts[6].trim());

      return TrajectoryPoint.fromCsvRow(time, x, y, yaw, vx, vy, omega);
    } catch (NumberFormatException e) {
      throw new IOException("Failed to parse numeric values in line: " + line, e);
    }
  }
}
