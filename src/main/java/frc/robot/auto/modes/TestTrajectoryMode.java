package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveTrajectoryAction;
import frc.robot.auto.trajectory.Trajectory;
import frc.robot.auto.trajectory.TrajectoryReader;
import java.io.IOException;

/**
 * Test autonomous mode for trajectory following.
 *
 * <p>Loads a trajectory from a CSV file and executes it. This is the simplest autonomous routine
 * and serves as a template for more complex modes.
 *
 * <p><strong>File Location</strong>: Trajectories are loaded from {@code deploy/paths/*.csv}.
 */
public class TestTrajectoryMode extends AutoModeBase {
  private final String mTrajectoryFileName;

  /**
   * Creates a test trajectory mode with a specific file.
   *
   * @param trajectoryFileName CSV file name (without path, e.g., "RealTest.csv").
   */
  public TestTrajectoryMode(String trajectoryFileName) {
    mTrajectoryFileName = trajectoryFileName;
  }

  /** Creates a test trajectory mode with the default file. */
  public TestTrajectoryMode() {
    this("RealTest.csv");
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    System.out.println("[TestTrajectoryMode] Starting autonomous routine");

    try {
      // Load trajectory from CSV
      Trajectory trajectory = TrajectoryReader.fromDeploy(mTrajectoryFileName);
      System.out.println(
          "[TestTrajectoryMode] Loaded trajectory: "
              + mTrajectoryFileName
              + ", duration: "
              + trajectory.getTotalTimeSeconds()
              + "s");

      // Execute trajectory (reset odometry to start pose)
      runAction(new DriveTrajectoryAction(trajectory, true));

      System.out.println("[TestTrajectoryMode] Trajectory completed!");

    } catch (IOException e) {
      System.err.println("[TestTrajectoryMode] Failed to load trajectory: " + e.getMessage());
      e.printStackTrace();
    }
  }

  @Override
  public void done() {
    System.out.println("[TestTrajectoryMode] Autonomous routine finished");
  }
}
