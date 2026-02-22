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
    try {
      Trajectory trajectory = TrajectoryReader.fromDeploy(mTrajectoryFileName);
      runAction(new DriveTrajectoryAction(trajectory, true));
    } catch (IOException e) {
      // Critical error: trajectory file not found — cannot recover
      e.printStackTrace();
    }
  }

  @Override
  public void done() {}
}
