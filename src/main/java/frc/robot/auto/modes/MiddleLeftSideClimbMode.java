package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.GlobalData;
import frc.robot.RobotState;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveTrajectoryAction;
import frc.robot.auto.actions.SetStateAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.auto.actions.WaitUntilMatchTimeAction;
import frc.robot.auto.trajectory.Trajectory;
import frc.robot.auto.trajectory.TrajectoryReader;
import java.io.IOException;

/**
 * Middle-Left Side Climb autonomous routine.
 *
 * <p>Dashboard sends the string {@code "MiddleLeftSideClimb"} to select this mode.
 *
 * <p><strong>Sequence</strong>:
 *
 * <ol>
 *   <li>Shoot + Intake active for 2.0 seconds.
 *   <li>Follow MiddleLeftSideClimb.csv to climb position.
 *   <li>Hold until match time depletes.
 *   <li>Fail-safe cleanup to zero-energy state.
 * </ol>
 */
public class MiddleLeftSideClimbMode extends AutoModeBase {

  // Trajectory allocated at instantiation (Zero-Allocation Principle during
  // execution)
  private Trajectory middleLeftSideClimbPath;

  // Pre-allocated action objects to eliminate GC latency during the routine
  private final SetStateAction scoreAndIntakeAction =
      new SetStateAction(
          () -> {
            // [FAIL-SAFE] Explicitly set states for simultaneous score and intake
            GlobalData.robotState = RobotState.SCORE;
            GlobalData.hasGamePiece = true; // Signals the internal StateMachine
            GlobalData.pivotWantsDown = true;
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
            GlobalData.autoIntakeVoltage = 8.0;
          });

  private final SetStateAction deployIntakeAction =
      new SetStateAction(
          () -> {
            // [FAIL-SAFE] Transition to travel, deploy intake for path following
            GlobalData.robotState = RobotState.TRAVEL;
            GlobalData.hasGamePiece = false;
            GlobalData.pivotWantsDown = true;
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
            GlobalData.autoIntakeVoltage = 5.0;
          });

  private final SetStateAction cleanupAction =
      new SetStateAction(
          () -> {
            // [ENERGY ZERO STATE] Minimum potential energy state at the end of autonomous
            GlobalData.robotState = RobotState.TRAVEL;
            GlobalData.pivotWantsDown = false;
            GlobalData.hasGamePiece = false;
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.IDLE;
            GlobalData.autoIntakeVoltage = 0.0;
          });

  public MiddleLeftSideClimbMode() {
    try {
      middleLeftSideClimbPath = TrajectoryReader.fromDeploy("MiddleLeftSideClimb.csv");
    } catch (IOException e) {
      System.err.println("[9427-AUTO] Failed to load trajectory: " + e.getMessage());
      e.printStackTrace(); // Must not fail silently regarding filesystem IO issues
    }
  }

  /**
   * Returns the initial field pose for odometry reset.
   *
   * <p>Coordinates sourced from MiddleLeftSideClimb.csv row 0 (2026-03-06).
   *
   * @return Starting pose (x, y in metres; heading in radians).
   */
  @Override
  public Pose2d getInitialPose() {
    // Coordinates sourced from MiddleLeftSideClimb.csv row 0 (2026-03-06)
    return new Pose2d(3.6246, 4.0352, Rotation2d.fromRadians(0.0));
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    if (middleLeftSideClimbPath == null) {
      System.err.println("[9427-AUTO] Trajectory failed to initialize. Aborting routine.");
      return;
    }

    // ── Phase 1: Shoot + Intake for 2.0 seconds ──
    runAction(scoreAndIntakeAction);
    runAction(new WaitAction(2.0));

    // ── Phase 2: Deploy Intake ──
    runAction(deployIntakeAction);

    // ── Phase 3: Drive MiddleLeftSideClimb.csv ──
    runAction(new DriveTrajectoryAction(middleLeftSideClimbPath, false));

    // ── Phase 4: Hold until match time depletes ──
    runAction(new WaitUntilMatchTimeAction(0.5));

    // ── Phase 5: Fail-safe Cleanup ──
    // Force system into zero-energy safe configuration before Teleop takes over.
    runAction(cleanupAction);
  }
}
