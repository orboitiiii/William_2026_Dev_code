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
import frc.robot.auto.trajectory.Trajectory;
import frc.robot.auto.trajectory.TrajectoryReader;
import java.io.IOException;

/**
 * Left Side autonomous routine.
 *
 * <p><strong>Sequence</strong>:
 *
 * <ol>
 *   <li>Shoot + Intake active for 1.75 seconds.
 *   <li>Deploy Intake.
 *   <li>Follow LeftSideTake.csv with intake active and passing continuously.
 *   <li>Follow LeftSideTakeToShoot.csv to shoot position, and hold score for 2.5s.
 *   <li>Follow LeftSideSwpieAgain.csv to intake more game pieces.
 *   <li>Follow LeftSideDoubleSwipeToShoot.csv back to shoot position.
 *   <li>Hold Shoot + Intake until end of auto limit.
 * </ol>
 *
 * <p>Subsystem behavior is controlled entirely through {@link GlobalData} fields.
 */
public class LeftSideMode extends AutoModeBase {

  // Trajectory references allocated at instantiation (Zero-Allocation Principle
  // during execution)
  private Trajectory leftSideTakePath;
  private Trajectory leftSideTakeToShootPath;
  private Trajectory leftSideSwipeAgainPath;
  private Trajectory leftSideDoubleSwipeToShootPath;

  // Pre-allocation of Action objects to eliminate GC latency during the routine
  // sequence execution
  // Memory is allocated strictly during initialization phase to avoid
  // Stop-the-world Jitter
  private final SetStateAction scoreAndIntakeAction =
      new SetStateAction(
          () -> {
            // [FAIL-SAFE] Explicitly setting states for simultaneous score and intake
            GlobalData.robotState = RobotState.SCORE;
            GlobalData.hasGamePiece = true; // Signals the internal StateMachine
            GlobalData.pivotWantsDown = true;
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
            GlobalData.autoIntakeVoltage = 8.0;
          });

  private final SetStateAction deployIntakeAction =
      new SetStateAction(
          () -> {
            // [FAIL-SAFE] Explicitly clearing hasGamePiece to ensure we don't prematurely
            // trigger SCORE sequence in sub-components before actual acquisition.
            GlobalData.robotState = RobotState.TRAVEL;
            GlobalData.hasGamePiece = false;
            GlobalData.pivotWantsDown = true;
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
            GlobalData.autoIntakeVoltage = 5.0;
          });

  private final SetStateAction deployIntakeSlowAction =
      new SetStateAction(
          () -> {
            // [FAIL-SAFE] Explicitly clearing hasGamePiece to ensure we don't prematurely
            // trigger SCORE sequence in sub-components before actual acquisition.
            GlobalData.robotState = RobotState.TRAVEL;
            GlobalData.hasGamePiece = false;
            GlobalData.pivotWantsDown = true;
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
            GlobalData.autoIntakeVoltage = 3.0;
          });

  private final SetStateAction deployToShootAction =
      new SetStateAction(
          () -> {
            // [FAIL-SAFE] Explicitly clearing hasGamePiece to ensure we don't prematurely
            // trigger SCORE sequence in sub-components before actual acquisition.
            GlobalData.robotState = RobotState.TRAVEL;
            GlobalData.hasGamePiece = false;
            GlobalData.pivotWantsDown = true;
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
            GlobalData.autoIntakeVoltage = 0.0;
          });

  // private final SetStateAction passAndIntakeAction = new SetStateAction(
  // () -> {
  // // Continuous passing
  // GlobalData.robotState = RobotState.TRAVEL;
  // GlobalData.hasGamePiece = true; // Let state machine know we are processing
  // it
  // GlobalData.pivotWantsDown = true;
  // GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
  // GlobalData.autoIntakeVoltage = 6.0;
  // });

  private final SetStateAction cleanupAction =
      new SetStateAction(
          () -> {
            // [ENERGY ZERO STATE] Minimum potential energy state at the end of autonomous
            GlobalData.robotState = RobotState.TRAVEL;
            GlobalData.pivotWantsDown = false;
            GlobalData.hasGamePiece = false;
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
            GlobalData.autoIntakeVoltage = 6.0;
          });

  public LeftSideMode() {
    try {
      leftSideTakePath = TrajectoryReader.fromDeploy("LeftSideTake.csv");
      leftSideTakeToShootPath = TrajectoryReader.fromDeploy("LeftSideTakeToShoot.csv");
      leftSideSwipeAgainPath = TrajectoryReader.fromDeploy("LeftSideSwpieAgain.csv");
      leftSideDoubleSwipeToShootPath =
          TrajectoryReader.fromDeploy("LeftSideDoubleSwipeToShoot.csv");
    } catch (IOException e) {
      System.err.println("[9427-AUTO] Failed to load trajectory: " + e.getMessage());
      e.printStackTrace(); // Must not fail silently regarding filesystem IO issues
    }
  }

  @Override
  public Pose2d getInitialPose() {
    // Coordinates sourced from LeftSideTake.csv row 0 (2026-03-06)
    return new Pose2d(4.0865, 7.2301, Rotation2d.fromRadians(3.1411));
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    if (leftSideTakePath == null
        || leftSideTakeToShootPath == null
        || leftSideSwipeAgainPath == null
        || leftSideDoubleSwipeToShootPath == null) {
      System.err.println("[9427-AUTO] Trajectories failed to initialize. Aborting routine.");
      return;
    }

    // ── Phase 1: Shoot + Intake Down + Intake Spin for 2.0seconds ──
    runAction(scoreAndIntakeAction);
    runAction(new WaitAction(2.0));

    // ── Phase 2: Deploy Intake ──
    runAction(deployIntakeAction);

    // ── Phase 3: Drive LeftSideTake.csv with Intake and PASS active ──
    // runAction(passAndIntakeAction);
    runAction(new DriveTrajectoryAction(leftSideTakePath, false));

    // ── Phase 4: Drive LeftSideTakeToShoot.csv (Travel + Intake), then SCORE ──
    runAction(deployToShootAction);
    runAction(new DriveTrajectoryAction(leftSideTakeToShootPath, false));

    runAction(scoreAndIntakeAction);
    runAction(new WaitAction(3.10));

    // ── Phase 5: Drive LeftSideSwpieAgain.csv ──
    runAction(deployIntakeSlowAction);
    runAction(new DriveTrajectoryAction(leftSideSwipeAgainPath, false));

    // ── Phase 6: Drive LeftSideDoubleSwipeToShoot.csv, then SCORE ──
    runAction(new DriveTrajectoryAction(leftSideDoubleSwipeToShootPath, false));
    runAction(scoreAndIntakeAction);

    // ── Phase 7: Continuous Shoot + Intake Down/Spin until Match Time Depletes ──
    runAction(new frc.robot.auto.actions.WaitUntilMatchTimeAction(0.5));

    // ── Phase 8: Fail-safe Cleanup ──
    // Force system into a zero-energy safe configuration right before Teleop takes
    // over.
    runAction(cleanupAction);
  }
}
