package frc.robot.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.GlobalData;
import frc.robot.RobotState;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetStateAction;
import frc.robot.auto.actions.WaitUntilMatchTimeAction;

/**
 * Only Shoot autonomous routine.
 *
 * <p><strong>Sequence</strong>:
 *
 * <ol>
 *   <li>Continuous shoot until match time ends.
 * </ol>
 */
public class OnlyShootMode extends AutoModeBase {

  private final SetStateAction shootAction =
      new SetStateAction(
          () -> {
            // [FAIL-SAFE] Explicitly setting states for score
            GlobalData.robotState = RobotState.SCORE;
            GlobalData.hasGamePiece = true; // Signals the internal StateMachine
            GlobalData.pivotWantsDown = false; // Need nothing but shooting
            GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.IDLE;
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

  public OnlyShootMode() {}

  @Override
  public Pose2d getInitialPose() {
    // Assuming starting at the subwoofer or somewhere standard, we don't move
    // anyways.
    // Setting default to 0,0,0 as movement isn't expected
    return new Pose2d(0, 0, new Rotation2d(0));
  }

  @Override
  protected void routine() throws AutoModeEndedException {
    // ── Phase 1: Shoot ──
    runAction(shootAction);

    // ── Phase 2: Wait until end ──
    runAction(new WaitUntilMatchTimeAction(0.5));

    // ── Phase 3: Fail-safe Cleanup ──
    runAction(cleanupAction);
  }
}
