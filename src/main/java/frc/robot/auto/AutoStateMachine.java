package frc.robot.auto;

import frc.robot.GlobalData;
import frc.robot.RobotState;

/**
 * Autonomous State Machine (Orbit 1690 Pattern).
 *
 * <p>This class determines the {@link RobotState} during autonomous mode. It runs in the same loop
 * as teleop — the only difference is that state decisions come from auto logic rather than joystick
 * buttons.
 *
 * <p>The auto state machine works alongside the MPC trajectory system:
 *
 * <ul>
 *   <li>Drive is controlled by {@link AutoModeExecutor} for path following
 *   <li>All other subsystems are controlled by {@link GlobalData#robotState}
 * </ul>
 *
 * <p><strong>Usage</strong>: Create an instance in Robot.java, call {@link #getAutoRobotState()}
 * each cycle during autonomous to set {@link GlobalData#robotState}.
 *
 * <p><strong>Customization</strong>: Override or modify the phase transitions in {@link #update()}
 * to match your autonomous strategy. The phases and transitions should be tuned based on the
 * specific game and field layout.
 *
 * @see GlobalData
 * @see RobotState
 */
public class AutoStateMachine {

  /**
   * Autonomous phases that map to robot states.
   *
   * <p>Each phase determines what RobotState the subsystems should operate in. The Drive subsystem
   * is controlled separately via MPC trajectory actions.
   */
  public enum AutoPhase {
    /** Driving to a game piece pickup location. Subsystems prepare for intake. */
    DRIVE_TO_INTAKE,

    /** Actively intaking a game piece. */
    INTAKE,

    /** Driving to a scoring location. Subsystems prepare for scoring. */
    DRIVE_TO_SCORE,

    /** Actively scoring a game piece. */
    SCORE,

    /** Autonomous routine complete. Default travel mode. */
    DONE
  }

  private AutoPhase mCurrentPhase = AutoPhase.DRIVE_TO_INTAKE;

  /**
   * Returns the RobotState corresponding to the current auto phase.
   *
   * <p>Called once per control cycle during autonomous. The result is written to {@link
   * GlobalData#robotState}.
   *
   * @return The RobotState for the current auto phase.
   */
  public RobotState getAutoRobotState() {
    return switch (mCurrentPhase) {
      case DRIVE_TO_INTAKE -> RobotState.INTAKE;
      case INTAKE -> RobotState.INTAKE;
      case DRIVE_TO_SCORE -> RobotState.TRAVEL;
      case SCORE -> RobotState.SCORE;
      case DONE -> RobotState.TRAVEL;
    };
  }

  /**
   * Updates the auto state machine phase based on game conditions.
   *
   * <p>Call this each cycle during autonomous to advance through phases. Phase transitions are
   * based on game piece possession and scoring status.
   */
  public void update() {
    switch (mCurrentPhase) {
      case DRIVE_TO_INTAKE -> {
        // Transition to INTAKE when near the pickup location
        // (placeholder: use path completion or proximity check)
      }
      case INTAKE -> {
        if (GlobalData.hasGamePiece) {
          mCurrentPhase = AutoPhase.DRIVE_TO_SCORE;
        }
      }
      case DRIVE_TO_SCORE -> {
        // Transition to SCORE when near the scoring location
        // (placeholder: use path completion or proximity check)
      }
      case SCORE -> {
        if (!GlobalData.hasGamePiece) {
          mCurrentPhase = AutoPhase.DRIVE_TO_INTAKE;
        }
      }
      case DONE -> {
        // Stay in DONE
      }
    }
  }

  /**
   * Resets the auto state machine to the initial phase.
   *
   * <p>Call this at the start of autonomous.
   */
  public void reset() {
    mCurrentPhase = AutoPhase.DRIVE_TO_INTAKE;
  }

  /**
   * Sets the current auto phase directly.
   *
   * <p>Useful for auto modes that need to skip phases or start from a specific state.
   *
   * @param phase The phase to set.
   */
  public void setPhase(AutoPhase phase) {
    mCurrentPhase = phase;
  }

  /**
   * Returns the current auto phase.
   *
   * @return The current phase.
   */
  public AutoPhase getCurrentPhase() {
    return mCurrentPhase;
  }
}
