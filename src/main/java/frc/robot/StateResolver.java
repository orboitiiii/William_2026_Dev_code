package frc.robot;

/**
 * Centralized robot state determination (Orbit 1690 Pattern).
 *
 * <p>This class contains the single {@link #getRobotState()} function that determines the robot's
 * behavioral state based on joystick inputs, sensor data, and game conditions.
 *
 * <p>The function is a large if-statement chain with returns, following 1690's pattern:
 * higher-priority states are checked first, with {@link RobotState#TRAVEL} as the default fallback.
 *
 * <p><strong>Priority Order</strong> (highest first):
 *
 * <ol>
 *   <li>CALIBRATE - Calibration mode button
 *   <li>CLIMB - Climbing button (hard to exit once entered)
 *   <li>SCORE - Scoring button + game piece present
 *   <li>PASS - Passing button
 *   <li>INTAKE - Intake button + no game piece
 *   <li>TRAVEL - Default fallback
 * </ol>
 */
public class StateResolver {

  private StateResolver() {} // Prevent instantiation

  /**
   * Determines the current robot state based on operator inputs and game conditions.
   *
   * <p>Called once per control cycle. The result is written to {@link GlobalData#robotState}.
   *
   * @return The determined {@link RobotState} for this cycle.
   */
  public static RobotState getRobotState() {
    var control = ControlBoard.getInstance();

    // ==========================================================
    // PARALLEL TOGGLE STATES (Decoupled from RobotState)
    // ==========================================================

    // 1. Intake Pivot (R1 toggles pivot up/down)
    if (control.getR1ButtonPressed()) {
      GlobalData.pivotWantsDown = !GlobalData.pivotWantsDown;
    }

    // 2. Intake Wheels (Circle = Forward toggle, Square = Reverse toggle)
    if (control.getCircleButtonPressed()) {
      if (GlobalData.intakeWheelWantedState == GlobalData.IntakeActiveState.FORWARD) {
        GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.IDLE;
      } else {
        GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.FORWARD;
      }
    } else if (control.getSquareButtonPressed()) {
      if (GlobalData.intakeWheelWantedState == GlobalData.IntakeActiveState.REVERSE) {
        GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.IDLE;
      } else {
        GlobalData.intakeWheelWantedState = GlobalData.IntakeActiveState.REVERSE;
      }
    }

    // ==========================================================
    // HUB ACTIVE TRACKING
    // ==========================================================
    GlobalData.isHubActive = frc.robot.utils.HubGameState.isHubActive();
    GlobalData.hubActiveRemainingTime = frc.robot.utils.HubGameState.getHubRemainingActiveTime();

    // ==========================================================
    // PRIMARY ROBOT STATE HIERARCHY
    // ==========================================================

    // --- Highest Priority: Calibration Mode ---
    if (control.getCalibrationModeButton()) {
      return RobotState.CALIBRATE;
    }

    // --- Climbing (non-interruptible once committed) ---
    if (control.getClimbButton()) {
      return RobotState.CLIMB;
    }

    // --- Intent Parsing: SCORE vs PASS ---
    boolean wantsToScore = false;
    boolean wantsToPass = false;

    // Manual triggers
    boolean actionPressed = control.getCrossButton();
    if (control.getTriangleButton()) {
      wantsToPass = true;
    }

    boolean poseTrusted = frc.robot.subsystems.RobotStateEstimator.getInstance().isPoseTrusted();

    if (GlobalData.isHubActive) {
      // 1. Hub is ACTIVE -> Primary Context is SCORING
      if (actionPressed) {
        wantsToScore = true;
      } else if (!frc.robot.DashboardState.getInstance().isShootOnMoveDisabled()) {
        // Auto-SCORE condition (Shoot on Move Enabled)
        double distanceToHub =
            frc.robot.Constants.getHubCenter()
                .getTranslation()
                .getDistance(
                    frc.robot.subsystems.RobotStateEstimator.getInstance()
                        .getEstimatedPose()
                        .getTranslation());
        double timeOfFlight = frc.robot.subsystems.shooter.ShotTables.timeOfFlightS(distanceToHub);

        if (poseTrusted && GlobalData.hubActiveRemainingTime > timeOfFlight) {
          wantsToScore = true;
        }
      }
    } else {
      // 2. Hub is NOT ACTIVE -> Primary Context is PASSING
      if (actionPressed) {
        wantsToPass = true;
      } else if (!frc.robot.DashboardState.getInstance().isAutoPassingDisabled()) {
        // Auto-PASS condition (Auto Passing Enabled)
        var shotParams = frc.robot.subsystems.shooter.ShotCalculator.getInstance().calculate();
        if (poseTrusted && shotParams.isValid && shotParams.isPassing) {
          wantsToPass = true;
        }
      }
    }

    // Final state selection requires game piece
    if (GlobalData.hasGamePiece) {
      if (wantsToScore) {
        return RobotState.SCORE;
      }
      if (wantsToPass) {
        return RobotState.PASS;
      }
    }

    // Default: Travel
    return RobotState.TRAVEL;
  }
}
