package frc.robot;

/**
 * Centralized robot state determination (Orbit 1690 Pattern).
 *
 * <p>
 * This class contains the single {@link #getRobotState()} function that
 * determines the robot's
 * behavioral state based on joystick inputs, sensor data, and game conditions.
 *
 * <p>
 * The function is a large if-statement chain with returns, following 1690's
 * pattern:
 * higher-priority states are checked first, with {@link RobotState#TRAVEL} as
 * the default fallback.
 *
 * <p>
 * <strong>Priority Order</strong> (highest first):
 *
 * <ol>
 * <li>CALIBRATE - Calibration mode button
 * <li>CLIMB - Climbing button (hard to exit once entered)
 * <li>SCORE - Scoring button + game piece present
 * <li>PASS - Passing button
 * <li>INTAKE - Intake button + no game piece
 * <li>TRAVEL - Default fallback
 * </ol>
 */
public class StateResolver {

  private StateResolver() {
  } // Prevent instantiation

  // Minimum cycles to hold SCORE/PASS before allowing a transition back to
  // TRAVEL. Prevents single-cycle glitches (e.g. poseTrusted flicker,
  // isValid edge-case) from interrupting an active shot sequence.
  // [2026-03] Increased from 1 to 5 (100ms) to ensure mechanism readiness isn't
  // aborted by vision noise.
  private static final int MIN_HOLD_CYCLES = 5;
  private static int sHoldCounter = 0;
  private static RobotState sHeldState = RobotState.TRAVEL;

  /**
   * Determines the current robot state based on operator inputs and game
   * conditions.
   *
   * <p>
   * Called once per control cycle. The result is written to
   * {@link GlobalData#robotState}.
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

    boolean scoreActionPressed = control.getCrossButton();
    wantsToPass = control.getTriangleButton();

    if (scoreActionPressed) {
      wantsToScore = true;
    }

    boolean poseTrusted = frc.robot.subsystems.RobotStateEstimator.getInstance().isPoseTrusted();

    // Auto-shoot gate: pose trusted + drift within budget.
    // Mechanism readiness (Turret/Hood at goal) is checked downstream by Indexer
    // before feeding the ball — NOT here, to avoid circular dependency with SCORE
    // state.
    boolean autoShootReady = poseTrusted;
    boolean practiceMode = frc.robot.utils.HubGameState.isPracticeMode();

    // ── Turret field position for hub center line check ──
    // The ball exits from the turret, so the turret's X determines
    // whether we are in the scoring zone or the passing zone.
    edu.wpi.first.math.geometry.Pose2d srPose = frc.robot.subsystems.RobotStateEstimator.getInstance()
        .getEstimatedPose();
    double srTheta = srPose.getRotation().getRadians();
    double srRtX = frc.robot.Constants.Shot.kRobotToTurret.getX();
    double srRtY = frc.robot.Constants.Shot.kRobotToTurret.getY();
    double turretFieldX = srPose.getX() + srRtX * Math.cos(srTheta) - srRtY * Math.sin(srTheta);
    double turretFieldY = srPose.getY() + srRtX * Math.sin(srTheta) + srRtY * Math.cos(srTheta);
    boolean srIsRed = frc.robot.util.geometry.AllianceFlipUtil.shouldFlip();
    double turretFlippedX = srIsRed ? frc.robot.FieldConstants.fieldLength - turretFieldX : turretFieldX;
    boolean turretPastHub = turretFlippedX > frc.robot.FieldConstants.LinesVertical.hubCenter;

    // Distance from turret to hub (more accurate than robot-center distance)
    double hubX = frc.robot.Constants.getHubCenter().getX();
    double hubY = frc.robot.Constants.getHubCenter().getY();
    double distTurretToHub = Math.hypot(hubX - turretFieldX, hubY - turretFieldY);

    // 2. Auto-Shoot Feedback Loop (Only when operator is NOT overriding)
    if (!wantsToScore && !wantsToPass && !frc.robot.DashboardState.getInstance().isShootOnMoveDisabled()) {
      if (practiceMode) {
        // No FMS / match timer — ignore Hub state, use position-based logic
        if (autoShootReady && !turretPastHub) {
          wantsToScore = true;
        }
      } else if (GlobalData.isHubActive) {
        // Hub is ACTIVE -> Primary Context is SCORING
        double timeOfFlight = frc.robot.subsystems.shooter.ShotTables.timeOfFlightS(distTurretToHub);

        if (autoShootReady && !turretPastHub && GlobalData.hubActiveRemainingTime > timeOfFlight) {
          wantsToScore = true;
        }
      } else {
        // Hub is NOT ACTIVE — check prefire window
        double timeOfFlight = frc.robot.subsystems.shooter.ShotTables.timeOfFlightS(distTurretToHub);
        double timeUntilActive = frc.robot.utils.HubGameState.getTimeUntilHubActive();

        // Prefire: ball arrives after Hub becomes active for our alliance
        boolean prefireViable = poseTrusted && timeUntilActive < timeOfFlight;

        if (prefireViable && autoShootReady && !turretPastHub) {
          wantsToScore = true;
        }
      }
    }

    // Final state selection requires game piece
    if (GlobalData.hasGamePiece) {
      if (wantsToScore) {
        sHeldState = RobotState.SCORE;
        sHoldCounter = MIN_HOLD_CYCLES;
        return RobotState.SCORE;
      }
      if (wantsToPass) {
        sHeldState = RobotState.PASS;
        sHoldCounter = MIN_HOLD_CYCLES;
        return RobotState.PASS;
      }
    }

    // Hold: keep SCORE/PASS for a few cycles to ride out momentary glitches
    if (sHoldCounter > 0 && GlobalData.hasGamePiece) {
      sHoldCounter--;
      return sHeldState;
    }

    sHeldState = RobotState.TRAVEL;
    // Default: Travel
    return RobotState.TRAVEL;
  }
}
