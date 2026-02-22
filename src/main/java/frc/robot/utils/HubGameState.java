package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/**
 * Tracks the FRC 2026 Hub Active / Inactive states based on match time shifts.
 *
 * <p>Rule implementation: The Hub shifts between alliances during Teleop.
 */
public class HubGameState {

  private HubGameState() {}

  /**
   * Determines if the Hub is currently active and open for scoring for our alliance.
   *
   * @return True if the Hub is active, false otherwise.
   */
  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData == null || gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R':
        redInactiveFirst = true;
        break;
      case 'B':
        redInactiveFirst = false;
        break;
      default:
        // If we have invalid game data, assume hub is active.
        return true;
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = false;
    if (alliance.get() == Alliance.Red) {
      shift1Active = !redInactiveFirst;
    } else if (alliance.get() == Alliance.Blue) {
      shift1Active = redInactiveFirst;
    }

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  /**
   * Returns the remaining time in seconds that the Hub is expected to be in its CURRENT active
   * state.
   *
   * @return Remaining seconds of the current shift, or the rest of the match if in Endgame.
   */
  public static double getHubRemainingActiveTime() {
    if (!DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled()) {
      return 20.0;
    }

    double matchTime = DriverStation.getMatchTime();

    // Safety check - if match time is negative or invalid
    if (matchTime < 0) return 0.0;

    if (matchTime > 130) {
      return matchTime - 130.0;
    } else if (matchTime > 105) {
      return matchTime - 105.0;
    } else if (matchTime > 80) {
      return matchTime - 80.0;
    } else if (matchTime > 55) {
      return matchTime - 55.0;
    } else if (matchTime > 30) {
      return matchTime - 30.0;
    } else {
      return matchTime; // End game, rest of the match
    }
  }
}
