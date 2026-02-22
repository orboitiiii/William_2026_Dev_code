package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/** Utility class for Phoenix 6 devices. */
public class Phoenix6Util {

  /**
   * Attempts to run a Phoenix 6 command (like applyConfig) with retries.
   *
   * @param function The command to run, returning a StatusCode.
   * @param context A string description for error logging (e.g., "TalonFX 4 Config").
   * @return true if successful, false if all retries failed.
   */
  public static boolean checkManeuver(Supplier<StatusCode> function, String context) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = function.get();
      if (status.isOK()) {
        return true;
      }
      // Small delay before retry to allow bus to settle
      Timer.delay(0.1);
    }
    System.err.println(
        "[Phoenix6Util] CRITICAL: "
            + context
            + " FAILED after 5 attempts. Last Error: "
            + status.toString());
    return false;
  }
}
