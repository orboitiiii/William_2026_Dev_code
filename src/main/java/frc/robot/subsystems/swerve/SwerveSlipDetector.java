package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Slip detection and prevention for swerve drive.
 *
 * <p>This utility class determines if the commanded module accelerations exceed what tire friction
 * can provide, and computes a scale factor to bring all accelerations within the traction limit.
 *
 * <p><strong>Physics Derivation</strong>:
 *
 * <pre>
 * The maximum friction force a tire can exert is:
 *   F_max = μ * N = μ * (m/4) * g
 *
 * where:
 *   μ = coefficient of friction (Constants.Swerve.kCoF)
 *   m = robot mass (Constants.Swerve.kRobotMass)
 *   g = gravitational acceleration (9.81 m/s²)
 *
 * The maximum achievable acceleration for each wheel is:
 *   a_max = F_max / (m/4) = μ * g ≈ 10.8 m/s² for μ = 1.1
 *
 * If any wheel's commanded acceleration exceeds this, it will slip,
 * causing unpredictable robot motion.
 * </pre>
 *
 * <p><strong>Algorithm</strong>: The scale factor is computed as:
 *
 * <pre>
 * scale = min(1, a_max / max_commanded_accel)
 * </pre>
 *
 * @see Constants.Swerve#kMaxDriveAcceleration
 */
public class SwerveSlipDetector {

  /**
   * Computes the slip prevention scale factor.
   *
   * <p>Returns 1.0 if no scaling is needed (all accelerations within limits), or a value less than
   * 1.0 to proportionally reduce all accelerations.
   *
   * <p><strong>Physics Rationale</strong>: We limit acceleration as a scalar (speed change rate),
   * not as a 2D vector. The wheel direction is a kinematic constraint determined by the swerve
   * kinematics solver and is not subject to traction limits. Only the magnitude of velocity change
   * (i.e., how fast the wheel speeds up or slows down) is limited by tire friction.
   *
   * @param prevSetpoints Previous cycle's module states (for dt derivative).
   * @param idealStates Desired module states for this cycle.
   * @param dt Time step between cycles (seconds).
   * @return Scale factor in range (0, 1].
   */
  public static double getSlipScalingFactor(
      SwerveModuleState[] prevSetpoints, SwerveModuleState[] idealStates, double dt) {
    double maxRequiredAccel = 0.0;

    for (int i = 0; i < 4; i++) {
      // Calculate acceleration as scalar: rate of change of wheel speed
      double speedCurrent = prevSetpoints[i].speedMetersPerSecond;
      double speedTarget = idealStates[i].speedMetersPerSecond;
      double accel = Math.abs((speedTarget - speedCurrent) / dt);

      if (accel > maxRequiredAccel) {
        maxRequiredAccel = accel;
      }
    }

    // Apply scaling if any module exceeds the traction limit
    if (maxRequiredAccel > Constants.Swerve.kMaxDriveAcceleration) {
      return Constants.Swerve.kMaxDriveAcceleration / maxRequiredAccel;
    }

    return 1.0;
  }
}
