package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * Takes desired ChassisSpeeds and limits them based on physical constraints (Anti-Slip logic
 * inspired by Orbit 1690).
 */
public class SwerveSetpointGenerator {
  private final SwerveDriveKinematics mKinematics;

  public SwerveSetpointGenerator(SwerveDriveKinematics kinematics) {
    mKinematics = kinematics;
  }

  /**
   * Generates a feasible setpoint that prevents wheel slip.
   *
   * @param currentPose Current robot pose (for field-relative calculations if needed, but mainly
   *     for curvature).
   * @param desiredSpeeds The desired chassis speeds.
   * @return Feasible SwerveModuleStates.
   */
  public SwerveModuleState[] generateSetpoint(ChassisSpeeds desiredSpeeds) {
    // 1. Discretize (optional, but good for high speed)
    // ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(desiredSpeeds, Constants.kLooperDt);
    // For now, let's stick to the anti-slip core.

    ChassisSpeeds limitedSpeeds = limitAntiSlip(desiredSpeeds);

    // 2. Inverse Kinematics
    SwerveModuleState[] states = mKinematics.toSwerveModuleStates(limitedSpeeds);

    // 3. Desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.kMaxDriveVelocity);

    return states;
  }

  /**
   * Limits the chassis speeds to prevent slipping. Logic: - Calculate curvature (kappa) = omega /
   * v_linear - Max feasible velocity for this curvature: v_max = sqrt(mu * g * r) = sqrt(mu * g /
   * kappa) - Cap v_linear to v_max.
   */
  private ChassisSpeeds limitAntiSlip(ChassisSpeeds speeds) {
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    double omega = speeds.omegaRadiansPerSecond;

    double linearVelocity = Math.hypot(vx, vy);

    // If we are not moving or not turning, anti-slip for turning is not the primary concern
    // (though linear accel is, but we assume the driver/profiler handles that mostly.
    // This function focuses on Cornering Limit).
    if (linearVelocity < 1e-3 || Math.abs(omega) < 1e-3) {
      return speeds;
    }

    // Curvature k = omega / v
    // Radius r = 1 / k = v / omega
    double radius = Math.abs(linearVelocity / omega);

    // Max velocity before slipping due to centripetal force
    // F_c = m * v^2 / r <= F_f = mu * m * g
    // v^2 / r <= mu * g
    // v <= sqrt(mu * g * r)
    double maxFeasibleVelocity =
        Math.sqrt(Constants.Swerve.kNormalFrictionCoefficient * 9.81 * radius);

    if (linearVelocity > maxFeasibleVelocity) {
      // Scale down the linear velocity vector to the limit
      double scale = maxFeasibleVelocity / linearVelocity;
      return new ChassisSpeeds(
          vx * scale, vy * scale, omega); // Maintain omega to keep curvature same?
      // If we reduce v, but keep omega, r decreases (r = v/w). Then v_max decreases further.
      // Wait, if we slow down, we can turn strictly TIGHTER radius physically, OR
      // for the SAME radius, we are safer.
      // The constraint is v <= sqrt(mu*g*r).
      // If we satisfy this, we are good.
      // By scaling v down, new v' < v.
      // If we keep omega same, new r' = v'/omega = scale * r.
      // New max v' = sqrt(mu*g*r') = sqrt(mu*g*scale*r) = sqrt(scale) * v_max_old.
      // Is v' <= sqrt(scale) * v_max_old?
      // current v = maxFeasible. v' = scale * maxFeasible.
      // scale * maxFeasible <= sqrt(scale) * maxFeasible -> scale <= sqrt(scale) -> scale^2 <=
      // scale -> scale <= 1. OK.
      // So scaling both v linearly works?

      // Actually, usually we prioritize maintaining the path (curvature).
      // So if we reduce V, we should generally reduce Omega?
      // No, curvature = omega/v. If we scale BOTH vx,vy AND omega by 'scale', then curvature is
      // CONSTANT.
      // Path is preserved.
      // v' = s*v, w' = s*w. r' = (s*v)/(s*w) = v/w = r.
      // Max vel for radius r is still maxFeasibleVelocity.
      // We reduced v to maxFeasibleVelocity.
      // So we are safe.

      // Re-calc:
      // limitedSpeeds = new ChassisSpeeds(vx * scale, vy * scale, omega * scale);
    }

    // HOWEVER, Orbit slides often imply prioritizing Rotation or Translation?
    // Usually, we just cap V.
    // If we cap V but keep Omega, curvature increases (tighter turn).
    // Tighter turn requires LOWER max velocity.
    // It might be recursive.

    // Let's implement the standard approach: "Slow down the robot if it's turning too fast for its
    // speed (or moving too fast for its turn)."
    // We select to clamp Linear Velocity to the max allowed for the CURRENT Radius defined by (V,
    // Omega).
    // If we reduce V ONLY, radius R decreases. The limit gets stricter.

    // Better approach:
    // Limit based on the acceleration budget?
    // Friction circle: a_total^2 <= (mu*g)^2
    // a_total = a_trans + a_centripetal
    // a_centripetal = v*omega.
    // This assumes steady state turning.

    // Let's go with the v <= sqrt(mu*g*r) check and scale V down.
    // If we scale V down, we reduce the centrifugal force.
    // We will maintain Omega? If we maintain Omega, the robot turns "faster" relative to distance
    // traveled.
    // Drivers usually expect Omega to be satisfied?
    // Actually, if we are slipping, we satisfy NEITHER.

    // Decision: Scale BOTH v and omega to preserve curvature (path shape).
    // If the driver commanded a specific arc, we traverse it slower.

    if (linearVelocity > maxFeasibleVelocity) {
      double scale = maxFeasibleVelocity / linearVelocity;
      return new ChassisSpeeds(vx * scale, vy * scale, omega * scale);
    }

    return speeds;
  }
}
