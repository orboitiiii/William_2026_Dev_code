package frc.robot.libraries.lib9427.profiles;

/**
 * S-Curve (Jerk-Limited) Motion Profile Generator.
 *
 * <p>Generates smooth motion profiles by limiting not just velocity and acceleration, but also jerk
 * (the rate of change of acceleration). This prevents mechanical shocks and reduces voltage spikes
 * during direction changes.
 *
 * <p><strong>Profile Structure</strong> (7 Segments):
 *
 * <pre>
 * Segment 1: Jerk up     (acceleration ramps up)
 * Segment 2: Const accel (acceleration at maximum)
 * Segment 3: Jerk down   (acceleration ramps down to zero)
 * Segment 4: Cruise      (constant velocity)
 * Segment 5: Jerk down   (negative jerk, acceleration becomes negative)
 * Segment 6: Const decel (acceleration at minimum)
 * Segment 7: Jerk up     (acceleration ramps back to zero)
 *
 * Velocity Profile:
 *        ┌───────────┐
 *       /             \
 *      /               \
 *     /                 \
 * ───┘                   └───
 *   t1 t2    t3 t4    t5 t6 t7
 * </pre>
 *
 * <p><strong>Benefits over Trapezoidal Profiles</strong>:
 *
 * <ul>
 *   <li>Smoother acceleration transitions reduce mechanical stress
 *   <li>Lower peak currents due to gradual torque changes
 *   <li>Better tracking with model-based controllers (LQR, etc.)
 * </ul>
 *
 * <p><strong>Kinematics</strong>:
 *
 * <pre>
 * Given jerk j = constant:
 *   a(t) = a₀ + j*t
 *   v(t) = v₀ + a₀*t + (1/2)*j*t²
 *   p(t) = p₀ + v₀*t + (1/2)*a₀*t² + (1/6)*j*t³
 * </pre>
 *
 * @see MotionConstraints
 * @see MotionState
 */
public class SCurveProfile {
  private final MotionConstraints constraints;
  private final double startPos;
  private final double endPos;
  private final double direction;

  // Timing knot points (relative to start time 0)
  private double t1, t2, t3, t4, t5, t6, t7;

  // State at knot points
  private final double[] pKnot = new double[8];
  private final double[] vKnot = new double[8];

  // Applied limits (may differ from constraints for short moves)
  private double aLim;
  private double vLim;
  private double jMax;

  /**
   * Constructs an S-Curve profile.
   *
   * @param constraints Maximum velocity, acceleration, and jerk.
   * @param startPos Starting position.
   * @param endPos Target position.
   * @throws IllegalArgumentException If constraints have non-positive values.
   */
  public SCurveProfile(MotionConstraints constraints, double startPos, double endPos) {
    if (constraints.maxJerk <= 1e-9)
      throw new IllegalArgumentException("Max Jerk must be positive.");
    if (constraints.maxAcceleration <= 1e-9)
      throw new IllegalArgumentException("Max Accel must be positive.");
    if (constraints.maxVelocity <= 1e-9)
      throw new IllegalArgumentException("Max Vel must be positive.");

    this.constraints = constraints;
    this.startPos = startPos;
    this.endPos = endPos;
    double dist = endPos - startPos;
    this.direction = Math.signum(dist);

    calculateProfile(Math.abs(dist));
  }

  /**
   * Calculates the 7-segment profile timing.
   *
   * @param dist Absolute distance to travel.
   */
  private void calculateProfile(double dist) {
    double vMax = constraints.maxVelocity;
    double aMax = constraints.maxAcceleration;
    jMax = constraints.maxJerk;

    // Time to reach max acceleration
    double tj = aMax / jMax;
    // Time at max acceleration
    double ta = (vMax / aMax) - tj;

    // Check if we can reach max acceleration
    if (ta < 0) {
      // Triangular acceleration profile
      tj = Math.sqrt(vMax / jMax);
      ta = 0;
      aMax = jMax * tj;
    }

    double v_reached = aMax * (ta + tj);
    double time_accel = 2 * tj + ta;
    double dist_accel = 0.5 * v_reached * time_accel;
    double total_dist_triangular = 2 * dist_accel;

    double tv = 0;

    if (total_dist_triangular > dist) {
      // Distance-limited: won't reach max velocity
      vLim = solveVMaxBinarySearch(dist, constraints.maxAcceleration, jMax);

      aMax = constraints.maxAcceleration;
      tj = aMax / jMax;
      ta = (vLim / aMax) - tj;

      if (ta < 0) {
        tj = Math.sqrt(vLim / jMax);
        ta = 0;
        aMax = jMax * tj;
      }
      tv = 0;
      vLim = aMax * (ta + tj);
    } else {
      // Can reach max velocity with cruise phase
      vLim = v_reached;
      double dist_cruise = dist - total_dist_triangular;
      tv = dist_cruise / vLim;
    }

    this.aLim = aMax;

    // Compute timing knot points
    t1 = tj;
    t2 = tj + ta;
    t3 = 2 * tj + ta;
    t4 = t3 + tv;
    t5 = t4 + tj;
    t6 = t5 + ta;
    t7 = t6 + tj;

    computeKnotPoints();
  }

  /** Computes position and velocity at each knot point. */
  private void computeKnotPoints() {
    pKnot[0] = 0;
    vKnot[0] = 0;

    // Segment 1: Jerk up
    integrateSegment(0, t1, 0, 0, 0, jMax, 1);

    // Segment 2: Constant acceleration
    integrateSegment(t1, t2, pKnot[1], vKnot[1], aLim, 0, 2);

    // Segment 3: Jerk down
    integrateSegment(t2, t3, pKnot[2], vKnot[2], aLim, -jMax, 3);

    // Segment 4: Cruise
    integrateSegment(t3, t4, pKnot[3], vKnot[3], 0, 0, 4);

    // Segment 5: Jerk down (start deceleration)
    integrateSegment(t4, t5, pKnot[4], vKnot[4], 0, -jMax, 5);

    // Segment 6: Constant deceleration
    integrateSegment(t5, t6, pKnot[5], vKnot[5], -aLim, 0, 6);

    // Segment 7: Jerk up (end deceleration)
    integrateSegment(t6, t7, pKnot[6], vKnot[6], -aLim, jMax, 7);
  }

  /** Integrates kinematics for a single segment. */
  private void integrateSegment(
      double tStart, double tEnd, double p0, double v0, double a0, double jerk, int currIdx) {
    double dt = tEnd - tStart;
    if (dt < 0) dt = 0;

    double vFinal = v0 + a0 * dt + 0.5 * jerk * dt * dt;
    double pFinal = p0 + v0 * dt + 0.5 * a0 * dt * dt + (1.0 / 6.0) * jerk * dt * dt * dt;

    pKnot[currIdx] = pFinal;
    vKnot[currIdx] = vFinal;
  }

  /** Binary search to find achievable velocity for short moves. */
  private double solveVMaxBinarySearch(double dist, double aMaxConstraint, double jMax) {
    double low = 0;
    double high = constraints.maxVelocity;

    for (int i = 0; i < 50; i++) {
      double v = (low + high) / 2.0;

      double a = aMaxConstraint;
      double t_j = a / jMax;
      double t_a = (v / a) - t_j;

      if (t_a < 0) {
        t_j = Math.sqrt(v / jMax);
        t_a = 0;
      }

      double dist_for_v = v * (2 * t_j + t_a);

      if (dist_for_v < dist) {
        low = v;
      } else {
        high = v;
      }
    }
    return low;
  }

  /**
   * Samples the profile at a given time.
   *
   * @param t Time from profile start (seconds).
   * @return The motion state at time t.
   */
  public MotionState calculate(double t) {
    if (t <= 0) return new MotionState(0, startPos, 0, 0);
    if (t >= t7) return new MotionState(t7, endPos, 0, 0);

    double p = 0, v = 0, a = 0;
    double dt = 0;

    if (t < t1) {
      // Segment 1: Jerk up
      dt = t;
      a = jMax * dt;
      v = 0.5 * jMax * dt * dt;
      p = (1.0 / 6.0) * jMax * dt * dt * dt;
    } else if (t < t2) {
      // Segment 2: Const accel
      dt = t - t1;
      a = aLim;
      v = vKnot[1] + aLim * dt;
      p = pKnot[1] + vKnot[1] * dt + 0.5 * aLim * dt * dt;
    } else if (t < t3) {
      // Segment 3: Jerk down
      dt = t - t2;
      a = aLim - jMax * dt;
      v = vKnot[2] + aLim * dt - 0.5 * jMax * dt * dt;
      p = pKnot[2] + vKnot[2] * dt + 0.5 * aLim * dt * dt - (1.0 / 6.0) * jMax * dt * dt * dt;
    } else if (t < t4) {
      // Segment 4: Cruise
      dt = t - t3;
      a = 0;
      v = vLim;
      p = pKnot[3] + vLim * dt;
    } else if (t < t5) {
      // Segment 5: Jerk down (start decel)
      dt = t - t4;
      a = -jMax * dt;
      v = vKnot[4] - 0.5 * jMax * dt * dt;
      p = pKnot[4] + vKnot[4] * dt - (1.0 / 6.0) * jMax * dt * dt * dt;
    } else if (t < t6) {
      // Segment 6: Const decel
      dt = t - t5;
      a = -aLim;
      v = vKnot[5] - aLim * dt;
      p = pKnot[5] + vKnot[5] * dt - 0.5 * aLim * dt * dt;
    } else {
      // Segment 7: Jerk up (end decel)
      dt = t - t6;
      a = -aLim + jMax * dt;
      v = vKnot[6] - aLim * dt + 0.5 * jMax * dt * dt;
      p = pKnot[6] + vKnot[6] * dt - 0.5 * aLim * dt * dt + (1.0 / 6.0) * jMax * dt * dt * dt;
    }

    return new MotionState(t, startPos + direction * p, direction * v, direction * a);
  }

  /**
   * Returns the total profile duration.
   *
   * @return Time to reach target position (seconds).
   */
  public double getTotalTime() {
    return t7;
  }
}
