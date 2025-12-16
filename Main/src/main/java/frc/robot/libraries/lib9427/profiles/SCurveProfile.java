package frc.robot.libraries.lib9427.profiles;

/**
 * Generates an S-Curve (Jerk-Limited) Motion Profile.
 *
 * <p>This profile limits the rate of change of acceleration (Jerk) to prevent mechanism shock and
 * voltage spikes, enabling high-performance control tracking.
 */
public class SCurveProfile {
  private final MotionConstraints constraints;
  private final double startPos;
  private final double endPos;
  private final double direction;

  // Timing Knot Points (Relative to start time 0)
  private double t1, t2, t3, t4, t5, t6, t7;

  // State at Knot Points (Relative to start state 0)
  // pKnot[i] is position at end of segment i (at time ti)
  private final double[] pKnot = new double[8];
  private final double[] vKnot = new double[8];

  // Calculated Limits used in execution
  private double aLim;
  private double vLim;
  private double jMax;

  /**
   * Constructs an S-Curve Profile.
   *
   * @param constraints Maximum velocity, acceleration, and jerk.
   * @param startPos Starting position.
   * @param endPos Target position.
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

  private void calculateProfile(double dist) {
    double vMax = constraints.maxVelocity;
    double aMax = constraints.maxAcceleration;
    jMax = constraints.maxJerk;

    // Kinematic Limits
    double tj = aMax / jMax;
    double ta = (vMax / aMax) - tj;

    // Check if we reach Max Acceleration
    if (ta < 0) {
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
      // Distance Limited
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
      vLim = v_reached;
      double dist_cruise = dist - total_dist_triangular;
      tv = dist_cruise / vLim;
    }

    this.aLim = aMax;

    // Knot Points
    t1 = tj;
    t2 = tj + ta;
    t3 = 2 * tj + ta;
    t4 = t3 + tv;
    t5 = t4 + tj;
    t6 = t5 + ta;
    t7 = t6 + tj;

    computeKnotPoints();
  }

  private void computeKnotPoints() {
    pKnot[0] = 0;
    vKnot[0] = 0;

    // Use local variable for timings to ensure clarity
    // Seg 1: Jerk Up
    integrateSegment(0, t1, 0, 0, 0, jMax, 1);

    // Seg 2: Const Accel
    integrateSegment(t1, t2, pKnot[1], vKnot[1], aLim, 0, 2);

    // Seg 3: Jerk Down
    integrateSegment(t2, t3, pKnot[2], vKnot[2], aLim, -jMax, 3);

    // Seg 4: Const Vel
    integrateSegment(t3, t4, pKnot[3], vKnot[3], 0, 0, 4);

    // Seg 5: Jerk Down (Decel)
    integrateSegment(t4, t5, pKnot[4], vKnot[4], 0, -jMax, 5);

    // Seg 6: Const Decel
    integrateSegment(t5, t6, pKnot[5], vKnot[5], -aLim, 0, 6);

    // Seg 7: Jerk Up (Decel)
    integrateSegment(t6, t7, pKnot[6], vKnot[6], -aLim, jMax, 7);
  }

  private void integrateSegment(
      double tStart, double tEnd, double p0, double v0, double a0, double jerk, int currIdx) {
    double dt = tEnd - tStart;
    if (dt < 0) dt = 0; // Floating point safety

    double vFinal = v0 + a0 * dt + 0.5 * jerk * dt * dt;
    double pFinal = p0 + v0 * dt + 0.5 * a0 * dt * dt + (1.0 / 6.0) * jerk * dt * dt * dt;

    pKnot[currIdx] = pFinal;
    vKnot[currIdx] = vFinal;
  }

  private double solveVMaxBinarySearch(double dist, double aMaxConstraint, double jMax) {
    double low = 0;
    double high = constraints.maxVelocity;

    // 50 iterations => very high precision
    for (int i = 0; i < 50; i++) {
      double v = (low + high) / 2.0;

      double a = aMaxConstraint;
      double t_j = a / jMax;
      double t_a = (v / a) - t_j;

      if (t_a < 0) {
        t_j = Math.sqrt(v / jMax);
        t_a = 0;
      }

      // v_peak = j * t_j for triangle, or a * (t_a + t_j) for trap
      // Here assuming a matches logic above
      // Dist = v * (2 tj + ta)
      double dist_for_v = v * (2 * t_j + t_a);

      if (dist_for_v < dist) {
        low = v;
      } else {
        high = v;
      }
    }
    return low;
  }

  public MotionState calculate(double t) {
    if (t <= 0) return new MotionState(0, startPos, 0, 0);
    if (t >= t7) return new MotionState(t7, endPos, 0, 0);

    double p = 0, v = 0, a = 0;
    double dt = 0;

    if (t < t1) {
      dt = t;
      a = jMax * dt;
      v = 0.5 * jMax * dt * dt;
      p = (1.0 / 6.0) * jMax * dt * dt * dt;
    } else if (t < t2) {
      dt = t - t1;
      a = aLim;
      v = vKnot[1] + aLim * dt;
      p = pKnot[1] + vKnot[1] * dt + 0.5 * aLim * dt * dt;
    } else if (t < t3) {
      dt = t - t2;
      a = aLim - jMax * dt;
      v = vKnot[2] + aLim * dt - 0.5 * jMax * dt * dt;
      p = pKnot[2] + vKnot[2] * dt + 0.5 * aLim * dt * dt - (1.0 / 6.0) * jMax * dt * dt * dt;
    } else if (t < t4) {
      dt = t - t3;
      a = 0;
      v = vLim;
      p = pKnot[3] + vLim * dt;
    } else if (t < t5) {
      dt = t - t4;
      a = -jMax * dt;
      v = vKnot[4] - 0.5 * jMax * dt * dt;
      p = pKnot[4] + vKnot[4] * dt - (1.0 / 6.0) * jMax * dt * dt * dt;
    } else if (t < t6) {
      dt = t - t5;
      a = -aLim;
      v = vKnot[5] - aLim * dt;
      p = pKnot[5] + vKnot[5] * dt - 0.5 * aLim * dt * dt;
    } else {
      dt = t - t6;
      a = -aLim + jMax * dt;
      v = vKnot[6] - aLim * dt + 0.5 * jMax * dt * dt;
      p = pKnot[6] + vKnot[6] * dt - 0.5 * aLim * dt * dt + (1.0 / 6.0) * jMax * dt * dt * dt;
    }

    return new MotionState(t, startPos + direction * p, direction * v, direction * a);
  }

  public double getTotalTime() {
    return t7;
  }
}
