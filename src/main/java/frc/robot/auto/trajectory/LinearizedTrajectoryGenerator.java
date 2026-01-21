package frc.robot.auto.trajectory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;

/**
 * <strong>First Principles & Aerospace Rigor:</strong>
 *
 * <p>This class generates linearized dynamics matrices for a pre-planned trajectory.
 *
 * <p><strong>Optimization Principle (Zero Allocation):</strong> To adhere to real-time constraints
 * and avoid GC pressure (stop-the-world pauses), this class avoids creating objects (like `Matrix`
 * or `ArrayList`) to represent the result. Instead, it packs all data into a single efficient
 * flattened primitive array `double[]`.
 *
 * <p><strong>Memory Layout (Row-Major):</strong> Each trajectory point i occupies 18 doubles in the
 * `matrices` array:
 *
 * <pre>
 * Offset + 0..8:  A matrix (3x3) = [ a11, a12, a13, a21, ... ]
 * Offset + 9..17: B matrix (3x3) = [ b11, b12, b13, b21, ... ]
 * </pre>
 *
 * <p>Total Size = N * 18 doubles.
 */
public class LinearizedTrajectoryGenerator {

  /** Container for the raw pre-computed data. */
  public record LinearizedTrajectory(
      double[] matrices, // Flattened A and B matrices [A0, B0, A1, B1, ...]
      int length // Number of points
      ) {}

  private static final int STATE_DIM = 3;
  private static final int INPUT_DIM = 3;
  private static final int A_SIZE = STATE_DIM * STATE_DIM; // 9
  private static final int B_SIZE = STATE_DIM * INPUT_DIM; // 9
  private static final int BLOCK_SIZE = A_SIZE + B_SIZE; // 18

  /**
   * Generates linearized points for the entire trajectory into a flat array.
   *
   * @param trajectory The reference trajectory (Field-Relative).
   * @return LinearizedTrajectory containing the primitive data buffer.
   */
  public static LinearizedTrajectory generate(List<TrajectoryPoint> trajectory) {
    if (trajectory == null || trajectory.isEmpty()) {
      return new LinearizedTrajectory(new double[0], 0);
    }

    int n = trajectory.size();
    // We allocate ONE large buffer.
    // Note: trajectory.size() points means trajectory.size()-1 intervals,
    // but we pad the last point to match the size for safe indexing.
    double[] matrices = new double[n * BLOCK_SIZE];

    for (int i = 0; i < n; i++) {
      // "Next" point handling: For the last point, we reuse the previous dynamics
      // or assume zero dynamics (stopped). Reusing previous is safer for horizon
      // lookahead.
      TrajectoryPoint current = trajectory.get(i);
      TrajectoryPoint next = (i < n - 1) ? trajectory.get(i + 1) : current;

      double dt = next.timeSeconds() - current.timeSeconds();

      // First Principles: Divergence Protection
      // If dt is 0 (last point) or negligible, impose a small safeguards or
      // just use the previous valid dt implies we hold the last state.
      if (dt <= 1e-6) {
        if (i > 0) {
          // Copy previous block
          int prevOffset = (i - 1) * BLOCK_SIZE;
          int currOffset = i * BLOCK_SIZE;
          System.arraycopy(matrices, prevOffset, matrices, currOffset, BLOCK_SIZE);
          continue;
        } else {
          dt = 0.02; // Fallback for single point trajectory
        }
      }

      // 1. Extract Reference State (Field Relative)
      // x_k states: x, y, theta
      double theta = current.pose().getRotation().getRadians();
      double sinTheta = Math.sin(theta);
      double cosTheta = Math.cos(theta);

      // 2. Extract Reference Control (Robot Relative)
      // CSV is Field-Relative (vx_f, vy_f). We need Robot-Relative (vx_r, vy_r).
      // Transformation: v_r = Rot(-theta) * v_f
      ChassisSpeeds speeds = current.speeds();
      double vx_field = speeds.vxMetersPerSecond;
      double vy_field = speeds.vyMetersPerSecond;

      double vx_robot = vx_field * cosTheta + vy_field * sinTheta;
      double vy_robot = -vx_field * sinTheta + vy_field * cosTheta;

      // 3. Compute Jacobian A elements (3x3)
      // Row-major order: A[0], A[1], A[2] (Row 1), A[3]...
      // A = I + [ 0 0 dxdtheta ] * dt
      // [ 0 0 dydtheta ]
      // [ 0 0 0 ]

      double dxdtheta_dt = (-vx_robot * sinTheta - vy_robot * cosTheta) * dt;
      double dydtheta_dt = (vx_robot * cosTheta - vy_robot * sinTheta) * dt;

      int offset = i * BLOCK_SIZE;

      // A Matrix (3x3) - stored at offset + 0
      // Row 1: [1, 0, dxdtheta_dt]
      matrices[offset + 0] = 1.0;
      matrices[offset + 1] = 0.0;
      matrices[offset + 2] = dxdtheta_dt;

      // Row 2: [0, 1, dydtheta_dt]
      matrices[offset + 3] = 0.0;
      matrices[offset + 4] = 1.0;
      matrices[offset + 5] = dydtheta_dt;

      // Row 3: [0, 0, 1]
      matrices[offset + 6] = 0.0;
      matrices[offset + 7] = 0.0;
      matrices[offset + 8] = 1.0;

      // 4. Compute Jacobian B elements (3x3) - stored at offset + 9
      // B = [ cos -sin 0 ] * dt
      // [ sin cos 0 ]
      // [ 0 0 1 ]

      double c_dt = cosTheta * dt;
      double s_dt = sinTheta * dt;

      // Row 1
      matrices[offset + 9] = c_dt;
      matrices[offset + 10] = -s_dt;
      matrices[offset + 11] = 0.0;

      // Row 2
      matrices[offset + 12] = s_dt;
      matrices[offset + 13] = c_dt;
      matrices[offset + 14] = 0.0;

      // Row 3
      matrices[offset + 15] = 0.0;
      matrices[offset + 16] = 0.0;
      matrices[offset + 17] = dt;
    }

    return new LinearizedTrajectory(matrices, n);
  }
}
