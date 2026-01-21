package frc.robot.libraries.lib9427.utils;

import java.util.Arrays;

/**
 * Utility methods for control system tuning.
 *
 * <p>Provides helper functions for constructing the cost and covariance matrices used in LQR and
 * Kalman Filter design.
 */
public class ControlUtils {

  /**
   * Constructs a diagonal cost matrix using Bryson's Rule.
   *
   * <p><strong>Bryson's Rule</strong>: The weight for each state or input should be {@code 1 /
   * (max_tolerable_value)^2}. This normalizes variables so that an error of {@code max_value}
   * contributes exactly 1.0 to the cost.
   *
   * <p><strong>Usage</strong>:
   *
   * <pre>{@code
   * // Q for LQR: tolerate 0.01m position error, 0.1m/s velocity error
   * double[] Q = ControlUtils.makeCostMatrix(0.01, 0.1);
   *
   * // R for LQR: tolerate 12V input
   * double[] R = ControlUtils.makeCostMatrix(12.0);
   * }</pre>
   *
   * <p><strong>Reference</strong>: Bryson, A.E. and Ho, Y.C., "Applied Optimal Control" (1975)
   *
   * @param maxTolerances Maximum tolerable error for each state/input.
   * @return Diagonal matrix as row-major 1D array.
   * @throws IllegalArgumentException If any tolerance is zero.
   */
  public static double[] makeCostMatrix(double... maxTolerances) {
    int n = maxTolerances.length;
    double[] matrix = new double[n * n];
    Arrays.fill(matrix, 0.0);

    for (int i = 0; i < n; i++) {
      double tol = maxTolerances[i];
      if (tol == 0) throw new IllegalArgumentException("Tolerance cannot be zero.");
      matrix[i * n + i] = 1.0 / (tol * tol);
    }
    return matrix;
  }

  /**
   * Extracts the diagonal elements from a square matrix produced by {@link #makeCostMatrix}.
   *
   * <p>This is useful when an API (e.g., OSQP via JNI) expects only diagonal weights rather than a
   * full matrix.
   *
   * @param squareMatrix Row-major square matrix (n × n).
   * @return Array of n diagonal elements.
   */
  public static double[] extractDiagonal(double[] squareMatrix) {
    int n = (int) Math.sqrt(squareMatrix.length);
    if (n * n != squareMatrix.length) {
      throw new IllegalArgumentException("Input must be a square matrix.");
    }
    double[] diag = new double[n];
    for (int i = 0; i < n; i++) {
      diag[i] = squareMatrix[i * n + i];
    }
    return diag;
  }

  /**
   * Constructs a diagonal covariance matrix for Kalman Filters.
   *
   * <p>The covariance matrix elements are the square of the standard deviations: {@code R[i][i] =
   * σ_i²}
   *
   * <p><strong>Usage</strong>:
   *
   * <pre>{@code
   * // Process noise: 1mm position uncertainty, 10mm/s velocity uncertainty
   * double[] Q = ControlUtils.makeCovarianceMatrix(0.001, 0.01);
   *
   * // Measurement noise: 5mm position measurement uncertainty
   * double[] R = ControlUtils.makeCovarianceMatrix(0.005);
   * }</pre>
   *
   * @param standardDeviations Standard deviation of noise for each dimension.
   * @return Diagonal covariance matrix as row-major 1D array.
   */
  public static double[] makeCovarianceMatrix(double... standardDeviations) {
    int n = standardDeviations.length;
    double[] matrix = new double[n * n];
    Arrays.fill(matrix, 0.0);

    for (int i = 0; i < n; i++) {
      double std = standardDeviations[i];
      matrix[i * n + i] = std * std;
    }
    return matrix;
  }
}
