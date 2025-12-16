package frc.robot.libraries.lib9427.utils;

import java.util.Arrays;

/** Utility class for control theory helpers, specifically for tuning LQR and Kalman Filters. */
public class ControlUtils {

  /**
   * Applies Bryson's Rule to generate a diagonal Cost Matrix (Q or R) for LQR.
   *
   * <p>Bryson's Rule states that the weight for a state or input should be 1 / (max_value)^2. This
   * normalizes the variables so that a deviation of 'max_value' contributes 1.0 to the cost.
   *
   * @param maxTolerances The maximum tolerable error (for states) or input (for inputs).
   * @return A flattened diagonal matrix [1/tol1^2, 0, ..., 0, 1/tol2^2, ...].
   */
  public static double[] makeCostMatrix(double... maxTolerances) {
    int n = maxTolerances.length;
    double[] matrix = new double[n * n];
    Arrays.fill(matrix, 0.0);

    for (int i = 0; i < n; i++) {
      double tol = maxTolerances[i];
      if (tol == 0) throw new IllegalArgumentException("Tolerance cannot be zero.");
      // Cost = 1 / x^2
      matrix[i * n + i] = 1.0 / (tol * tol);
    }
    return matrix;
  }

  /**
   * Generates a diagonal Covariance Matrix (Q or R) for Kalman Filters.
   *
   * <p>For a Kalman Filter, the covariance is the square of the standard deviation. Q = E[w w^T], R
   * = E[v v^T].
   *
   * @param standardDeviations The standard deviation of the noise for each state or measurement.
   * @return A flattened diagonal matrix [std1^2, 0, ..., std2^2, ...].
   */
  public static double[] makeCovarianceMatrix(double... standardDeviations) {
    int n = standardDeviations.length;
    double[] matrix = new double[n * n];
    Arrays.fill(matrix, 0.0);

    for (int i = 0; i < n; i++) {
      double std = standardDeviations[i];
      // Covariance = std^2
      matrix[i * n + i] = std * std;
    }
    return matrix;
  }
}
