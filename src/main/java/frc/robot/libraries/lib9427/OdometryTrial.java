package frc.robot.libraries.lib9427;

import java.util.ArrayList;
import java.util.List;

/**
 * Complete record of a single odometry characterization trial.
 *
 * <p>Contains time-series data points and computed summary statistics. Statistics are computed when
 * {@link #computeStatistics()} is called after data collection completes.
 *
 * @see OdometryDataPoint
 * @see OdometryCharacterizer
 */
public class OdometryTrial {

  private final int trialId;
  private final List<OdometryDataPoint> dataPoints;
  private OdometryError finalError;

  // === Robot-Level Statistics ===
  private double vMean, vMedian, vStd, vMax, vMin;
  private double vSkew, vKurtosis;
  private double vP90, vP10;
  private double omegaMean, omegaStd, omegaMax;
  private double omegaSkew, omegaKurtosis;
  private double omegaP90;

  private double aMean, aStd, aMax;
  private double alphaMax;

  // === Per-Wheel Statistics ===
  private double[] wheelVMean = new double[4];
  private double[] wheelVStd = new double[4];
  private double[] wheelVMax = new double[4];

  private double[] wheelSteerOmegaMean = new double[4];
  private double[] wheelSteerOmegaMax = new double[4];
  private double[] wheelSteerAlphaMax = new double[4];

  // === Cross-Wheel Metrics ===
  private double wheelVarianceMean, wheelVarianceMax, wheelVarianceStd;

  // === Temporal Statistics ===
  private double totalDistance;
  private double totalRotation;
  private double duration;

  // === Behavioral Statistics ===
  private double stationaryRatio;
  private int directionChanges;
  private int accelSignChanges;
  private int sharpTurns;

  // === Frequency Domain ===
  private double dominantFreqV;
  private double highFreqEnergyRatio;

  // === Electrical Statistics ===
  private double batteryVoltageMean, batteryVoltageMin;
  private double driveCurrentMean, driveCurrentMax;

  // === Control Error Statistics ===
  private double driveErrorMean, driveErrorMax;
  private double steerErrorMean, steerErrorMax;

  // === Slip Statistics ===
  private double slipRatioMean, slipRatioMax;

  // === Timing Statistics ===
  private double dtMean, dtStd, dtMax;

  /**
   * Creates a new trial with the given ID.
   *
   * @param trialId Unique identifier for this trial.
   */
  public OdometryTrial(int trialId) {
    this.trialId = trialId;
    this.dataPoints = new ArrayList<>();
  }

  /**
   * Adds a data point to this trial.
   *
   * @param point The data point to add.
   */
  public void addDataPoint(OdometryDataPoint point) {
    dataPoints.add(point);
  }

  /**
   * Sets the final error for this trial.
   *
   * @param error The pose error when returning to origin.
   */
  public void setFinalError(OdometryError error) {
    this.finalError = error;
  }

  /**
   * Computes all summary statistics from the collected data points.
   *
   * <p>Must be called after data collection is complete and before exporting.
   */
  public void computeStatistics() {
    if (dataPoints.isEmpty()) {
      return;
    }

    int n = dataPoints.size();
    double[] velocities = new double[n];
    double[] omegas = new double[n];
    double[] accels = new double[n];
    double[] alphas = new double[n];
    double[] wheelVariances = new double[n];
    double[] slipRatios = new double[n];
    double[] dts = new double[n];
    double[] driveErrors = new double[n];
    double[] steerErrors = new double[n];
    double[] batteryVoltages = new double[n];
    double[] driveCurrents = new double[n];

    double[][] wheelDriveVels = new double[4][n];
    double[][] wheelSteerOmegas = new double[4][n];
    double[][] wheelSteerAlphas = new double[4][n];

    // Extract arrays
    for (int i = 0; i < n; i++) {
      OdometryDataPoint p = dataPoints.get(i);
      velocities[i] = Math.hypot(p.vx(), p.vy());
      omegas[i] = p.omega();
      accels[i] = Math.hypot(p.ax(), p.ay());
      alphas[i] = Math.abs(p.alpha());
      wheelVariances[i] = p.wheelSpeedVariance();
      slipRatios[i] = p.slipRatio();
      dts[i] = p.dtActual();
      batteryVoltages[i] = p.batteryVoltage();

      // Average drive errors across modules
      double avgDriveErr = 0, avgSteerErr = 0, avgCurrent = 0;
      for (int m = 0; m < 4; m++) {
        wheelDriveVels[m][i] = p.wheelDriveVelocity()[m];
        wheelSteerOmegas[m][i] = Math.abs(p.wheelSteerVelocity()[m]);
        wheelSteerAlphas[m][i] = Math.abs(p.wheelSteerAccel()[m]);
        avgDriveErr += Math.abs(p.wheelDriveError()[m]);
        avgSteerErr += Math.abs(p.wheelSteerError()[m]);
        avgCurrent += p.driveMotorCurrent()[m];
      }
      driveErrors[i] = avgDriveErr / 4;
      steerErrors[i] = avgSteerErr / 4;
      driveCurrents[i] = avgCurrent / 4;
    }

    // Compute robot-level statistics
    vMean = mean(velocities);
    vMedian = median(velocities);
    vStd = std(velocities, vMean);
    vMax = max(velocities);
    vMin = min(velocities);
    vSkew = skewness(velocities, vMean, vStd);
    vKurtosis = kurtosis(velocities, vMean, vStd);
    vP90 = percentile(velocities, 90);
    vP10 = percentile(velocities, 10);

    omegaMean = mean(absArray(omegas));
    omegaStd = std(omegas, 0);
    omegaMax = maxAbs(omegas);
    omegaSkew = skewness(omegas, 0, omegaStd);
    omegaKurtosis = kurtosis(omegas, 0, omegaStd);
    omegaP90 = percentile(absArray(omegas), 90);

    aMean = mean(accels);
    aStd = std(accels, aMean);
    aMax = max(accels);
    alphaMax = max(alphas);

    // Per-wheel statistics
    for (int m = 0; m < 4; m++) {
      wheelVMean[m] = mean(wheelDriveVels[m]);
      wheelVStd[m] = std(wheelDriveVels[m], wheelVMean[m]);
      wheelVMax[m] = max(wheelDriveVels[m]);
      wheelSteerOmegaMean[m] = mean(wheelSteerOmegas[m]);
      wheelSteerOmegaMax[m] = max(wheelSteerOmegas[m]);
      wheelSteerAlphaMax[m] = max(wheelSteerAlphas[m]);
    }

    // Cross-wheel
    wheelVarianceMean = mean(wheelVariances);
    wheelVarianceMax = max(wheelVariances);
    wheelVarianceStd = std(wheelVariances, wheelVarianceMean);

    // Temporal
    OdometryDataPoint first = dataPoints.get(0);
    OdometryDataPoint last = dataPoints.get(n - 1);
    duration = last.timestamp() - first.timestamp();

    // Total distance via trapezoidal integration
    totalDistance = 0;
    totalRotation = 0;
    for (int i = 1; i < n; i++) {
      double dt = dts[i];
      totalDistance += 0.5 * (velocities[i - 1] + velocities[i]) * dt;
      totalRotation += 0.5 * (Math.abs(omegas[i - 1]) + Math.abs(omegas[i])) * dt;
    }

    // Behavioral
    stationaryRatio = countBelow(velocities, 0.05) / (double) n;
    directionChanges = countDirectionChanges(dataPoints);
    accelSignChanges = countSignChanges(diff(velocities));
    sharpTurns = countAbove(alphas, 3.0);

    // Electrical
    batteryVoltageMean = mean(batteryVoltages);
    batteryVoltageMin = min(batteryVoltages);
    driveCurrentMean = mean(driveCurrents);
    driveCurrentMax = max(driveCurrents);

    // Control errors
    driveErrorMean = mean(driveErrors);
    driveErrorMax = max(driveErrors);
    steerErrorMean = mean(steerErrors);
    steerErrorMax = max(steerErrors);

    // Slip
    slipRatioMean = mean(slipRatios);
    slipRatioMax = max(slipRatios);

    // Timing
    dtMean = mean(dts);
    dtStd = std(dts, dtMean);
    dtMax = max(dts);

    // Frequency domain (simplified - just dominant frequency)
    if (n > 10) {
      dominantFreqV = estimateDominantFrequency(velocities, dtMean);
      highFreqEnergyRatio = 0; // Simplified - would need FFT
    }
  }

  /**
   * Returns CSV row for trial summary.
   *
   * @return Comma-separated summary values.
   */
  public String toSummaryCsvRow() {
    StringBuilder sb = new StringBuilder();
    sb.append(trialId).append(",");
    sb.append(duration).append(",");
    sb.append(totalDistance).append(",");
    sb.append(totalRotation).append(",");

    sb.append(vMean).append(",");
    sb.append(vMedian).append(",");
    sb.append(vStd).append(",");
    sb.append(vMax).append(",");
    sb.append(vMin).append(",");
    sb.append(vSkew).append(",");
    sb.append(vKurtosis).append(",");
    sb.append(vP90).append(",");
    sb.append(vP10).append(",");

    sb.append(omegaMean).append(",");
    sb.append(omegaStd).append(",");
    sb.append(omegaMax).append(",");
    sb.append(omegaSkew).append(",");
    sb.append(omegaKurtosis).append(",");
    sb.append(omegaP90).append(",");

    sb.append(aMean).append(",");
    sb.append(aStd).append(",");
    sb.append(aMax).append(",");
    sb.append(alphaMax).append(",");

    for (int i = 0; i < 4; i++) {
      sb.append(wheelVMean[i]).append(",");
      sb.append(wheelVStd[i]).append(",");
      sb.append(wheelVMax[i]).append(",");
    }
    for (int i = 0; i < 4; i++) {
      sb.append(wheelSteerOmegaMean[i]).append(",");
      sb.append(wheelSteerOmegaMax[i]).append(",");
    }
    for (int i = 0; i < 4; i++) {
      sb.append(wheelSteerAlphaMax[i]).append(",");
    }

    sb.append(wheelVarianceMean).append(",");
    sb.append(wheelVarianceMax).append(",");
    sb.append(wheelVarianceStd).append(",");

    sb.append(batteryVoltageMean).append(",");
    sb.append(batteryVoltageMin).append(",");
    sb.append(driveCurrentMean).append(",");
    sb.append(driveCurrentMax).append(",");

    sb.append(driveErrorMean).append(",");
    sb.append(driveErrorMax).append(",");
    sb.append(steerErrorMean).append(",");
    sb.append(steerErrorMax).append(",");

    sb.append(stationaryRatio).append(",");
    sb.append(directionChanges).append(",");
    sb.append(accelSignChanges).append(",");
    sb.append(sharpTurns).append(",");

    sb.append(dominantFreqV).append(",");
    sb.append(highFreqEnergyRatio).append(",");

    sb.append(slipRatioMean).append(",");
    sb.append(slipRatioMax).append(",");

    sb.append(dtMean).append(",");
    sb.append(dtStd).append(",");
    sb.append(dtMax).append(",");

    if (finalError != null) {
      sb.append(finalError.toCsvRow());
    } else {
      sb.append("0,0,0,0");
    }

    return sb.toString();
  }

  /** Returns CSV header for trial summary. */
  public static String summaryCsvHeader() {
    return "trial_id,duration_s,total_distance_m,total_rotation_rad,"
        + "v_mean,v_median,v_std,v_max,v_min,v_skew,v_kurtosis,v_p90,v_p10,"
        + "omega_mean,omega_std,omega_max,omega_skew,omega_kurtosis,omega_p90,"
        + "a_mean,a_std,a_max,alpha_max,"
        + "wheel_FL_v_mean,wheel_FL_v_std,wheel_FL_v_max,"
        + "wheel_FR_v_mean,wheel_FR_v_std,wheel_FR_v_max,"
        + "wheel_BL_v_mean,wheel_BL_v_std,wheel_BL_v_max,"
        + "wheel_BR_v_mean,wheel_BR_v_std,wheel_BR_v_max,"
        + "wheel_FL_steer_omega_mean,wheel_FL_steer_omega_max,"
        + "wheel_FR_steer_omega_mean,wheel_FR_steer_omega_max,"
        + "wheel_BL_steer_omega_mean,wheel_BL_steer_omega_max,"
        + "wheel_BR_steer_omega_mean,wheel_BR_steer_omega_max,"
        + "wheel_FL_steer_alpha_max,wheel_FR_steer_alpha_max,"
        + "wheel_BL_steer_alpha_max,wheel_BR_steer_alpha_max,"
        + "wheel_variance_mean,wheel_variance_max,wheel_variance_std,"
        + "battery_voltage_mean,battery_voltage_min,drive_current_mean,drive_current_max,"
        + "drive_error_mean,drive_error_max,steer_error_mean,steer_error_max,"
        + "stationary_ratio,direction_changes,accel_sign_changes,sharp_turns,"
        + "dominant_freq_v,high_freq_energy_ratio,"
        + "slip_ratio_mean,slip_ratio_max,"
        + "dt_mean,dt_std,dt_max,"
        + OdometryError.csvHeader();
  }

  // Getters
  public int getTrialId() {
    return trialId;
  }

  public List<OdometryDataPoint> getDataPoints() {
    return dataPoints;
  }

  public OdometryError getFinalError() {
    return finalError;
  }

  public double getDuration() {
    return duration;
  }

  public double getTotalDistance() {
    return totalDistance;
  }

  // === Statistical Helper Methods ===

  private static double mean(double[] arr) {
    if (arr.length == 0) return 0;
    double sum = 0;
    for (double v : arr) sum += v;
    return sum / arr.length;
  }

  private static double std(double[] arr, double mean) {
    if (arr.length == 0) return 0;
    double sumSq = 0;
    for (double v : arr) sumSq += (v - mean) * (v - mean);
    return Math.sqrt(sumSq / arr.length);
  }

  private static double max(double[] arr) {
    if (arr.length == 0) return 0;
    double m = arr[0];
    for (double v : arr) m = Math.max(m, v);
    return m;
  }

  private static double min(double[] arr) {
    if (arr.length == 0) return 0;
    double m = arr[0];
    for (double v : arr) m = Math.min(m, v);
    return m;
  }

  private static double maxAbs(double[] arr) {
    if (arr.length == 0) return 0;
    double m = 0;
    for (double v : arr) m = Math.max(m, Math.abs(v));
    return m;
  }

  private static double median(double[] arr) {
    if (arr.length == 0) return 0;
    double[] sorted = arr.clone();
    java.util.Arrays.sort(sorted);
    int mid = sorted.length / 2;
    return sorted.length % 2 == 0 ? (sorted[mid - 1] + sorted[mid]) / 2.0 : sorted[mid];
  }

  private static double percentile(double[] arr, double p) {
    if (arr.length == 0) return 0;
    double[] sorted = arr.clone();
    java.util.Arrays.sort(sorted);
    int idx = (int) Math.ceil(p / 100.0 * sorted.length) - 1;
    return sorted[Math.max(0, Math.min(idx, sorted.length - 1))];
  }

  private static double skewness(double[] arr, double mean, double std) {
    if (arr.length == 0 || std == 0) return 0;
    double sum = 0;
    for (double v : arr) sum += Math.pow((v - mean) / std, 3);
    return sum / arr.length;
  }

  private static double kurtosis(double[] arr, double mean, double std) {
    if (arr.length == 0 || std == 0) return 0;
    double sum = 0;
    for (double v : arr) sum += Math.pow((v - mean) / std, 4);
    return sum / arr.length - 3; // Excess kurtosis
  }

  private static double[] absArray(double[] arr) {
    double[] result = new double[arr.length];
    for (int i = 0; i < arr.length; i++) result[i] = Math.abs(arr[i]);
    return result;
  }

  private static int countBelow(double[] arr, double threshold) {
    int count = 0;
    for (double v : arr) if (v < threshold) count++;
    return count;
  }

  private static int countAbove(double[] arr, double threshold) {
    int count = 0;
    for (double v : arr) if (v > threshold) count++;
    return count;
  }

  private static double[] diff(double[] arr) {
    if (arr.length < 2) return new double[0];
    double[] result = new double[arr.length - 1];
    for (int i = 0; i < result.length; i++) result[i] = arr[i + 1] - arr[i];
    return result;
  }

  private static int countSignChanges(double[] arr) {
    int count = 0;
    for (int i = 1; i < arr.length; i++) {
      if (arr[i] * arr[i - 1] < 0) count++;
    }
    return count;
  }

  private static int countDirectionChanges(List<OdometryDataPoint> points) {
    if (points.size() < 2) return 0;
    int count = 0;
    double prevAngle = Math.atan2(points.get(0).vy(), points.get(0).vx());
    for (int i = 1; i < points.size(); i++) {
      double angle = Math.atan2(points.get(i).vy(), points.get(i).vx());
      double diff = Math.abs(angle - prevAngle);
      diff = Math.min(diff, 2 * Math.PI - diff);
      if (diff > Math.PI / 2) count++;
      prevAngle = angle;
    }
    return count;
  }

  private static double estimateDominantFrequency(double[] signal, double dt) {
    // Simplified: count zero crossings
    int crossings = countSignChanges(diff(signal));
    double totalTime = dt * signal.length;
    return totalTime > 0 ? crossings / (2.0 * totalTime) : 0;
  }
}
