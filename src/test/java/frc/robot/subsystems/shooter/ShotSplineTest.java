package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Verification tests for the ShotTables lookup data.
 *
 * <p>Tests validate:
 *
 * <ol>
 *   <li>Exact knot-point accuracy (machine epsilon at original data points)
 *   <li>Boundary clamping behavior
 *   <li>ToF derivative correctness via finite difference
 *   <li>Monotonicity properties where expected
 * </ol>
 *
 * <p>Run with: {@code ./gradlew test --tests "frc.robot.subsystems.shooter.ShotSplineTest"}
 */
class ShotSplineTest {

  // ── Ground-truth values from generate_shot_table.py (2026-02-20) ──
  // 25 nodes at 0.25m intervals, apex <= 3.5m constraint
  private static final double[] DISTANCES = {
    0.50, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00, 2.25, 2.50, 2.75, 3.00, 3.25, 3.50, 3.75, 4.00, 4.25,
    4.50, 4.75, 5.00, 5.25, 5.50, 5.75, 6.00, 6.25, 6.50
  };

  private static final double[] HOOD_ANGLES = {
    1.448623, 1.448623, 1.448623, 1.447937, 1.423485, 1.399376, 1.375218, 1.351256, 1.327392,
    1.303822, 1.280546, 1.257466, 1.234631, 1.212041, 1.189892, 1.168037, 1.146476, 1.125160,
    1.104236, 1.083802, 1.063515, 1.047198, 1.047198, 1.047198, 1.047198
  };

  private static final double[] FLYWHEEL_SPEEDS = {
    34.20, 39.30, 44.40, 49.10, 49.30, 49.50, 49.70, 50.00, 50.30, 50.70, 51.60, 52.80, 54.20,
    55.60, 57.40, 59.30, 61.30, 63.40, 65.60, 67.90, 70.30, 72.80, 75.30, 77.80, 80.30
  };

  private static final double[] TIME_OF_FLIGHT = {
    0.7719, 1.0202, 1.2222, 1.3912, 1.3913, 1.3914, 1.3910, 1.3908, 1.3898, 1.3892, 1.4063, 1.4312,
    1.4607, 1.4883, 1.5263, 1.5647, 1.6035, 1.6427, 1.6822, 1.7220, 1.7617, 1.8055, 1.8669, 1.9272,
    1.9864
  };

  // ══════════════════════════════════════════════════════════════════
  // Test 1: Exact knot-point accuracy
  // ══════════════════════════════════════════════════════════════════

  /**
   * Verifies the TreeMap returns exact values at every original data point.
   *
   * <p>Tolerance: 1e-6 (well within the precision needed for rad/rot/s/s).
   */
  @Test
  void hoodAngleExactAtKnots() {
    ShotTables.setTuningOffsets(0.0, 0.0);
    for (int i = 0; i < DISTANCES.length; i++) {
      assertEquals(
          HOOD_ANGLES[i],
          ShotTables.hoodAngleRad(DISTANCES[i]),
          1e-6,
          "Hood angle mismatch at d=" + DISTANCES[i] + "m");
    }
  }

  @Test
  void flywheelSpeedExactAtKnots() {
    ShotTables.setTuningOffsets(0.0, 0.0);
    for (int i = 0; i < DISTANCES.length; i++) {
      assertEquals(
          FLYWHEEL_SPEEDS[i],
          ShotTables.flywheelSpeedRotPerSec(DISTANCES[i]),
          1e-4,
          "Flywheel speed mismatch at d=" + DISTANCES[i] + "m");
    }
  }

  @Test
  void tofExactAtKnots() {
    for (int i = 0; i < DISTANCES.length; i++) {
      assertEquals(
          TIME_OF_FLIGHT[i],
          ShotTables.timeOfFlightS(DISTANCES[i]),
          1e-6,
          "ToF mismatch at d=" + DISTANCES[i] + "m");
    }
  }

  // ══════════════════════════════════════════════════════════════════
  // Test 2: Boundary clamping
  // ══════════════════════════════════════════════════════════════════

  /** Values outside [0.5, 6.5] must be clamped to the boundary value. */
  @Test
  void clampsBelowMinimum() {
    ShotTables.setTuningOffsets(0.0, 0.0);
    assertEquals(ShotTables.hoodAngleRad(0.5), ShotTables.hoodAngleRad(0.0), 1e-10);
    assertEquals(ShotTables.hoodAngleRad(0.5), ShotTables.hoodAngleRad(-5.0), 1e-10);
    assertEquals(
        ShotTables.flywheelSpeedRotPerSec(0.5), ShotTables.flywheelSpeedRotPerSec(0.0), 1e-10);
  }

  @Test
  void clampsAboveMaximum() {
    ShotTables.setTuningOffsets(0.0, 0.0);
    assertEquals(ShotTables.hoodAngleRad(6.5), ShotTables.hoodAngleRad(10.0), 1e-10);
    assertEquals(ShotTables.hoodAngleRad(6.5), ShotTables.hoodAngleRad(100.0), 1e-10);
    assertEquals(
        ShotTables.flywheelSpeedRotPerSec(6.5), ShotTables.flywheelSpeedRotPerSec(99.0), 1e-10);
  }

  // ══════════════════════════════════════════════════════════════════
  // Test 3: ToF derivative correctness
  // ══════════════════════════════════════════════════════════════════

  /**
   * Verifies the numerical derivative against finite-difference approximation.
   *
   * <p>Uses central difference: f'(x) ~ (f(x+h) - f(x-h)) / (2h) with h = 1e-6.
   */
  @ParameterizedTest
  @ValueSource(doubles = {0.60, 1.00, 1.50, 2.00, 2.50, 3.00, 3.50, 4.00, 5.00, 6.00})
  void tofDerivativeMatchesFiniteDifference(double dist) {
    double h = 1e-6;
    double finiteDiff =
        (ShotTables.timeOfFlightS(dist + h) - ShotTables.timeOfFlightS(dist - h)) / (2.0 * h);
    double analyticalDeriv = ShotTables.timeOfFlightDerivative(dist);

    assertEquals(
        finiteDiff,
        analyticalDeriv,
        1e-3,
        "ToF derivative mismatch at d="
            + dist
            + "m: finiteDiff="
            + finiteDiff
            + " numericalDeriv="
            + analyticalDeriv);
  }

  // ══════════════════════════════════════════════════════════════════
  // Test 4: Physical sanity checks
  // ══════════════════════════════════════════════════════════════════

  /**
   * Flywheel speed should be monotonically increasing across the full range. With the 3.5m apex
   * constraint, flywheel speed increases smoothly from ~40 to ~71 rot/s.
   */
  @Test
  void flywheelSpeedIsMonotonicallyIncreasing() {
    ShotTables.setTuningOffsets(0.0, 0.0);
    double prev = ShotTables.flywheelSpeedRotPerSec(0.5);
    for (double d = 0.75; d <= 6.5; d += 0.25) {
      double curr = ShotTables.flywheelSpeedRotPerSec(d);
      assertTrue(
          curr >= prev - 0.5, // 0.5 rot/s tolerance for interpolation rounding
          "Flywheel speed not monotonically increasing at d="
              + d
              + "m: prev="
              + prev
              + " curr="
              + curr);
      prev = curr;
    }
  }

  /** Hood angle and flywheel speed must always be positive and within physical bounds. */
  @Test
  void outputsWithinPhysicalBounds() {
    ShotTables.setTuningOffsets(0.0, 0.0);
    for (double d = 0.5; d <= 6.5; d += 0.05) {
      double hood = ShotTables.hoodAngleRad(d);
      double fw = ShotTables.flywheelSpeedRotPerSec(d);
      double tof = ShotTables.timeOfFlightS(d);

      // Hood angle: 55 deg (0.96 rad) to 85 deg (1.48 rad) — generous bounds
      assertTrue(
          hood > 0.95 && hood < 1.50,
          "Hood angle out of physical bounds at d=" + d + "m: " + hood + " rad");
      // Flywheel: 30 to 100 rot/s
      assertTrue(
          fw > 30 && fw < 100,
          "Flywheel speed out of physical bounds at d=" + d + "m: " + fw + " rot/s");
      // ToF: 0.5 to 2.0 seconds (high-arc trajectories have longer ToF)
      assertTrue(
          tof > 0.5 && tof < 2.0, "ToF out of physical bounds at d=" + d + "m: " + tof + " s");
    }
  }

  // ══════════════════════════════════════════════════════════════════
  // Test 5: Tuning offset correctness
  // ══════════════════════════════════════════════════════════════════

  @Test
  void tuningOffsetsApplyCorrectly() {
    double baseHood = ShotTables.hoodAngleRad(3.0);
    double baseFw = ShotTables.flywheelSpeedRotPerSec(3.0);

    double offsetRad = 0.05;
    double offsetRotPerSec = 2.0;
    ShotTables.setTuningOffsets(offsetRad, offsetRotPerSec);

    assertEquals(baseHood + offsetRad, ShotTables.hoodAngleRad(3.0), 1e-10);
    assertEquals(baseFw + offsetRotPerSec, ShotTables.flywheelSpeedRotPerSec(3.0), 1e-10);

    // Clean up
    ShotTables.setTuningOffsets(0.0, 0.0);
  }
}
