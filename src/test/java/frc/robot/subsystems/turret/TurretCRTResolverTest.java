package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for TurretCRTResolver.
 *
 * <p>Verifies CRT solver correctness across full tooth range, boundary conditions, and noise
 * tolerance.
 */
class TurretCRTResolverTest {

  private static final int MOTOR_TEETH = 41;
  private static final int AUX_TEETH = 37;
  private static final int RING_TEETH = 340;

  /**
   * Full sweep test: N ∈ [0, 339]. Simulates ideal encoder readings for both encoders, verifies CRT
   * correctly reconstructs every tooth position.
   */
  @Test
  void resolveToothPosition_fullSweep_allPositionsCorrect() {
    for (int expectedN = 0; expectedN < RING_TEETH; expectedN++) {
      // 模擬 encoder 讀數: θ = frac(N / teeth)
      double enc1 = (double) (expectedN % MOTOR_TEETH) / MOTOR_TEETH;
      double enc2 = (double) (expectedN % AUX_TEETH) / AUX_TEETH;

      int result = TurretCRTResolver.resolveToothPosition(enc1, enc2, MOTOR_TEETH, AUX_TEETH);

      assertEquals(
          expectedN,
          result,
          "CRT failed for N=" + expectedN + " (enc1=" + enc1 + ", enc2=" + enc2 + ")");
    }
  }

  /** Boundary tests: N = 0, 1, 339. */
  @Test
  void resolveToothPosition_boundaryValues() {
    // N = 0
    assertEquals(0, TurretCRTResolver.resolveToothPosition(0.0, 0.0, MOTOR_TEETH, AUX_TEETH));

    // N = 1
    double enc1_1 = 1.0 / MOTOR_TEETH;
    double enc2_1 = 1.0 / AUX_TEETH;
    assertEquals(1, TurretCRTResolver.resolveToothPosition(enc1_1, enc2_1, MOTOR_TEETH, AUX_TEETH));

    // N = 339
    double enc1_339 = (339.0 % MOTOR_TEETH) / MOTOR_TEETH;
    double enc2_339 = (339.0 % AUX_TEETH) / AUX_TEETH;
    assertEquals(
        339, TurretCRTResolver.resolveToothPosition(enc1_339, enc2_339, MOTOR_TEETH, AUX_TEETH));
  }

  /**
   * Noise tolerance test: Adds ±0.005 rotations random offset to encoder readings, verifies round()
   * quantization still works correctly.
   *
   * <p>CTRE Through Bore Encoder precision ≈ ±0.01 rotations (3.6°), 41T half-tooth threshold =
   * 0.5/41 ≈ 0.0122 rotations, so ±0.005 noise has sufficient margin.
   */
  @Test
  void resolveToothPosition_withNoise_stillCorrect() {
    double noiseAmplitude = 0.005;

    for (int expectedN = 0; expectedN < RING_TEETH; expectedN++) {
      double idealEnc1 = (double) (expectedN % MOTOR_TEETH) / MOTOR_TEETH;
      double idealEnc2 = (double) (expectedN % AUX_TEETH) / AUX_TEETH;

      // 正向偏移
      double plusEnc1 = idealEnc1 + noiseAmplitude;
      double plusEnc2 = idealEnc2 + noiseAmplitude;
      assertEquals(
          expectedN,
          TurretCRTResolver.resolveToothPosition(plusEnc1, plusEnc2, MOTOR_TEETH, AUX_TEETH),
          "CRT +noise failed for N=" + expectedN);

      // 負向偏移
      double minusEnc1 = idealEnc1 - noiseAmplitude;
      double minusEnc2 = idealEnc2 - noiseAmplitude;
      // 負值歸一化: floorMod 在內部處理
      assertEquals(
          expectedN,
          TurretCRTResolver.resolveToothPosition(minusEnc1, minusEnc2, MOTOR_TEETH, AUX_TEETH),
          "CRT -noise failed for N=" + expectedN);
    }
  }

  /** Angle calculation test: Verifies angle precision of resolveAngleRads(). */
  @Test
  void resolveAngleRads_fullSweep_anglesAccurate() {
    for (int n = 0; n < RING_TEETH; n++) {
      double enc1 = (double) (n % MOTOR_TEETH) / MOTOR_TEETH;
      double enc2 = (double) (n % AUX_TEETH) / AUX_TEETH;

      double expectedAngle = (double) n / RING_TEETH * 2.0 * Math.PI;
      double result =
          TurretCRTResolver.resolveAngleRads(enc1, enc2, MOTOR_TEETH, AUX_TEETH, RING_TEETH);

      assertFalse(Double.isNaN(result), "resolveAngleRads returned NaN for N=" + n);
      assertEquals(expectedAngle, result, 0.001, "Angle mismatch for N=" + n);
    }
  }

  /** Coprimality check: Should return -1 when tooth counts are not coprime. */
  @Test
  void resolveToothPosition_nonCoprime_returnsNegativeOne() {
    // gcd(6, 4) = 2, 不互質
    int result = TurretCRTResolver.resolveToothPosition(0.5, 0.5, 6, 4);
    assertEquals(-1, result);
  }

  /** resolveAngleRads should return NaN when not coprime. */
  @Test
  void resolveAngleRads_nonCoprime_returnsNaN() {
    double result = TurretCRTResolver.resolveAngleRads(0.5, 0.5, 6, 4, 12);
    assertTrue(Double.isNaN(result));
  }

  /** Extended Euclidean Algorithm verification: 41a + 37b = 1. */
  @Test
  void extendedGcd_forTurretGears_correct() {
    long[] result = TurretCRTResolver.extendedGcd(41, 37);
    assertEquals(1, result[0], "GCD should be 1");
    assertEquals(1, 41 * result[1] + 37 * result[2], "Bezout identity: 41*a + 37*b should = 1");
  }
}
