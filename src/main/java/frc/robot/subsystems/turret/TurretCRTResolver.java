package frc.robot.subsystems.turret;

/**
 * Chinese Remainder Theorem (CRT) Resolver: Calculates unique turret tooth position from two
 * absolute encoders.
 *
 * <p><strong>Mathematical Principle</strong>: Two gears with coprime tooth counts (teeth1, teeth2)
 * mesh with the same ring gear. The absolute encoders on each gear shaft measure the fractional
 * position of that gear within one rotation, which is equivalent to "how many teeth the ring gear
 * has advanced mod gear teeth".
 *
 * <p>Because gcd(teeth1, teeth2) == 1, CRT guarantees a unique solution within lcm(teeth1, teeth2)
 * tooth positions. As long as the ring gear tooth count < lcm, we can uniquely determine the
 * turret's position within one full rotation.
 *
 * <p><strong>Precision Bounds</strong>: Encoder readings are quantized to integer tooth positions
 * via round(). As long as backlash + encoder error < 0.5/teeth rotations (i.e., ±half a tooth), the
 * quantization will be correct. CTRE Through Bore precision of ±0.01 rot has sufficient margin for
 * 41T/37T gears (threshold ±0.0122 / ±0.0135 rot).
 *
 * <p><strong>Reference</strong>: Extended Euclidean Algorithm.
 *
 * @see <a href="https://en.wikipedia.org/wiki/Chinese_remainder_theorem">CRT - Wikipedia</a>
 */
public final class TurretCRTResolver {

  /**
   * Consistency Threshold: Max allowed difference between independently calculated angles from each
   * encoder (rad).
   */
  private static final double kConsistencyThresholdRads = Math.toRadians(5.0);

  /** Prevent instantiation. */
  private TurretCRTResolver() {}

  /**
   * Resolves turret tooth position via CRT.
   *
   * <p>Calculates how many teeth the ring gear has moved based on two absolute encoder readings
   * (rotations, [0, 1)).
   *
   * <p><strong>Algorithm Steps</strong>:
   *
   * <ol>
   *   <li>Convert encoder fractional position * teeth -> round -> mod teeth to get discrete tooth
   *       residues n1, n2
   *   <li>Use Extended Euclidean Algorithm to find Bezout coefficients for teeth1·a + teeth2·b = 1
   *   <li>Apply CRT formula: N = (n1·teeth2·b + n2·teeth1·a) mod (teeth1·teeth2)
   * </ol>
   *
   * <p><strong>Preconditions</strong>:
   *
   * <ul>
   *   <li>gcd(teeth1, teeth2) == 1 (coprime)
   *   <li>Encoder readings must be MagnetOffset compensated, range [0, 1)
   *   <li>encoder1 corresponds to teeth1 gear, encoder2 corresponds to teeth2 gear
   * </ul>
   *
   * @param encoder1Rotations Absolute encoder reading for gear 1 (rotations, [0, 1)).
   * @param encoder2Rotations Absolute encoder reading for gear 2 (rotations, [0, 1)).
   * @param teeth1 Tooth count for gear 1 (must be > 0).
   * @param teeth2 Tooth count for gear 2 (must be > 0, coprime to teeth1).
   * @return The calculated ring gear tooth position N, range [0, teeth1 * teeth2). Returns -1 if
   *     gcd(teeth1, teeth2) != 1.
   */
  public static int resolveToothPosition(
      double encoder1Rotations, double encoder2Rotations, int teeth1, int teeth2) {

    // 1. Calculate un-synchronized raw tooth positions (fractional)
    double x1 = encoder1Rotations * teeth1;
    double x2 = encoder2Rotations * teeth2;

    // 2. Phase-Synchronization to prevent independent rounding mismatches
    // (If N is halfway between integers, noise can cause n1 and n2 to round
    // differently, breaking CRT)
    double f1 = x1 - Math.floor(x1);
    double f2 = x2 - Math.floor(x2);

    // Find shortest circular difference between fractions
    double df = edu.wpi.first.math.MathUtil.inputModulus(f1 - f2, -0.5, 0.5);

    // Synchronize them to have the exact same fractional part
    double x1_sync = x1 - df / 2.0;
    double x2_sync = x2 + df / 2.0;

    // 3. Safely round to integers now that they are synchronized
    int n1 = Math.floorMod((int) Math.round(x1_sync), teeth1);
    int n2 = Math.floorMod((int) Math.round(x2_sync), teeth2);

    // Extended Euclidean: teeth1 * a + teeth2 * b = gcd
    long[] result = extendedGcd(teeth1, teeth2);
    long gcd = result[0];
    long a = result[1]; // Bezout coefficient for teeth1
    long b = result[2]; // Bezout coefficient for teeth2

    // CRT requires coprime
    if (gcd != 1) {
      return -1;
    }

    long mod = (long) teeth1 * teeth2;
    // CRT Formula: N = (n1 * teeth2 * b + n2 * teeth1 * a) mod (teeth1 * teeth2)
    long toothPosition = Math.floorMod((long) n1 * teeth2 * b + (long) n2 * teeth1 * a, mod);

    // Map the unidirectional [0, t1*t2) mathematical space to the symmetric
    // physical physical space [-t1*t2/2, t1*t2/2)
    long maxCRT = (long) teeth1 * teeth2;
    if (toothPosition > maxCRT / 2L) {
      toothPosition -= maxCRT;
    }

    return (int) toothPosition;
  }

  /**
   * Resolves turret angle (radians) via CRT with continuous precision.
   *
   * <p>Based on {@link #resolveToothPosition}, finds the discrete tooth position, then reconstructs
   * the exact continuous angle using the raw high-precision encoder reading.
   */
  public static double resolveAngleRads(
      double encoder1Rotations, double encoder2Rotations, int teeth1, int teeth2, int ringTeeth) {

    int toothPositionN = resolveToothPosition(encoder1Rotations, encoder2Rotations, teeth1, teeth2);
    if (toothPositionN == -1 && extendedGcd(teeth1, teeth2)[0] != 1) {
      return Double.NaN;
    }

    if (Math.abs(toothPositionN) >= ringTeeth) {
      System.err.println(
          "CRT WARNING: |toothPosition|="
              + Math.abs(toothPositionN)
              + " exceeds full rotation ringTeeth="
              + ringTeeth
              + ". Encoder phase data may be corrupted.");
      return Double.NaN;
    }

    // --- Continuous Precision Reconstruction ---
    // The continuous tooth reading from encoder 1 is: candidate = x1 + k * teeth1
    // We want to find k such that candidate is as close to toothPositionN as
    // possible.
    double x1 = encoder1Rotations * teeth1;
    double x2 = encoder2Rotations * teeth2;

    double continuousTooth1 = findClosestContinuousTooth(x1, teeth1, toothPositionN);
    double continuousTooth2 = findClosestContinuousTooth(x2, teeth2, toothPositionN);

    double angle1 = (continuousTooth1 / ringTeeth) * 2.0 * Math.PI;
    double angle2 = (continuousTooth2 / ringTeeth) * 2.0 * Math.PI;

    // --- Consistency Check ---
    double error = Math.abs(angle1 - angle2);
    if (error > kConsistencyThresholdRads) {
      System.err.println(
          "CRT CONSISTENCY FAIL: encoder1 angle="
              + Math.toDegrees(angle1)
              + "\u00b0 vs encoder2 angle="
              + Math.toDegrees(angle2)
              + "\u00b0 (diff="
              + Math.toDegrees(error)
              + "\u00b0)");
      return Double.NaN;
    }

    return angle1; // Return high-precision continuous angle
  }

  /** Finds the unique continuous tooth position matching the fractional reading. */
  private static double findClosestContinuousTooth(
      double rawFractionalTeeth, int teeth, int targetN) {
    double bestContinuousTooth = 0.0;
    double bestError = Double.MAX_VALUE;

    // Search a few rotations around the target integer to find the exact continuous
    // match
    int kStart = (targetN / teeth) - 3;
    int kEnd = (targetN / teeth) + 3;

    for (int k = kStart; k <= kEnd; k++) {
      double candidateTooth = rawFractionalTeeth + k * teeth;
      double error = Math.abs(candidateTooth - targetN);
      if (error < bestError) {
        bestError = error;
        bestContinuousTooth = candidateTooth;
      }
    }
    return bestContinuousTooth;
  }

  /**
   * Extended Euclidean Algorithm.
   *
   * <p>Solves a*x + b*y = gcd(a, b) for integers x, y.
   *
   * <p><strong>Reference</strong>: Knuth, TAOCP Vol. 2, Algorithm X (Extended Euclidean).
   *
   * @param a Non-negative integer.
   * @param b Non-negative integer.
   * @return long[3] = {gcd, x, y} such that a*x + b*y = gcd.
   */
  static long[] extendedGcd(long a, long b) {
    if (b == 0) {
      return new long[] {a, 1, 0};
    }
    long[] result = extendedGcd(b, a % b);
    long gcd = result[0];
    long x = result[2];
    long y = result[1] - (a / b) * result[2];
    return new long[] {gcd, x, y};
  }
}
