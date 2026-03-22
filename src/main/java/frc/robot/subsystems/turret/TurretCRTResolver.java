package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;

/**
 * Vernier (Differential) Resolver: Calculates unique turret ring position from two absolute
 * encoders.
 *
 * <p>Replaces the old Chinese Remainder Theorem logic which is prone to skipping due to backlash.
 *
 * <p><strong>Mathematical Principle</strong>: Calculates the difference between the two encoder
 * readings. Because one gear has 19 teeth and the other has 20 teeth, their difference changes
 * smoothly and uniquely over a full rotation of the 87T ring gear.
 */
public final class TurretCRTResolver {
  private TurretCRTResolver() {}

  /**
   * @param e1Rotations Raw absolute encoder reading for gear 1 (19T), in [0, 1) rotations.
   * @param e2Rotations Raw absolute encoder reading for gear 2 (20T), in [0, 1) rotations.
   * @param teeth1 Tooth count for gear 1 (19T)
   * @param teeth2 Tooth count for gear 2 (20T)
   * @param ringTeeth Tooth count for turret ring (87T)
   * @param e1OffsetRotations Offset for gear 1, defined such that true zero is when raw == -offset
   * @param e2OffsetRotations Offset for gear 2, defined such that true zero is when raw == -offset
   * @return Absolute turret rotation in radians.
   */
  public static double resolveTurretAngleRads(
      double e1Rotations,
      double e2Rotations,
      int teeth1,
      int teeth2,
      int ringTeeth,
      double e1OffsetRotations,
      double e2OffsetRotations) {

    // 1. Remove offset and wrap to [0, 1)
    double e1_clean = MathUtil.inputModulus(e1Rotations + e1OffsetRotations, 0.0, 1.0);
    if (e1_clean < 0) e1_clean += 1.0;

    double e2_clean = MathUtil.inputModulus(e2Rotations + e2OffsetRotations, 0.0, 1.0);
    if (e2_clean < 0) e2_clean += 1.0;

    // 2. Calculate the difference. Since gear1 and gear2 rotate backwards when ring
    // rotates forwards:
    // G1 = -x * (87/19), G2 = -x * (87/20)
    // difference = G1 - G2 = -x * 87 * (1/19 - 1/20) = -x * 87 / (19 * 20)
    double diff = MathUtil.inputModulus(e1_clean - e2_clean, -0.5, 0.5);

    // 3. Rough turret rotations (x_rough)
    // diff = -x * ringTeeth / (teeth1 * teeth2)
    // x_rough = -diff * (teeth1 * teeth2) / ringTeeth
    double x_rough = -diff * (teeth1 * teeth2) / (double) ringTeeth;

    // 4. Find the continuous rotations of gear 1 (G1_rough)
    // G1 = -x * (ringTeeth / teeth1)
    double G1_rough = -x_rough * ((double) ringTeeth / teeth1);

    // 5. Determine the integer number of full rotations gear 1 has made
    long integerRotations = Math.round(G1_rough - e1_clean);

    // 6. Calculate exactly how many rotations gear 1 has made
    double G1_exact = e1_clean + integerRotations;

    // 7. Back-calculate the exact turret rotations
    double x_exact_rotations = -G1_exact * ((double) teeth1 / ringTeeth);

    // 8. Convert to radians and return
    return x_exact_rotations * 2.0 * Math.PI;
  }
}
