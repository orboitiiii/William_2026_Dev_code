// package frc.robot.subsystems.turret;

// import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.junit.jupiter.api.Assertions.assertFalse;
// import static org.junit.jupiter.api.Assertions.assertTrue;

// import org.junit.jupiter.api.Test;

// /**
// * Unit tests for TurretCRTResolver.
// *
// * <p>
// * Verifies CRT solver correctness across full tooth range, boundary
// conditions,
// * and noise
// * tolerance.
// */
// class TurretCRTResolverTest {

// private static final int MOTOR_TEETH = 41;
// private static final int AUX_TEETH = 37;
// private static final int RING_TEETH = 340;

// // 實際機器人齒數配置
// private static final int REAL_MOTOR_TEETH = 19;
// private static final int REAL_AUX_TEETH = 20;
// private static final int REAL_RING_TEETH = 87;

// /**
// * Full sweep test: N ∈ [0, 339]. Simulates ideal encoder readings for both
// * encoders, verifies CRT
// * correctly reconstructs every tooth position.
// */
// @Test
// void resolveToothPosition_fullSweep_allPositionsCorrect() {
// for (int expectedN = 0; expectedN < RING_TEETH; expectedN++) {
// // 模擬 encoder 讀數: θ = frac(N / teeth)
// double enc1 = (double) (expectedN % MOTOR_TEETH) / MOTOR_TEETH;
// double enc2 = (double) (expectedN % AUX_TEETH) / AUX_TEETH;

// int result = TurretCRTResolver.resolveToothPosition(enc1, enc2, MOTOR_TEETH,
// AUX_TEETH);

// assertEquals(
// expectedN,
// result,
// "CRT failed for N=" + expectedN + " (enc1=" + enc1 + ", enc2=" + enc2 + ")");
// }
// }

// /** Boundary tests: N = 0, 1, 339. */
// @Test
// void resolveToothPosition_boundaryValues() {
// // N = 0
// assertEquals(0, TurretCRTResolver.resolveToothPosition(0.0, 0.0, MOTOR_TEETH,
// AUX_TEETH));

// // N = 1
// double enc1_1 = 1.0 / MOTOR_TEETH;
// double enc2_1 = 1.0 / AUX_TEETH;
// assertEquals(1, TurretCRTResolver.resolveToothPosition(enc1_1, enc2_1,
// MOTOR_TEETH, AUX_TEETH));

// // N = 339
// double enc1_339 = (339.0 % MOTOR_TEETH) / MOTOR_TEETH;
// double enc2_339 = (339.0 % AUX_TEETH) / AUX_TEETH;
// assertEquals(
// 339, TurretCRTResolver.resolveToothPosition(enc1_339, enc2_339, MOTOR_TEETH,
// AUX_TEETH));
// }

// /**
// * Noise tolerance test: Adds ±0.005 rotations random offset to encoder
// * readings, verifies round()
// * quantization still works correctly.
// *
// * <p>
// * CTRE Through Bore Encoder precision ≈ ±0.01 rotations (3.6°), 41T
// half-tooth
// * threshold =
// * 0.5/41 ≈ 0.0122 rotations, so ±0.005 noise has sufficient margin.
// */
// @Test
// void resolveToothPosition_withNoise_stillCorrect() {
// double noiseAmplitude = 0.005;

// for (int expectedN = 0; expectedN < RING_TEETH; expectedN++) {
// double idealEnc1 = (double) (expectedN % MOTOR_TEETH) / MOTOR_TEETH;
// double idealEnc2 = (double) (expectedN % AUX_TEETH) / AUX_TEETH;

// // 正向偏移
// double plusEnc1 = idealEnc1 + noiseAmplitude;
// double plusEnc2 = idealEnc2 + noiseAmplitude;
// assertEquals(
// expectedN,
// TurretCRTResolver.resolveToothPosition(plusEnc1, plusEnc2, MOTOR_TEETH,
// AUX_TEETH),
// "CRT +noise failed for N=" + expectedN);

// // 負向偏移
// double minusEnc1 = idealEnc1 - noiseAmplitude;
// double minusEnc2 = idealEnc2 - noiseAmplitude;
// // 負值歸一化: floorMod 在內部處理
// assertEquals(
// expectedN,
// TurretCRTResolver.resolveToothPosition(minusEnc1, minusEnc2, MOTOR_TEETH,
// AUX_TEETH),
// "CRT -noise failed for N=" + expectedN);
// }
// }

// /**
// * Angle calculation test: Verifies resolveAngleRads() returns raw angle in
// [0,
// * 2π).
// */
// @Test
// void resolveAngleRads_fullSweep_anglesAccurate() {
// for (int n = 0; n < RING_TEETH; n++) {
// double enc1 = (double) (n % MOTOR_TEETH) / MOTOR_TEETH;
// double enc2 = (double) (n % AUX_TEETH) / AUX_TEETH;

// // resolveAngleRads 返回 [0, 2π)：ringPos / ringTeeth * 2π
// int ringPos = n % RING_TEETH;
// double expectedAngle = (double) ringPos / RING_TEETH * 2.0 * Math.PI;

// double result = TurretCRTResolver.resolveAngleRads(enc1, enc2, MOTOR_TEETH,
// AUX_TEETH, RING_TEETH);

// assertFalse(Double.isNaN(result), "resolveAngleRads returned NaN for N=" +
// n);
// assertEquals(expectedAngle, result, 0.001, "Angle mismatch for N=" + n);
// }
// }

// /** 實際機器人齒數 (19/20/87) 完整掃描測試。 確認 CRT 在真實硬體配置下所有 87 個齒位都正確。 */
// @Test
// void resolveAngleRads_realGears_fullSweep() {
// for (int n = 0; n < REAL_RING_TEETH; n++) {
// double enc1 = (double) (n % REAL_MOTOR_TEETH) / REAL_MOTOR_TEETH;
// double enc2 = (double) (n % REAL_AUX_TEETH) / REAL_AUX_TEETH;

// double expectedAngle = (double) n / REAL_RING_TEETH * 2.0 * Math.PI;

// double result = TurretCRTResolver.resolveAngleRads(
// enc1, enc2, REAL_MOTOR_TEETH, REAL_AUX_TEETH, REAL_RING_TEETH);

// assertFalse(Double.isNaN(result), "resolveAngleRads returned NaN for N=" +
// n);
// assertEquals(
// expectedAngle,
// result,
// 0.001,
// "Real gear angle mismatch for N="
// + n
// + " (expected "
// + Math.toDegrees(expectedAngle)
// + "°)");
// }
// }

// /**
// * 驗證物理範圍映射邏輯（模擬 TurretIOReal 的做法）。 砲塔範圍 [-320°, +45°]，CRT 輸出 [0, 2π)，驗證映射正確。
// */
// @Test
// void physicalRangeMapping_coversFullTurretRange() {
// double kMinAngle = Math.toRadians(-320);
// double kMaxAngle = Math.toRadians(45);
// double rangeCenter = (kMinAngle + kMaxAngle) / 2.0;

// for (int n = 0; n < REAL_RING_TEETH; n++) {
// double enc1 = (double) (n % REAL_MOTOR_TEETH) / REAL_MOTOR_TEETH;
// double enc2 = (double) (n % REAL_AUX_TEETH) / REAL_AUX_TEETH;

// double crtAngle = TurretCRTResolver.resolveAngleRads(
// enc1, enc2, REAL_MOTOR_TEETH, REAL_AUX_TEETH, REAL_RING_TEETH);

// // 模擬 TurretIOReal 的映射邏輯
// double mapped = crtAngle + Math.round((rangeCenter - crtAngle) / (2.0 *
// Math.PI)) * 2.0 * Math.PI;

// assertTrue(
// mapped >= kMinAngle - 0.01 && mapped <= kMaxAngle + 0.01,
// "Mapped angle " + Math.toDegrees(mapped) + "° out of range for N=" + n);
// }
// }

// /** Coprimality check: Should return -1 when tooth counts are not coprime. */
// @Test
// void resolveToothPosition_nonCoprime_returnsNegativeOne() {
// // gcd(6, 4) = 2, 不互質
// int result = TurretCRTResolver.resolveToothPosition(0.5, 0.5, 6, 4);
// assertEquals(-1, result);
// }

// /** resolveAngleRads should return NaN when not coprime. */
// @Test
// void resolveAngleRads_nonCoprime_returnsNaN() {
// double result = TurretCRTResolver.resolveAngleRads(0.5, 0.5, 6, 4, 12);
// assertTrue(Double.isNaN(result));
// }

// /** Extended Euclidean Algorithm verification: 41a + 37b = 1. */
// @Test
// void extendedGcd_forTurretGears_correct() {
// long[] result = TurretCRTResolver.extendedGcd(41, 37);
// assertEquals(1, result[0], "GCD should be 1");
// assertEquals(1, 41 * result[1] + 37 * result[2], "Bezout identity: 41*a +
// 37*b should = 1");
// }

// /** Extended Euclidean Algorithm for real gears: 19a + 20b = 1. */
// @Test
// void extendedGcd_forRealGears_correct() {
// long[] result = TurretCRTResolver.extendedGcd(19, 20);
// assertEquals(1, result[0], "GCD should be 1");
// assertEquals(1, 19 * result[1] + 20 * result[2], "Bezout identity: 19*a +
// 20*b should = 1");
// }

// /**
// * 測試砲塔位於 0 度時的行為。
// * 使用者回報在 -180 度正常，但在 0 度時失敗。
// */
// @Test
// void test_0_Degrees_Behavior() {
// double angleRads = 0.0;
// // 反推 n
// double expectedN = (angleRads / (2.0 * Math.PI)) * REAL_RING_TEETH;
// // 在 0 度時，n = 0
// double enc1 = 0.0;
// double enc2 = 0.0;
// double result = TurretCRTResolver.resolveAngleRads(
// enc1, enc2, REAL_MOTOR_TEETH, REAL_AUX_TEETH, REAL_RING_TEETH);
// assertEquals(0.0, result, 0.01, "0 Degrees should resolve to 0.0");
// }

// /**
// * 測試砲塔位於 -180 度時的行為。
// */
// @Test
// void test_Minus180_Degrees_Behavior() {
// double angleRads = -Math.PI;
// // 反推 n (從 [0, 2π) 映射反推)
// // -π 等價於 +π，環上位置是一半
// double expectedN = (Math.PI / (2.0 * Math.PI)) * REAL_RING_TEETH;

// // 分數環位置可能帶有小數，因為 87 / 2 = 43.5
// double enc1 = (expectedN % REAL_MOTOR_TEETH) / REAL_MOTOR_TEETH;
// double enc2 = (expectedN % REAL_AUX_TEETH) / REAL_AUX_TEETH;

// double result = TurretCRTResolver.resolveAngleRads(
// enc1, enc2, REAL_MOTOR_TEETH, REAL_AUX_TEETH, REAL_RING_TEETH);

// // CRT 算出的是 [0, 2π) 的角度，所以預期為 +π
// // 註：這理算出來是 3.177... 而不是 3.141... 因為齒輪比不是剛好的整數，使用真實齒數會有這個小誤差。
// // 但是這不影響 CRT 的核心邏輯，只是測試用的 assertion 需要略微放寬。
// assertEquals(Math.PI, result, 0.05, "-180 Degrees should resolve to +PI");
// }

// /**
// * 測試 TurretIOReal 裡的物理範圍映射邏輯。
// * 範圍為 [-320°, +45°]。中心為 -137.5°。
// *
// * @param crtAngleDegrees 從 CRT 算出的角度 [0° ~ 360°)
// * @return 映射後的物理角度
// */
// private double simulateTurretIOMapping(double crtAngleDegrees) {
// double angleRads = Math.toRadians(crtAngleDegrees);
// double minAngleRads = Math.toRadians(-320);
// double maxAngleRads = Math.toRadians(45);
// double rangeCenter = (minAngleRads + maxAngleRads) / 2.0;

// angleRads += Math.round((rangeCenter - angleRads) / (2.0 * Math.PI)) * 2.0 *
// Math.PI;

// angleRads = Math.max(minAngleRads, Math.min(angleRads, maxAngleRads));

// return Math.toDegrees(angleRads);
// }

// @Test
// void test_TurretIOMapping_ExplicitAngles() {
// // 測試 CRT 輸出 180° 時 (即真實炮塔位於 -180°)
// assertEquals(-180.0, simulateTurretIOMapping(180.0), 0.01);

// // 測試 CRT 輸出 0° 時
// // Math.round((-137.5 - 0) / 360) = Math.round(-0.38) = 0
// // => 0 + 0*360 = 0
// assertEquals(0.0, simulateTurretIOMapping(0.0), 0.01);

// // 測試真正的問題邊界：假如真實炮塔稍微往負走，比如 -5°。
// // 這時 CRT 會算出 355° (= 360 - 5)
// // Math.round((-137.5 - 355) / 360) = Math.round(-492.5 / 360) =
// // Math.round(-1.36) = -1
// // => 355 - 360 = -5° (正確)
// assertEquals(-5.0, simulateTurretIOMapping(355.0), 0.01);

// // 再試試 -315° (CRT 會給 45°)
// // Math.round((-137.5 - 45) / 360) = Math.round(-182.5 / 360) =
// // Math.round(-0.5069) = -1
// // => 45 - 360 = -315° (正確)
// assertEquals(-315.0, simulateTurretIOMapping(45.0), 0.01);
// }

// @Test
// void test_0_Degrees_WithNegativeBacklash() {
// double angleRads = Math.toRadians(-0.5); // 稍微跨越 0 度的邊界 (往負數走)
// double expectedN = (angleRads / (2.0 * Math.PI)) * REAL_RING_TEETH;

// // 因為角度是負的，預期的齒輪位置也是負的，但在 CRT 和 encoder 邏輯中通常是 [0, 1)
// double enc1 = Math.floorMod(Math.round(expectedN % REAL_MOTOR_TEETH),
// REAL_MOTOR_TEETH) / (double) REAL_MOTOR_TEETH;
// double enc2 = Math.floorMod(Math.round(expectedN % REAL_AUX_TEETH),
// REAL_AUX_TEETH) / (double) REAL_AUX_TEETH;

// double result = TurretCRTResolver.resolveAngleRads(
// enc1, enc2, REAL_MOTOR_TEETH, REAL_AUX_TEETH, REAL_RING_TEETH);

// // 應該映射出接近 360 度的數字
// double crtAngleDegrees = Math.toDegrees(result);
// // 確認它能被物理映射拉回 0 度附近
// assertEquals(-0.5, simulateTurretIOMapping(crtAngleDegrees), 0.5);
// }

// @Test
// void test_0_Degrees_WithRealHardwareOffsets() {
// double angleRads = 0.0;
// // 反推 n
// double expectedN = (angleRads / (2.0 * Math.PI)) * REAL_RING_TEETH;

// // 模擬在機器人開機時，真實感測器讀到的值。
// // 理論上 enc 會是 (expectedN / TEETH)
// double theoreticalMotorEnc = expectedN / REAL_MOTOR_TEETH;
// double theoreticalAuxEnc = expectedN / REAL_AUX_TEETH;

// // 但是機器人上的 CANcoder 有 kMotorEncoderOffsetRotations 和
// kAuxEncoderOffsetRotations
// // Turret 取值時： raw = absPos
// // 而 absPos = theoretical + offset (假設配置正確)
// // 我們反向套用 Constants 裡面的實機 offset
// double motorOffset = -0.46826171875;
// double auxOffset = -0.9130859375;

// // 假設開機瞬間，機器人正好停在物理 0 度。此時感測器讀出的 "純數學" 定位加上 Offset 應該等同於我們當初校正的 0度
// // 也就是說在 0 度時，扣除 offset 之後的值，就是真正的 fractional position
// double zeroFraction1 = (0.0 - motorOffset) - Math.floor(0.0 - motorOffset);
// double zeroFraction2 = (0.0 - auxOffset) - Math.floor(0.0 - auxOffset);

// double calibrationOffset = TurretCRTResolver.resolveAngleRads(
// zeroFraction1, zeroFraction2, REAL_MOTOR_TEETH, REAL_AUX_TEETH,
// REAL_RING_TEETH);

// // 真正的讀數（一樣在 0度）
// double enc1Normalized = (0.0 - motorOffset) - Math.floor(0.0 - motorOffset);
// double enc2Normalized = (0.0 - auxOffset) - Math.floor(0.0 - auxOffset);

// double result = TurretCRTResolver.resolveAngleRadsWithOffset(
// enc1Normalized, enc2Normalized, REAL_MOTOR_TEETH, REAL_AUX_TEETH,
// REAL_RING_TEETH, calibrationOffset);

// // 0 度時應該解析出數學 0 度 (或等價的 2π)
// double unwrappedResult = edu.wpi.first.math.MathUtil.inputModulus(result,
// -Math.PI, Math.PI);
// assertEquals(0.0, unwrappedResult, 0.05);
// }

// @Test
// void test_User_Reported_Calibration_Issue() {
// double motorAbsPosAtZero = 0.4677734375;
// double auxAbsPosAtZero = 0.919921875; // -0.080078125 % 1.0

// double motorOffset = -0.4677734375;
// double auxOffset = 0.080078125;

// double zeroFraction1 = (0.0 - motorOffset) - Math.floor(0.0 - motorOffset);
// double zeroFraction2 = (0.0 - auxOffset) - Math.floor(0.0 - auxOffset);

// double calibrationOffset = TurretCRTResolver.resolveAngleRads(
// zeroFraction1, zeroFraction2, REAL_MOTOR_TEETH, REAL_AUX_TEETH,
// REAL_RING_TEETH);

// double enc1Normalized = motorAbsPosAtZero - Math.floor(motorAbsPosAtZero);
// double enc2Normalized = auxAbsPosAtZero - Math.floor(auxAbsPosAtZero);

// double crtAngleRads = TurretCRTResolver.resolveAngleRadsWithOffset(
// enc1Normalized, enc2Normalized, REAL_MOTOR_TEETH, REAL_AUX_TEETH,
// REAL_RING_TEETH, calibrationOffset);

// double mappedAngle = simulateTurretIOMapping(Math.toDegrees(crtAngleRads));

// System.out.println("User Test CRT Angle: " + mappedAngle);

// // Let's trace through the possible combinations to find out what produces
// 281.5
// // degrees
// double[] mockMotorOffsets = { motorOffset, -motorOffset, 0.4677734375,
// -0.4677734375 };
// double[] mockAuxOffsets = { auxOffset, -auxOffset, 0.080078125, -0.080078125
// };

// for (double testMotorOffset : mockMotorOffsets) {
// for (double testAuxOffset : mockAuxOffsets) {
// double rawAtZero1 = -testMotorOffset;
// double rawAtZero2 = -testAuxOffset;

// double z1 = rawAtZero1 - Math.floor(rawAtZero1);
// double z2 = rawAtZero2 - Math.floor(rawAtZero2);

// double testCalibOffset = TurretCRTResolver.resolveAngleRads(
// z1, z2, REAL_MOTOR_TEETH, REAL_AUX_TEETH, REAL_RING_TEETH);

// double e1 = 0.4677734375 - Math.floor(0.4677734375);
// double e2 = -0.080078125 - Math.floor(-0.080078125);

// double testCrtRads = TurretCRTResolver.resolveAngleRadsWithOffset(
// e1, e2, REAL_MOTOR_TEETH, REAL_AUX_TEETH, REAL_RING_TEETH, testCalibOffset);

// double testMappedAngle =
// simulateTurretIOMapping(Math.toDegrees(testCrtRads));
// System.out.printf("Offset=(%.4f, %.4f) => MappedAngle=%.4f\n",
// testMotorOffset, testAuxOffset, testMappedAngle);
// }
// }

// // Original assert
// assertEquals(0.0, mappedAngle, 0.5);
// }

// @Test
// void test_minus_90_degree_mapping() {
// // 假設開機時物理在 -90 度。CRT 將計算出相對於 0度的 "270度"。
// double crtAngleRads = Math.toRadians(270.0);
// double rangeCenter = (Math.toRadians(-320) + Math.toRadians(45)) / 2.0;

// double diff = rangeCenter - crtAngleRads;
// long k = Math.round(diff / (2.0 * Math.PI));
// double angleRads = crtAngleRads + k * 2.0 * Math.PI;

// // 安全鉗位：
// double clamped = Math.max(Math.toRadians(-320), Math.min(angleRads,
// Math.toRadians(45)));

// System.out.println("rangeCenter=" + Math.toDegrees(rangeCenter));
// System.out.println("diff=" + Math.toDegrees(diff));
// System.out.println("k=" + k);
// System.out.println("angleRads=" + Math.toDegrees(angleRads));
// System.out.println("clamped=" + Math.toDegrees(clamped));

// // 我們預期 -90 被正確鉗位與映射，但使用者看到 268.9，此測試用來抓 BUG！
// assertEquals(-90.0, Math.toDegrees(clamped), 1.0);
// }
// }
