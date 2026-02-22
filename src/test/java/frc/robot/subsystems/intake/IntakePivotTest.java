package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

public class IntakePivotTest {
  @Test
  public void testVariableKGFeedforward() {
    InterpolatingDoubleTreeMap gravityMap = new InterpolatingDoubleTreeMap();
    for (double[] point : Constants.IntakePivot.kGravityMap) {
      gravityMap.put(point[0], point[1]);
    }

    // Test Point 1: 23 degrees
    // kG should be 0.95
    // Expected FF = 0.95 * cos(23)
    double kG_23 = gravityMap.get(23.0);
    assertEquals(0.95, kG_23, 0.001);
    double expectedFF_23 = 0.95 * edu.wpi.first.math.geometry.Rotation2d.fromDegrees(23).getCos();
    // This test simulates the logic inside IntakePivot, assuming the map is loaded
    // correctly.
    assertEquals(
        expectedFF_23,
        kG_23 * edu.wpi.first.math.geometry.Rotation2d.fromDegrees(23).getCos(),
        0.001);

    // Test Point 2: 70 degrees
    // kG should be 1.15
    // Expected FF = 1.15 * cos(70)
    double kG_70 = gravityMap.get(70.0);
    assertEquals(1.15, kG_70, 0.001);

    // Test Point 3: 90 degrees
    // kG should be 1.55
    // Expected FF = 1.55 * cos(90) = 0.0
    double kG_90 = gravityMap.get(90.0);
    assertEquals(1.55, kG_90, 0.001);
    assertEquals(
        0.0, kG_90 * edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90).getCos(), 0.001);

    // Test Interpolation: 46.5 degrees (midpoint between 23 and 70)
    // Between 23(0.95) and 50(1.00) => (1.00 - 0.95) / (50 - 23) * (46.5 - 23) +
    // 0.95 = 0.9935
    double kG_interp = gravityMap.get(46.5);
    assertEquals(0.9935, kG_interp, 0.01);
  }
}
