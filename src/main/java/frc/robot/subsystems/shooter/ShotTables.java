package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * ShotTables - Simulation-Generated Shooting Lookup Tables.
 *
 * <p>Uses {@link InterpolatingDoubleTreeMap} for linear interpolation between 25 calibration nodes
 * spanning [0.50, 6.50] m at 0.25 m intervals.
 *
 * <p>
 *
 * <p><strong>Generation Parameters (2026-03-06 V5)</strong>:
 *
 * <ul>
 *   <li>Physics: Gravity + quadratic drag (Cd=0.485), no Magnus/backspin
 *   <li>Solver: RK4 @ 0.5ms step, bisection on angle and velocity
 *   <li>Constraint: Hybrid — d&le;2.5m apex&le;3.5m / d&gt;2.5m apex&le;4.0m / d&gt;4.5m +offset
 *       boost
 *   <li>Shooter: 4" Stealth Wheel, slip=0.4993 (calibrated 2026-02-20), Reff=0.02536 m
 *   <li>Launch height: 0.50 m, Target height: 1.8288 m (72 in)
 * </ul>
 *
 * <p>At 6.00-6.50 m the hood hits its 60 deg mechanical lower limit, so apex slightly exceeds 4.0 m
 * (4.00-4.37 m). This is physically unavoidable.
 *
 * <p><strong>Empirical Calibration</strong>: slip factor corrected from 0.435 to 0.4993 based on
 * field testing (all shots overshooting by ~1.20 m at slip=0.435).
 */
public final class ShotTables {

  private ShotTables() {}

  public static final double MIN_DISTANCE_M = 0.50;
  public static final double MAX_DISTANCE_M = 6.50;

  public static final double MIN_PASSING_DISTANCE_M = 2.00;
  public static final double MAX_PASSING_DISTANCE_M = 12.00;

  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_MAP = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap FLYWHEEL_SPEED_MAP =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_MAP =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap MAX_SPINNER_RPS_MAP =
      new InterpolatingDoubleTreeMap();

  private static final InterpolatingDoubleTreeMap PASSING_HOOD_ANGLE_MAP =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap PASSING_FLYWHEEL_SPEED_MAP =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap PASSING_TIME_OF_FLIGHT_MAP =
      new InterpolatingDoubleTreeMap();

  private static double sHoodOffsetRad = 0.0;
  private static double sFlywheelOffsetRotPerSec = 0.0;

  static {
    // Hood Angle [rad] vs Distance [m]
    // Generated 2026-03-06 V5: NEAR(d<=2.5m apex<=3.5m, no offset)
    // MID(2.5<d<=4.5m apex<=4.0m, V3 offset=0.35*(d-2.6))
    // FAR(d>4.5m apex<=4.0m, V3 offset + boost=0.30*(d-4.5))
    // Cd=0.485, slip=0.4993
    HOOD_ANGLE_MAP.put(0.50, 1.448623); // 83.0 deg, apex=2.02m
    HOOD_ANGLE_MAP.put(0.75, 1.448623); // 83.0 deg, apex=2.48m
    HOOD_ANGLE_MAP.put(1.00, 1.448623); // 83.0 deg, apex=2.99m /* V3: 83.0deg apex=2.02m */
    HOOD_ANGLE_MAP.put(1.25, 1.447976); // 83.0 deg, apex=3.50m /* V3: 83.0deg apex=2.02m */
    HOOD_ANGLE_MAP.put(1.50, 1.423500); // 81.6 deg, apex=3.50m /* V3: 83.0deg apex=2.54m */
    HOOD_ANGLE_MAP.put(1.75, 1.399376); // 80.2 deg, apex=3.50m /* V3: 83.0deg apex=3.69m */
    HOOD_ANGLE_MAP.put(2.00, 1.375218); // 78.8 deg, apex=3.50m /* V3: 81.1deg apex=4.00m */
    HOOD_ANGLE_MAP.put(2.25, 1.351256); // 77.4 deg, apex=3.50m /* V3: 79.4deg apex=4.00m */
    HOOD_ANGLE_MAP.put(2.50, 1.327392); // 76.1 deg, apex=3.50m /* V3: 78.2deg apex=4.00m */
    HOOD_ANGLE_MAP.put(2.75, 1.340427); // 76.8 deg, apex=4.00m // =V3
    HOOD_ANGLE_MAP.put(3.00, 1.313328); // 75.2 deg, apex=4.00m // =V3
    HOOD_ANGLE_MAP.put(3.25, 1.286622); // 73.7 deg, apex=4.00m // =V3
    HOOD_ANGLE_MAP.put(3.50, 1.260210); // 72.2 deg, apex=4.00m // =V3
    HOOD_ANGLE_MAP.put(3.75, 1.234043); // 70.7 deg, apex=4.00m // =V3
    HOOD_ANGLE_MAP.put(4.00, 1.208317); // 69.2 deg, apex=4.00m // =V3
    HOOD_ANGLE_MAP.put(4.25, 1.183032); // 67.8 deg, apex=4.00m // =V3
    HOOD_ANGLE_MAP.put(4.50, 1.158040); // 66.4 deg, apex=4.00m // =V3
    HOOD_ANGLE_MAP.put(4.75, 1.128051); // 64.6 deg, apex=4.00m /* V3: 64.9deg */
    HOOD_ANGLE_MAP.put(5.00, 1.098895); // 63.0 deg, apex=4.00m /* V3: 63.6deg */
    HOOD_ANGLE_MAP.put(5.25, 1.070229); // 61.3 deg, apex=4.00m /* V3: 62.2deg */
    HOOD_ANGLE_MAP.put(5.50, 1.047198); // 60.0 deg, apex=4.04m /* V3: 60.9deg apex=4.00m */
    HOOD_ANGLE_MAP.put(5.75, 1.047198); // 60.0 deg, apex=4.24m /* V3: 60.0deg apex=4.05m */
    HOOD_ANGLE_MAP.put(6.00, 1.047198); // 60.0 deg, apex=4.44m /* V3: 60.0deg apex=4.22m */
    HOOD_ANGLE_MAP.put(6.25, 1.047198); // 60.0 deg, apex=4.65m /* V3: 60.0deg apex=4.39m */
    HOOD_ANGLE_MAP.put(6.50, 1.047198); // 60.0 deg, apex=4.86m /* V3: 60.0deg apex=4.56m */

    // Flywheel Speed [rot/s] vs Distance [m]
    // v_exit = rot/s * 2pi * 0.02536m
    FLYWHEEL_SPEED_MAP.put(0.50, 35.21); /* V3: 35.21 */
    FLYWHEEL_SPEED_MAP.put(0.75, 40.39); /* V3: 35.21 */
    FLYWHEEL_SPEED_MAP.put(1.00, 45.57); /* V3: 35.21 */
    FLYWHEEL_SPEED_MAP.put(1.25, 50.35); /* V3: 40.19 */
    FLYWHEEL_SPEED_MAP.put(1.50, 50.53); /* V3: 46.50 */
    FLYWHEEL_SPEED_MAP.put(1.75, 50.75); /* V3: 53.50 */
    FLYWHEEL_SPEED_MAP.put(2.00, 50.99); /* V3: 55.05 */
    FLYWHEEL_SPEED_MAP.put(2.25, 51.26); /* V3: 55.31 */
    FLYWHEEL_SPEED_MAP.put(2.50, 51.56); /* V3: 55.55 */
    FLYWHEEL_SPEED_MAP.put(2.75, 55.88); // =V3
    FLYWHEEL_SPEED_MAP.put(3.00, 56.28); // =V3
    FLYWHEEL_SPEED_MAP.put(3.25, 56.74); // =V3
    FLYWHEEL_SPEED_MAP.put(3.50, 57.25); // =V3
    FLYWHEEL_SPEED_MAP.put(3.75, 57.79); // =V3
    FLYWHEEL_SPEED_MAP.put(4.00, 58.37); // =V3
    FLYWHEEL_SPEED_MAP.put(4.25, 59.01); // =V3
    FLYWHEEL_SPEED_MAP.put(4.50, 59.69); // =V3
    FLYWHEEL_SPEED_MAP.put(4.75, 60.57); /* V3: 60.41, +0.16 boost */
    FLYWHEEL_SPEED_MAP.put(5.00, 61.53); /* V3: 61.17, +0.36 boost */
    FLYWHEEL_SPEED_MAP.put(5.25, 62.55); /* V3: 61.98, +0.57 boost */
    FLYWHEEL_SPEED_MAP.put(5.50, 63.78); /* V3: 62.83, +0.95 boost */
    FLYWHEEL_SPEED_MAP.put(5.75, 65.80); /* V3: 63.97, +1.83 boost */
    FLYWHEEL_SPEED_MAP.put(6.00, 67.80); /* V3: 65.62, +2.18 boost */
    FLYWHEEL_SPEED_MAP.put(6.25, 69.79); /* V3: 67.26, +2.53 boost */
    FLYWHEEL_SPEED_MAP.put(6.50, 71.77); /* V3: 68.89, +2.88 boost */

    // Time of Flight [s] vs Distance [m]
    TIME_OF_FLIGHT_MAP.put(0.50, 0.752875); /* V3: 0.752875 */
    TIME_OF_FLIGHT_MAP.put(0.75, 0.995992); /* V3: 0.752875 */
    TIME_OF_FLIGHT_MAP.put(1.00, 1.193931); /* V3: 0.752875 */
    TIME_OF_FLIGHT_MAP.put(1.25, 1.360082); /* V3: 0.987807 */
    TIME_OF_FLIGHT_MAP.put(1.50, 1.360017); /* V3: 1.228340 */
    TIME_OF_FLIGHT_MAP.put(1.75, 1.360254); /* V3: 1.465420 */
    TIME_OF_FLIGHT_MAP.put(2.00, 1.360015); /* V3: 1.503745 */
    TIME_OF_FLIGHT_MAP.put(2.25, 1.360030); /* V3: 1.503477 */
    TIME_OF_FLIGHT_MAP.put(2.50, 1.359597); /* V3: 1.503313 */
    TIME_OF_FLIGHT_MAP.put(2.75, 1.503675); // =V3
    TIME_OF_FLIGHT_MAP.put(3.00, 1.503232); // =V3
    TIME_OF_FLIGHT_MAP.put(3.25, 1.503268); // =V3
    TIME_OF_FLIGHT_MAP.put(3.50, 1.503418); // =V3
    TIME_OF_FLIGHT_MAP.put(3.75, 1.503037); // =V3
    TIME_OF_FLIGHT_MAP.put(4.00, 1.502852); // =V3
    TIME_OF_FLIGHT_MAP.put(4.25, 1.502867); // =V3
    TIME_OF_FLIGHT_MAP.put(4.50, 1.502670); // =V3
    TIME_OF_FLIGHT_MAP.put(4.75, 1.502390); /* V3: 1.502662 */
    TIME_OF_FLIGHT_MAP.put(5.00, 1.502420); /* V3: 1.502229 */
    TIME_OF_FLIGHT_MAP.put(5.25, 1.502164); /* V3: 1.502079 */
    TIME_OF_FLIGHT_MAP.put(5.50, 1.511490); /* V3: 1.502120 */
    TIME_OF_FLIGHT_MAP.put(5.75, 1.565491); /* V3: 1.516586 */
    TIME_OF_FLIGHT_MAP.put(6.00, 1.618022); /* V3: 1.560743 */
    TIME_OF_FLIGHT_MAP.put(6.25, 1.669423); /* V3: 1.603737 */
    TIME_OF_FLIGHT_MAP.put(6.50, 1.719504); /* V3: 1.646140 */

    // Maximum Spinner RPS [rot/s] vs Distance [m]
    // Anti-collision BPS limit:
    // At close range (high angle, high apex), horizontal velocity is very low at
    // apex.
    // Shooting balls too fast causes the second ball to collide with the first.
    // Based on ChiefDelphi physics threads: limit to ~3-4 BPS (15-20 RPS) up close.
    // Max indexer capability is ~25 RPS (5 BPS).
    // MAX_SPINNER_RPS_MAP.put(0.50, 15.0); // Extreme high arc
    // MAX_SPINNER_RPS_MAP.put(1.00, 15.0);
    // MAX_SPINNER_RPS_MAP.put(1.50, 20.0);
    // MAX_SPINNER_RPS_MAP.put(2.00, 22.5);
    // MAX_SPINNER_RPS_MAP.put(2.50, 25.0);
    // MAX_SPINNER_RPS_MAP.put(3.00, 25.5);
    // MAX_SPINNER_RPS_MAP.put(3.50, 31.0);
    // MAX_SPINNER_RPS_MAP.put(4.00, 33.0); // Flatter arc, horizontal speed is much
    // faster
    // MAX_SPINNER_RPS_MAP.put(4.50, 35.0); // Max throughput (25 RPS = 5.0
    // Balls/sec)
    // MAX_SPINNER_RPS_MAP.put(6.50, 45.0); // Max throughput

    // Passing Maps (RK4 Minimum Energy + 0.8m Boost)
    PASSING_HOOD_ANGLE_MAP.put(1.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(1.0, 26.87);
    PASSING_TIME_OF_FLIGHT_MAP.put(1.0, 0.8675);
    PASSING_HOOD_ANGLE_MAP.put(1.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(1.5, 31.02);
    PASSING_TIME_OF_FLIGHT_MAP.put(1.5, 0.9681);
    PASSING_HOOD_ANGLE_MAP.put(2.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(2.0, 34.77);
    PASSING_TIME_OF_FLIGHT_MAP.put(2.0, 1.0600);
    PASSING_HOOD_ANGLE_MAP.put(2.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(2.5, 38.26);
    PASSING_TIME_OF_FLIGHT_MAP.put(2.5, 1.1455);
    PASSING_HOOD_ANGLE_MAP.put(3.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(3.0, 41.54);
    PASSING_TIME_OF_FLIGHT_MAP.put(3.0, 1.2259);
    PASSING_HOOD_ANGLE_MAP.put(3.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(3.5, 44.65);
    PASSING_TIME_OF_FLIGHT_MAP.put(3.5, 1.3021);
    PASSING_HOOD_ANGLE_MAP.put(4.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(4.0, 47.62);
    PASSING_TIME_OF_FLIGHT_MAP.put(4.0, 1.3746);
    PASSING_HOOD_ANGLE_MAP.put(4.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(4.5, 50.50);
    PASSING_TIME_OF_FLIGHT_MAP.put(4.5, 1.4443);
    PASSING_HOOD_ANGLE_MAP.put(5.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(5.0, 53.29);
    PASSING_TIME_OF_FLIGHT_MAP.put(5.0, 1.5114);
    PASSING_HOOD_ANGLE_MAP.put(5.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(5.5, 56.00);
    PASSING_TIME_OF_FLIGHT_MAP.put(5.5, 1.5761);
    PASSING_HOOD_ANGLE_MAP.put(6.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(6.0, 58.65);
    PASSING_TIME_OF_FLIGHT_MAP.put(6.0, 1.6390);
    PASSING_HOOD_ANGLE_MAP.put(6.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(6.5, 61.25);
    PASSING_TIME_OF_FLIGHT_MAP.put(6.5, 1.7001);
    PASSING_HOOD_ANGLE_MAP.put(7.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(7.0, 63.82);
    PASSING_TIME_OF_FLIGHT_MAP.put(7.0, 1.7598);
    PASSING_HOOD_ANGLE_MAP.put(7.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(7.5, 66.34);
    PASSING_TIME_OF_FLIGHT_MAP.put(7.5, 1.8180);
    PASSING_HOOD_ANGLE_MAP.put(8.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(8.0, 68.84);
    PASSING_TIME_OF_FLIGHT_MAP.put(8.0, 1.8750);
    PASSING_HOOD_ANGLE_MAP.put(8.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(8.5, 71.30);
    PASSING_TIME_OF_FLIGHT_MAP.put(8.5, 1.9308);
    PASSING_HOOD_ANGLE_MAP.put(9.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(9.0, 73.76);
    PASSING_TIME_OF_FLIGHT_MAP.put(9.0, 1.9857);
    PASSING_HOOD_ANGLE_MAP.put(9.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(9.5, 76.19);
    PASSING_TIME_OF_FLIGHT_MAP.put(9.5, 2.0395);
    PASSING_HOOD_ANGLE_MAP.put(10.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(10.0, 78.61);
    PASSING_TIME_OF_FLIGHT_MAP.put(10.0, 2.0925);
    PASSING_HOOD_ANGLE_MAP.put(10.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(10.5, 80.00); // Need 81.0
    PASSING_TIME_OF_FLIGHT_MAP.put(10.5, 2.1227);
    PASSING_HOOD_ANGLE_MAP.put(11.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(11.0, 80.00); // Need 83.4
    PASSING_TIME_OF_FLIGHT_MAP.put(11.0, 2.1227);
    PASSING_HOOD_ANGLE_MAP.put(11.5, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(11.5, 80.00); // Need 85.8
    PASSING_TIME_OF_FLIGHT_MAP.put(11.5, 2.1227);
    PASSING_HOOD_ANGLE_MAP.put(12.0, 1.047198); // 60 deg
    PASSING_FLYWHEEL_SPEED_MAP.put(12.0, 80.00); // Need 88.2
    PASSING_TIME_OF_FLIGHT_MAP.put(12.0, 2.1227);
  }

  // ══════════════════════════════════════════════════════════════════
  // Public API — names match ShotCalculator expectations
  // ══════════════════════════════════════════════════════════════════

  /** Hood angle [rad] for the given distance, including tuning offset. */
  public static double hoodAngleRad(double distanceMeters) {
    distanceMeters = Math.max(MIN_DISTANCE_M, Math.min(distanceMeters, MAX_DISTANCE_M));
    return HOOD_ANGLE_MAP.get(distanceMeters) + sHoodOffsetRad;
  }

  /** Flywheel speed [rot/s] for the given distance, including tuning offset. */
  public static double flywheelSpeedRotPerSec(double distanceMeters) {
    distanceMeters = Math.max(MIN_DISTANCE_M, Math.min(distanceMeters, MAX_DISTANCE_M));
    return FLYWHEEL_SPEED_MAP.get(distanceMeters) + sFlywheelOffsetRotPerSec;
  }

  /** Time of flight [s] for the given distance. */
  public static double timeOfFlightS(double distanceMeters) {
    distanceMeters = Math.max(MIN_DISTANCE_M, Math.min(distanceMeters, MAX_DISTANCE_M));
    return TIME_OF_FLIGHT_MAP.get(distanceMeters);
  }

  public static double passingHoodAngleRad(double distanceMeters) {
    distanceMeters =
        Math.max(MIN_PASSING_DISTANCE_M, Math.min(distanceMeters, MAX_PASSING_DISTANCE_M));
    return PASSING_HOOD_ANGLE_MAP.get(distanceMeters) + sHoodOffsetRad;
  }

  public static double passingFlywheelSpeedRotPerSec(double distanceMeters) {
    distanceMeters =
        Math.max(MIN_PASSING_DISTANCE_M, Math.min(distanceMeters, MAX_PASSING_DISTANCE_M));
    return PASSING_FLYWHEEL_SPEED_MAP.get(distanceMeters) + sFlywheelOffsetRotPerSec;
  }

  public static double passingTimeOfFlightS(double distanceMeters) {
    distanceMeters =
        Math.max(MIN_PASSING_DISTANCE_M, Math.min(distanceMeters, MAX_PASSING_DISTANCE_M));
    return PASSING_TIME_OF_FLIGHT_MAP.get(distanceMeters);
  }

  /**
   * Numerical derivative of ToF w.r.t. distance [s/m]. Central difference with h = 1 cm for smooth
   * approximation.
   */
  public static double timeOfFlightDerivative(double distanceMeters) {
    double h = 0.01;
    double dPlus = Math.min(distanceMeters + h, MAX_DISTANCE_M);
    double dMinus = Math.max(distanceMeters - h, MIN_DISTANCE_M);
    return (timeOfFlightS(dPlus) - timeOfFlightS(dMinus)) / (dPlus - dMinus);
  }

  /** Maximum spinner RPS to avoid mid-air collisions at the given distance. */
  public static double maxSpinnerRps(double distanceMeters) {
    distanceMeters = Math.max(MIN_DISTANCE_M, Math.min(distanceMeters, MAX_DISTANCE_M));
    return MAX_SPINNER_RPS_MAP.get(distanceMeters);
  }

  /** Apply global tuning offsets at competition. */
  public static void setTuningOffsets(double hoodOffsetRad, double flywheelOffsetRotPerSec) {
    sHoodOffsetRad = hoodOffsetRad;
    sFlywheelOffsetRotPerSec = flywheelOffsetRotPerSec;
  }
}
