package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * ShotTables - Simulation-Generated Shooting Lookup Tables.
 *
 * <p>Uses {@link InterpolatingDoubleTreeMap} for linear interpolation between 25 calibration nodes
 * spanning [0.50, 6.50] m at 0.25 m intervals.
 *
 * <p><strong>Generation Parameters (2026-02-20)</strong>:
 *
 * <ul>
 *   <li>Physics: Gravity + quadratic drag (Cd=0.485), no Magnus/backspin
 *   <li>Solver: RK4 @ 0.5ms step, bisection on angle and velocity
 *   <li>Constraint: Trajectory apex &le; 3.5 m (angle-priority optimization)
 *   <li>Shooter: 4" Stealth Wheel, slip=0.4993 (calibrated 2026-02-20), Reff=0.02537 m
 *   <li>Launch height: 0.50 m, Target height: 1.8288 m (72 in)
 * </ul>
 *
 * <p>At 5.75-6.50 m the hood hits its 60 deg mechanical lower limit, so apex slightly exceeds 3.5 m
 * (3.52-3.88 m). This is physically unavoidable.
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
    // Generated 2026-02-20: apex <= 3.5m, Cd=0.485, slip=0.4993 (calibrated)
    HOOD_ANGLE_MAP.put(0.50, 1.448623); // 83.0 deg, apex=2.02m
    HOOD_ANGLE_MAP.put(0.75, 1.448623); // 83.0 deg, apex=2.48m
    HOOD_ANGLE_MAP.put(1.00, 1.448623); // 83.0 deg, apex=2.99m
    HOOD_ANGLE_MAP.put(1.25, 1.447937); // 83.0 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(1.50, 1.423485); // 81.6 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(1.75, 1.399376); // 80.2 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(2.00, 1.375218); // 78.8 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(2.25, 1.351256); // 77.4 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(2.50, 1.327392); // 76.1 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(2.75, 1.303822); // 74.7 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(3.00, 1.280546); // 73.4 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(3.25, 1.257466); // 72.0 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(3.50, 1.234631); // 70.7 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(3.75, 1.212041); // 69.4 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(4.00, 1.189892); // 68.2 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(4.25, 1.168037); // 66.9 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(4.50, 1.146476); // 65.7 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(4.75, 1.125160); // 64.5 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(5.00, 1.104236); // 63.3 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(5.25, 1.083802); // 62.1 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(5.50, 1.063515); // 60.9 deg, apex=3.50m
    HOOD_ANGLE_MAP.put(5.75, 1.047198); // 60.0 deg, apex=3.52m (hood limit)
    HOOD_ANGLE_MAP.put(6.00, 1.047198); // 60.0 deg, apex=3.64m (hood limit)
    HOOD_ANGLE_MAP.put(6.25, 1.047198); // 60.0 deg, apex=3.76m (hood limit)
    HOOD_ANGLE_MAP.put(6.50, 1.047198); // 60.0 deg, apex=3.88m (hood limit)

    // Flywheel Speed [rot/s] vs Distance [m]
    // Calibrated slip=0.4993, Reff = 0.0508 * 0.4993 = 0.02537 m
    // v_exit = rot/s * 2pi * 0.02537m
    FLYWHEEL_SPEED_MAP.put(0.50, 34.20);
    FLYWHEEL_SPEED_MAP.put(0.75, 39.30);
    FLYWHEEL_SPEED_MAP.put(1.00, 44.40);
    FLYWHEEL_SPEED_MAP.put(1.25, 49.10);
    FLYWHEEL_SPEED_MAP.put(1.50, 49.30);
    FLYWHEEL_SPEED_MAP.put(1.75, 49.50);
    FLYWHEEL_SPEED_MAP.put(2.00, 49.70);
    FLYWHEEL_SPEED_MAP.put(2.25, 50.00);
    FLYWHEEL_SPEED_MAP.put(2.50, 50.30);
    FLYWHEEL_SPEED_MAP.put(2.75, 50.70);
    FLYWHEEL_SPEED_MAP.put(3.00, 51.60);
    FLYWHEEL_SPEED_MAP.put(3.25, 52.80);
    FLYWHEEL_SPEED_MAP.put(3.50, 54.20);
    FLYWHEEL_SPEED_MAP.put(3.75, 55.60); // Anchor point: user confirmed 3.79m works
    FLYWHEEL_SPEED_MAP.put(4.00, 57.40); // Starts scaling up to overcome drag
    FLYWHEEL_SPEED_MAP.put(4.25, 59.30);
    FLYWHEEL_SPEED_MAP.put(4.50, 61.30);
    FLYWHEEL_SPEED_MAP.put(4.75, 63.40);
    FLYWHEEL_SPEED_MAP.put(5.00, 65.60);
    FLYWHEEL_SPEED_MAP.put(5.25, 67.90);
    FLYWHEEL_SPEED_MAP.put(5.50, 70.30);
    FLYWHEEL_SPEED_MAP.put(5.75, 72.80);
    FLYWHEEL_SPEED_MAP.put(6.00, 75.30);
    FLYWHEEL_SPEED_MAP.put(6.25, 77.80);
    FLYWHEEL_SPEED_MAP.put(6.50, 80.30);

    // Time of Flight [s] vs Distance [m] (Adjusted for slower projectile speeds)
    // High-arc trajectories: ToF ~1.36s plateau from 1.25m to 5.50m
    TIME_OF_FLIGHT_MAP.put(0.50, 0.7719);
    TIME_OF_FLIGHT_MAP.put(0.75, 1.0202);
    TIME_OF_FLIGHT_MAP.put(1.00, 1.2222);
    TIME_OF_FLIGHT_MAP.put(1.25, 1.3912);
    TIME_OF_FLIGHT_MAP.put(1.50, 1.3913);
    TIME_OF_FLIGHT_MAP.put(1.75, 1.3914);
    TIME_OF_FLIGHT_MAP.put(2.00, 1.3910);
    TIME_OF_FLIGHT_MAP.put(2.25, 1.3908);
    TIME_OF_FLIGHT_MAP.put(2.50, 1.3898);
    TIME_OF_FLIGHT_MAP.put(2.75, 1.3892);
    TIME_OF_FLIGHT_MAP.put(3.00, 1.4063);
    TIME_OF_FLIGHT_MAP.put(3.25, 1.4312);
    TIME_OF_FLIGHT_MAP.put(3.50, 1.4607);
    TIME_OF_FLIGHT_MAP.put(3.75, 1.4883);
    TIME_OF_FLIGHT_MAP.put(4.00, 1.5263);
    TIME_OF_FLIGHT_MAP.put(4.25, 1.5647);
    TIME_OF_FLIGHT_MAP.put(4.50, 1.6035);
    TIME_OF_FLIGHT_MAP.put(4.75, 1.6427);
    TIME_OF_FLIGHT_MAP.put(5.00, 1.6822);
    TIME_OF_FLIGHT_MAP.put(5.25, 1.7220);
    TIME_OF_FLIGHT_MAP.put(5.50, 1.7617);
    TIME_OF_FLIGHT_MAP.put(5.75, 1.8055);
    TIME_OF_FLIGHT_MAP.put(6.00, 1.8669);
    TIME_OF_FLIGHT_MAP.put(6.25, 1.9272);
    TIME_OF_FLIGHT_MAP.put(6.50, 1.9864);

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

    // Passing Maps (Calculated for 60-degree fixed hood)
    PASSING_HOOD_ANGLE_MAP.put(2.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(2.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(3.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(3.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(4.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(4.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(5.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(5.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(6.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(6.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(7.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(7.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(8.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(8.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(9.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(9.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(10.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(10.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(11.00, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(11.50, 1.047198);
    PASSING_HOOD_ANGLE_MAP.put(12.00, 1.047198);

    // Flywheel RPS (RK4 explicit drag simulation, slip=0.4993)
    PASSING_FLYWHEEL_SPEED_MAP.put(2.00, 28.6); // apex=1.27m
    PASSING_FLYWHEEL_SPEED_MAP.put(2.50, 32.6); // apex=1.50m
    PASSING_FLYWHEEL_SPEED_MAP.put(3.00, 36.2); // apex=1.72m
    PASSING_FLYWHEEL_SPEED_MAP.put(3.50, 39.6); // apex=1.95m
    PASSING_FLYWHEEL_SPEED_MAP.put(4.00, 42.8); // apex=2.19m
    PASSING_FLYWHEEL_SPEED_MAP.put(4.50, 45.9); // apex=2.42m
    PASSING_FLYWHEEL_SPEED_MAP.put(5.00, 48.8); // apex=2.66m
    PASSING_FLYWHEEL_SPEED_MAP.put(5.50, 51.6); // apex=2.90m
    PASSING_FLYWHEEL_SPEED_MAP.put(6.00, 54.4); // apex=3.14m
    PASSING_FLYWHEEL_SPEED_MAP.put(6.50, 57.1); // apex=3.39m
    PASSING_FLYWHEEL_SPEED_MAP.put(7.00, 59.7); // apex=3.63m
    PASSING_FLYWHEEL_SPEED_MAP.put(7.50, 62.3); // apex=3.89m
    PASSING_FLYWHEEL_SPEED_MAP.put(8.00, 64.8); // apex=4.14m
    PASSING_FLYWHEEL_SPEED_MAP.put(8.50, 67.4); // apex=4.39m
    PASSING_FLYWHEEL_SPEED_MAP.put(9.00, 69.8); // apex=4.65m
    PASSING_FLYWHEEL_SPEED_MAP.put(9.50, 72.3); // apex=4.91m
    PASSING_FLYWHEEL_SPEED_MAP.put(10.00, 74.7); // apex=5.18m
    PASSING_FLYWHEEL_SPEED_MAP.put(10.50, 77.2); // apex=5.44m
    PASSING_FLYWHEEL_SPEED_MAP.put(11.00, 79.6); // apex=5.71m
    PASSING_FLYWHEEL_SPEED_MAP.put(11.50, 82.0); // apex=5.98m
    PASSING_FLYWHEEL_SPEED_MAP.put(12.00, 84.4); // apex=6.26m

    // Passing ToF [s]
    // (If shoot-on-the-move compensation causes instability for huge target zones,
    // you can override these to return 0.0 to act as a fallback pure-feedforward)
    PASSING_TIME_OF_FLIGHT_MAP.put(2.00, 0.908);
    PASSING_TIME_OF_FLIGHT_MAP.put(2.50, 1.005);
    PASSING_TIME_OF_FLIGHT_MAP.put(3.00, 1.094);
    PASSING_TIME_OF_FLIGHT_MAP.put(3.50, 1.178);
    PASSING_TIME_OF_FLIGHT_MAP.put(4.00, 1.256);
    PASSING_TIME_OF_FLIGHT_MAP.put(4.50, 1.331);
    PASSING_TIME_OF_FLIGHT_MAP.put(5.00, 1.402);
    PASSING_TIME_OF_FLIGHT_MAP.put(5.50, 1.471);
    PASSING_TIME_OF_FLIGHT_MAP.put(6.00, 1.537);
    PASSING_TIME_OF_FLIGHT_MAP.put(6.50, 1.601);
    PASSING_TIME_OF_FLIGHT_MAP.put(7.00, 1.663);
    PASSING_TIME_OF_FLIGHT_MAP.put(7.50, 1.724);
    PASSING_TIME_OF_FLIGHT_MAP.put(8.00, 1.783);
    PASSING_TIME_OF_FLIGHT_MAP.put(8.50, 1.840);
    PASSING_TIME_OF_FLIGHT_MAP.put(9.00, 1.897);
    PASSING_TIME_OF_FLIGHT_MAP.put(9.50, 1.952);
    PASSING_TIME_OF_FLIGHT_MAP.put(10.00, 2.007);
    PASSING_TIME_OF_FLIGHT_MAP.put(10.50, 2.060);
    PASSING_TIME_OF_FLIGHT_MAP.put(11.00, 2.113);
    PASSING_TIME_OF_FLIGHT_MAP.put(11.50, 2.165);
    PASSING_TIME_OF_FLIGHT_MAP.put(12.00, 2.216);
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
    return PASSING_HOOD_ANGLE_MAP.get(distanceMeters) + sHoodOffsetRad;
  }

  public static double passingFlywheelSpeedRotPerSec(double distanceMeters) {
    return PASSING_FLYWHEEL_SPEED_MAP.get(distanceMeters) + sFlywheelOffsetRotPerSec;
  }

  public static double passingTimeOfFlightS(double distanceMeters) {
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
