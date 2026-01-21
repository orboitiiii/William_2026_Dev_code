package frc.robot.libraries.lib9427;

/**
 * High-level wrapper for the native C++ ballistic solver.
 *
 * <p>This class provides an object-oriented interface to the first-principles ballistic solver,
 * which computes optimal turret yaw, pitch, and launch velocity for scoring projectiles into the
 * hexagonal target.
 *
 * <h2>Physical Model</h2>
 *
 * The solver accounts for:
 *
 * <ul>
 *   <li>Gravitational acceleration (9.80665 m/s²)
 *   <li>Aerodynamic drag (quadratic velocity dependence)
 *   <li>Magnus effect (spin-induced lift)
 *   <li>Robot translational and rotational motion
 *   <li>Turret slew delay compensation
 * </ul>
 *
 * <h2>Usage Example</h2>
 *
 * <pre>{@code
 * try (BallisticSolver solver = new BallisticSolver()) {
 *     solver.setShooterConstraints(0, 4000, 0.0508, 0.1, 1.2, 6.0);
 *     solver.setTarget(0.0, 0.0, 1.8288);
 *
 *     ShootingSolution sol = solver.solve(
 *         5.0, 2.0,       // robot x, y
 *         1.0, 0.5,       // robot vx, vy
 *         0.1, 0.0,       // omega, heading
 *         0.5, 0.0, 0.0,  // turret height, yaw, yaw rate
 *         0.0, 0.0);      // turret offset x, y
 *
 *     if (sol.valid) {
 *         turret.setYaw(sol.yawRad);
 *         shooter.setPitch(sol.pitchRad);
 *         shooter.setRpm(sol.rpm);
 *     }
 * }
 * }</pre>
 *
 * <h2>Thread Safety</h2>
 *
 * This class is NOT thread-safe. Each thread should use its own instance.
 *
 * @author Team 9427 William
 */
public class BallisticSolver implements AutoCloseable {

  /** Native handle to the C++ BallisticSolver instance. */
  private long nativeHandle;

  // Pre-allocated JNI buffer to avoid heap allocation (Zero-GC)
  private final double[] m_jniBuffer = new double[11];

  // Cached solution object to avoid object allocation (Zero-GC)
  private final ShootingSolution m_solutionCache = new ShootingSolution(m_jniBuffer);

  /** Default projectile mass [kg]. */
  public static final double DEFAULT_MASS_KG = 0.215;

  /** Default projectile diameter [m]. */
  public static final double DEFAULT_DIAMETER_M = 0.150;

  /** Default drag coefficient. */
  public static final double DEFAULT_DRAG_COEFFICIENT = 0.485;

  /** Default Magnus coefficient. */
  public static final double DEFAULT_MAGNUS_COEFFICIENT = 0.50;

  /** Default moment of inertia [kg·m²]. */
  public static final double DEFAULT_MOMENT_OF_INERTIA = 0.0024;

  /** Default spin decay coefficient. */
  public static final double DEFAULT_SPIN_DECAY = 0.001;

  /** Target height: 72 inches in meters. */
  public static final double TARGET_HEIGHT_M = 72.0 * 0.0254;

  /** Shooting solution container. */
  public static class ShootingSolution {
    /** True if a valid solution was found. */
    public boolean valid;

    /** Turret yaw angle (field-absolute) [rad]. */
    public double yawRad;

    /** Launch pitch angle [rad]. */
    public double pitchRad;

    /** Launch exit velocity [m/s]. */
    public double velocityMps;

    /** Corresponding shooter RPM. */
    public double rpm;

    /** Predicted time of flight [s]. */
    public double timeOfFlightS;

    /** Predicted impact X [m]. */
    public double impactXM;

    /** Predicted impact Y [m]. */
    public double impactYM;

    /** Predicted impact Z [m]. */
    public double impactZM;

    /** Yaw lead angle added for motion compensation [rad]. */
    public double yawLeadRad;

    /** Time for turret to reach target yaw [s]. */
    public double turretTravelTimeS;

    /**
     * Constructs a solution from raw native array.
     *
     * @param data Native result array (11 elements).
     */
    ShootingSolution(double[] data) {
      updateFromBuffer(data);
    }

    /**
     * Updates the solution from the buffer without allocating new memory.
     *
     * @param data Native result array (11 elements).
     */
    public void updateFromBuffer(double[] data) {
      if (data == null || data.length < 11) {
        this.valid = false;
        this.yawRad = 0;
        this.pitchRad = 0;
        this.velocityMps = 0;
        this.rpm = 0;
        this.timeOfFlightS = 0;
        this.impactXM = 0;
        this.impactYM = 0;
        this.impactZM = 0;
        this.yawLeadRad = 0;
        this.turretTravelTimeS = 0;
      } else {
        this.valid = data[0] > 0.5;
        this.yawRad = data[1];
        this.pitchRad = data[2];
        this.velocityMps = data[3];
        this.rpm = data[4];
        this.timeOfFlightS = data[5];
        this.impactXM = data[6];
        this.impactYM = data[7];
        this.impactZM = data[8];
        this.yawLeadRad = data[9];
        this.turretTravelTimeS = data[10];
      }
    }

    /** Returns an invalid solution marker. */
    public static ShootingSolution invalid() {
      return new ShootingSolution(null);
    }

    @Override
    public String toString() {
      if (!valid) {
        return "ShootingSolution{INVALID}";
      }
      return String.format(
          "ShootingSolution{yaw=%.2f°, pitch=%.2f°, rpm=%.0f, ToF=%.3fs}",
          Math.toDegrees(yawRad), Math.toDegrees(pitchRad), rpm, timeOfFlightS);
    }
  }

  /** Trajectory point for visualization. */
  public static class TrajectoryPoint {
    /** Time since launch [s]. */
    public final double t;

    /** X position [m]. */
    public final double x;

    /** Y position [m]. */
    public final double y;

    /** Z position [m]. */
    public final double z;

    /** X velocity [m/s]. */
    public final double vx;

    /** Y velocity [m/s]. */
    public final double vy;

    /** Z velocity [m/s]. */
    public final double vz;

    TrajectoryPoint(double t, double x, double y, double z, double vx, double vy, double vz) {
      this.t = t;
      this.x = x;
      this.y = y;
      this.z = z;
      this.vx = vx;
      this.vy = vy;
      this.vz = vz;
    }
  }

  /** Creates a new BallisticSolver with default parameters. */
  public BallisticSolver() {
    nativeHandle = Team9427JNI.createBallisticSolver();
    if (nativeHandle == 0) {
      throw new RuntimeException("Failed to create native BallisticSolver");
    }
    // Apply defaults
    configureProjectile(
        DEFAULT_MASS_KG,
        DEFAULT_DIAMETER_M,
        DEFAULT_DRAG_COEFFICIENT,
        DEFAULT_MAGNUS_COEFFICIENT,
        DEFAULT_MOMENT_OF_INERTIA,
        DEFAULT_SPIN_DECAY);
  }

  /**
   * Configures projectile physical parameters.
   *
   * @param massKg Mass [kg].
   * @param diameterM Diameter [m].
   * @param cd Drag coefficient.
   * @param cm Magnus coefficient.
   * @param moi Moment of inertia [kg·m²].
   * @param spinDecay Spin decay coefficient.
   */
  public void configureProjectile(
      double massKg, double diameterM, double cd, double cm, double moi, double spinDecay) {
    ensureValid();
    Team9427JNI.configureBallisticSolver(nativeHandle, massKg, diameterM, cd, cm, moi, spinDecay);
  }

  /**
   * Sets shooter mechanism constraints.
   *
   * @param minRpm Minimum RPM.
   * @param maxRpm Maximum RPM.
   * @param wheelRadiusM Shooter wheel radius [m].
   * @param pitchMinRad Minimum pitch [rad].
   * @param pitchMaxRad Maximum pitch [rad].
   * @param turretMaxRateRadps Maximum turret slew rate [rad/s].
   */
  public void setShooterConstraints(
      double minRpm,
      double maxRpm,
      double wheelRadiusM,
      double pitchMinRad,
      double pitchMaxRad,
      double turretMaxRateRadps) {
    ensureValid();
    Team9427JNI.setBallisticShooterConstraints(
        nativeHandle, minRpm, maxRpm, wheelRadiusM, pitchMinRad, pitchMaxRad, turretMaxRateRadps);
  }

  /**
   * Sets target specification.
   *
   * @param centerXM Target center X [m].
   * @param centerYM Target center Y [m].
   * @param centerZM Target center height [m].
   */
  public void setTarget(double centerXM, double centerYM, double centerZM) {
    ensureValid();
    Team9427JNI.setBallisticTarget(nativeHandle, centerXM, centerYM, centerZM);
  }

  /**
   * Sets integration time step.
   *
   * @param dtS Time step [s]. Must be <= 0.01s for accuracy.
   */
  public void setTimeStep(double dtS) {
    ensureValid();
    Team9427JNI.setBallisticTimeStep(nativeHandle, dtS);
  }

  /**
   * Sets projectile spin rate.
   *
   * @param spinRpm Backspin rate [RPM]. Positive = backspin.
   */
  public void setSpinRate(double spinRpm) {
    ensureValid();
    Team9427JNI.setBallisticSpinRate(nativeHandle, spinRpm);
  }

  /**
   * Solves for optimal shooting parameters.
   *
   * @param robotXM Robot X position [m].
   * @param robotYM Robot Y position [m].
   * @param robotVxMps Robot X velocity [m/s].
   * @param robotVyMps Robot Y velocity [m/s].
   * @param robotOmegaRadps Robot angular velocity [rad/s].
   * @param robotHeadingRad Robot heading [rad].
   * @param turretHeightM Turret pivot height [m].
   * @param turretYawRad Current turret yaw relative to chassis [rad].
   * @param turretYawRateRadps Turret yaw velocity [rad/s].
   * @param turretOffsetXM Turret X offset from robot center [m].
   * @param turretOffsetYM Turret Y offset from robot center [m].
   * @return Computed shooting solution (check .valid before using).
   */
  public ShootingSolution solve(
      double robotXM,
      double robotYM,
      double robotVxMps,
      double robotVyMps,
      double robotOmegaRadps,
      double robotHeadingRad,
      double turretHeightM,
      double turretYawRad,
      double turretYawRateRadps,
      double turretOffsetXM,
      double turretOffsetYM) {
    ensureValid();

    // Call JNI with pre-allocated buffer
    Team9427JNI.solveBallisticSolution(
        nativeHandle,
        robotXM,
        robotYM,
        robotVxMps,
        robotVyMps,
        robotOmegaRadps,
        robotHeadingRad,
        turretHeightM,
        turretYawRad,
        turretYawRateRadps,
        turretOffsetXM,
        turretOffsetYM,
        m_jniBuffer); // Pass buffer

    // Update and return cached object
    m_solutionCache.updateFromBuffer(m_jniBuffer);
    return m_solutionCache;
  }

  /**
   * Simulates trajectory for given shooting parameters.
   *
   * @param yawRad Launch yaw [rad].
   * @param pitchRad Launch pitch [rad].
   * @param velocityMps Exit velocity [m/s].
   * @param robotXM Robot X position [m].
   * @param robotYM Robot Y position [m].
   * @param robotVxMps Robot X velocity [m/s].
   * @param robotVyMps Robot Y velocity [m/s].
   * @param robotOmegaRadps Robot angular velocity [rad/s].
   * @param robotHeadingRad Robot heading [rad].
   * @param turretHeightM Turret height [m].
   * @param turretOffsetXM Turret X offset [m].
   * @param turretOffsetYM Turret Y offset [m].
   * @param maxPoints Maximum trajectory points.
   * @return Array of trajectory points.
   */
  public TrajectoryPoint[] simulateTrajectory(
      double yawRad,
      double pitchRad,
      double velocityMps,
      double robotXM,
      double robotYM,
      double robotVxMps,
      double robotVyMps,
      double robotOmegaRadps,
      double robotHeadingRad,
      double turretHeightM,
      double turretOffsetXM,
      double turretOffsetYM,
      int maxPoints) {
    ensureValid();
    double[] raw =
        Team9427JNI.simulateBallisticTrajectory(
            nativeHandle,
            yawRad,
            pitchRad,
            velocityMps,
            robotXM,
            robotYM,
            robotVxMps,
            robotVyMps,
            robotOmegaRadps,
            robotHeadingRad,
            turretHeightM,
            turretOffsetXM,
            turretOffsetYM,
            maxPoints);

    if (raw == null || raw.length < 7) {
      return new TrajectoryPoint[0];
    }

    int numPoints = raw.length / 7;
    TrajectoryPoint[] trajectory = new TrajectoryPoint[numPoints];
    for (int i = 0; i < numPoints; i++) {
      trajectory[i] =
          new TrajectoryPoint(
              raw[i * 7],
              raw[i * 7 + 1],
              raw[i * 7 + 2],
              raw[i * 7 + 3],
              raw[i * 7 + 4],
              raw[i * 7 + 5],
              raw[i * 7 + 6]);
    }
    return trajectory;
  }

  /**
   * Checks if a point is inside the hexagon target.
   *
   * @param xInch X coordinate relative to hexagon center [inches].
   * @param yInch Y coordinate relative to hexagon center [inches].
   * @return True if inside hexagon.
   */
  public static boolean isInsideHexagon(double xInch, double yInch) {
    return Team9427JNI.isInsideHexagon(xInch, yInch);
  }

  /** Releases native resources. */
  @Override
  public void close() {
    if (nativeHandle != 0) {
      Team9427JNI.deleteBallisticSolver(nativeHandle);
      nativeHandle = 0;
    }
  }

  private void ensureValid() {
    if (nativeHandle == 0) {
      throw new IllegalStateException("BallisticSolver has been closed");
    }
  }
}
