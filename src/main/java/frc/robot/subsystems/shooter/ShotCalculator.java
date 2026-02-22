package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.PoseHistory;

/**
 * ShotCalculator - Zero-GC Shoot-On-Move Targeting System.
 *
 * <p>Computes turret angle, hood elevation, and flywheel speed required to hit a static target
 * while the robot is in motion. Uses simulation-generated lookup tables ({@link ShotTables})
 * following the Orbit 1690 + 6328 hybrid methodology.
 *
 * <p><strong>Performance</strong>: The hot path ({@link #calculate()}) performs zero heap
 * allocations. All intermediate geometry is computed with primitive {@code double} arithmetic.
 *
 * <p><strong>Algorithm (6328-style)</strong>:
 *
 * <ol>
 *   <li>Read estimated pose; shift forward by phase delay (linear extrapolation).
 *   <li>Calculate turret position and velocity in field frame (omega x r cross product).
 *   <li>Iteratively shift aim point by turret velocity * ToF until convergence.
 *   <li>Look up hood angle and flywheel speed at the converged "lookahead distance".
 *   <li>Compute setpoint derivatives for feedforward control.
 * </ol>
 *
 * <p><strong>Thread Safety</strong>: All public methods are synchronized. Call {@link
 * #clearCache()} at the start of each control cycle.
 *
 * @see ShotTables
 * @see Constants.Shot
 */
public class ShotCalculator {
  private static ShotCalculator mInstance;

  public static ShotCalculator getInstance() {
    if (mInstance == null) {
      mInstance = new ShotCalculator();
    }
    return mInstance;
  }

  // --- Velocity Filters ---
  private final LinearFilter mHoodAngleFilter;

  // --- State Tracking (primitives, no Rotation2d to avoid GC) ---
  private double mLastHoodAngleRad = Double.NaN;

  // --- Cached Output ---
  private final ShootingParameters mCachedParameters = new ShootingParameters();
  private boolean mIsCached = false;

  public static class ShootingParameters {
    public boolean isValid;
    public boolean isPassing;
    public double turretAngleRad;
    public double hoodAngleRad;
    public double hoodVelocityRadPerSec;
    public double flywheelSpeedRotPerSec;
    public double effectiveDistanceMeters;

    public void update(
        boolean isValid,
        boolean isPassing,
        double turretAngleRad,
        double hoodAngleRad,
        double hoodVelocityRadPerSec,
        double flywheelSpeedRotPerSec,
        double effectiveDistanceMeters) {
      this.isValid = isValid;
      this.isPassing = isPassing;
      this.turretAngleRad = turretAngleRad;
      this.hoodAngleRad = hoodAngleRad;
      this.hoodVelocityRadPerSec = hoodVelocityRadPerSec;
      this.flywheelSpeedRotPerSec = flywheelSpeedRotPerSec;
      this.effectiveDistanceMeters = effectiveDistanceMeters;
    }
  }

  private ShotCalculator() {
    int filterTaps = (int) (Constants.Shot.kVelocityFilterWindowSeconds / Constants.kLooperDt);
    mHoodAngleFilter = LinearFilter.movingAverage(filterTaps);
  }

  /** Convergence tolerance for the [LEGACY] iterative ToF solver [m]. */
  @SuppressWarnings("unused") // Referenced in legacy block comment; kept for fallback
  private static final double TOF_CONVERGENCE_M = 5e-4; // 0.5mm -- far below field accuracy

  /** Speed threshold below which we skip the ToF iteration (static robot). */
  private static final double STATIC_SPEED_THRESHOLD = 0.02; // m/s

  /**
   * Computes shooting parameters for the current robot state.
   *
   * <p><strong>Zero-GC</strong>: This method allocates no heap objects in the hot path. All
   * geometry is computed with primitive {@code double} values.
   *
   * <p>Memoized per cycle. Call {@link #clearCache()} at the start of each control cycle.
   *
   * @return The computed shooting parameters (check {@code isValid()} before using).
   */
  public synchronized ShootingParameters calculate() {
    if (mIsCached) {
      return mCachedParameters;
    }

    // Apply strict Point Shoot override
    if (frc.robot.DashboardState.getInstance().isPointShootDisabled()) {
      double minD = frc.robot.subsystems.shooter.ShotTables.MIN_DISTANCE_M;
      mCachedParameters.update(
          true, // isValid
          false, // isPassing
          0.0, // turretAngleRad (fixed straight ahead)
          frc.robot.subsystems.shooter.ShotTables.hoodAngleRad(minD),
          0.0, // hoodVelocity
          frc.robot.subsystems.shooter.ShotTables.flywheelSpeedRotPerSec(minD),
          minD);
      mIsCached = true;
      return mCachedParameters;
    }

    // ── Step 1: Read pose and velocity (only GC: WPILib Pose2d/ChassisSpeeds) ──
    // Use Fused Pose (Odometry + Vision) from RobotStateEstimator
    // Use Raw Velocity from PoseHistory (Drive) as EKF doesn't track velocity
    Pose2d rawPose;
    if (frc.robot.subsystems.RobotStateEstimator.hasInstance()) {
      rawPose = frc.robot.subsystems.RobotStateEstimator.getInstance().getEstimatedPose();
    } else {
      rawPose = PoseHistory.getInstance().getLatestFieldToVehicle();
    }
    ChassisSpeeds vel = PoseHistory.getInstance().getLatestFieldVelocity();

    // Extract to primitives -- all subsequent math is allocation-free
    double poseX = rawPose.getX();
    double poseY = rawPose.getY();
    double poseTheta = rawPose.getRotation().getRadians();
    double vx = vel.vxMetersPerSecond;
    double vy = vel.vyMetersPerSecond;
    double omega = vel.omegaRadiansPerSecond;

    // ── Step 2: Phase delay compensation (linear extrapolation) ──
    // For small dt (~30ms), linear extrapolation is equivalent to Twist2d.exp()
    // but avoids ~8 object allocations.
    double dt = Constants.Shot.kPhaseDelaySeconds;
    double estX = poseX + vx * dt;
    double estY = poseY + vy * dt;
    double estTheta = poseTheta + omega * dt;

    // ── Step 3: Turret position in field frame ──
    // turretField = robotPos + Rotate(robotAngle) * robotToTurret
    double rtX = Constants.Shot.kRobotToTurret.getX();
    double rtY = Constants.Shot.kRobotToTurret.getY();
    double cosTheta = Math.cos(estTheta);
    double sinTheta = Math.sin(estTheta);
    double turretX = estX + rtX * cosTheta - rtY * sinTheta;
    double turretY = estY + rtX * sinTheta + rtY * cosTheta;

    // ── Step 4: Turret velocity in field frame ──
    // v_turret = v_robot + omega x r_turret (2D cross product in field frame)
    // omega x r (body frame): (-omega*ry, omega*rx)
    // Rotated to field: R(theta) * (-omega*ry, omega*rx)
    // = (-omega*ry*cos - omega*rx*sin, -omega*ry*sin + omega*rx*cos)
    double turretVelX = vx + omega * (-rtY * cosTheta - rtX * sinTheta);
    double turretVelY = vy + omega * (-rtY * sinTheta + rtX * cosTheta);

    // ── Step 4.5: Target Selection & Bad Box Detection (Zero-GC) ──
    boolean isRed = frc.robot.util.geometry.AllianceFlipUtil.shouldFlip();
    double flippedX = isRed ? frc.robot.FieldConstants.fieldLength - estX : estX;
    double flippedY = isRed ? frc.robot.FieldConstants.fieldWidth - estY : estY;

    // Bad boxes (inline bounds for zero-allocation)
    boolean insideTower =
        flippedX >= 0.0 && flippedX <= 1.1684 && flippedY >= 3.2766 && flippedY <= 4.2672;
    boolean behindNearHub =
        flippedX >= frc.robot.FieldConstants.LinesVertical.neutralZoneNear
            && flippedX <= frc.robot.FieldConstants.LinesVertical.neutralZoneNear + 1.651
            && flippedY >= frc.robot.FieldConstants.LinesHorizontal.rightBumpStart
            && flippedY <= frc.robot.FieldConstants.LinesHorizontal.leftBumpEnd;
    boolean behindFarHub =
        flippedX >= frc.robot.FieldConstants.LinesVertical.oppAllianceZone
            && flippedX <= frc.robot.FieldConstants.fieldLength
            && flippedY >= frc.robot.FieldConstants.LinesHorizontal.rightBumpStart
            && flippedY <= frc.robot.FieldConstants.LinesHorizontal.leftBumpEnd;
    boolean inBadBox = insideTower || behindNearHub || behindFarHub;

    boolean isPassing = flippedX > frc.robot.FieldConstants.LinesVertical.hubCenter;

    // ── Step 5: Get target position ──
    double targetX;
    double targetY;

    if (isPassing) {
      // ── Zero-Allocation Passing Target Interpolator ──
      // Mimics
      // org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator.getPassingTarget()

      // Constants defined directly for zero-GC lookup
      double trackWidthY = Units.inchesToMeters(20.75); // from DriveConstants Swerve.kTrackWidth
      double rightBumpStart = frc.robot.FieldConstants.LinesHorizontal.rightBumpStart;
      double hubPassLine = rightBumpStart - (trackWidthY / 2.0);
      double xPassTarget = Units.inchesToMeters(25.0);
      double yPassTarget = Units.inchesToMeters(50.0);
      double minPassingDistance = ShotTables.MIN_PASSING_DISTANCE_M;

      boolean mirror = flippedY > frc.robot.FieldConstants.LinesHorizontal.center;
      double unflippedPassY;

      if (frc.robot.FieldConstants.fieldWidth - hubPassLine > flippedY && flippedY > hubPassLine) {
        // Linear interpolation without MathUtil object wrappers
        double evaluateY = mirror ? frc.robot.FieldConstants.fieldWidth - flippedY : flippedY;
        double amount =
            (evaluateY - hubPassLine)
                / (frc.robot.FieldConstants.LinesHorizontal.center - hubPassLine);

        // Clamp amount between 0.0 and 1.0 (equivalent to MathUtil.clamp)
        amount = Math.max(0.0, Math.min(1.0, amount));

        // Interpolate yPassTarget to minPassingDistance
        double interpolatedTargetY = yPassTarget + (minPassingDistance - yPassTarget) * amount;

        unflippedPassY =
            mirror
                ? frc.robot.FieldConstants.fieldWidth - interpolatedTargetY
                : interpolatedTargetY;
      } else {
        // Fixed passing target
        unflippedPassY = mirror ? frc.robot.FieldConstants.fieldWidth - yPassTarget : yPassTarget;
      }

      targetX = isRed ? frc.robot.FieldConstants.fieldLength - xPassTarget : xPassTarget;
      targetY = isRed ? frc.robot.FieldConstants.fieldWidth - unflippedPassY : unflippedPassY;
    } else {
      targetX = Constants.getHubCenter().getX();
      targetY = Constants.getHubCenter().getY();
    }

    // ── Step 6 & 7: 6328-Style Iterative Lookahead Solver ──
    // Note: Phase delay has already been applied in Step 2 to compute turretX/Y.
    double dx = targetX - turretX;
    double dy = targetY - turretY;
    double initialDist = Math.hypot(dx, dy);

    double lookaheadX = turretX;
    double lookaheadY = turretY;
    double lookaheadDist = initialDist;
    double turretAngleRad = Math.atan2(dy, dx); // Fallback estimate

    // Fast path check
    double turretSpeed = Math.abs(turretVelX) + Math.abs(turretVelY);
    if (turretSpeed > STATIC_SPEED_THRESHOLD) {
      // Due to aerodynamic drag, the ball does not inherit 100% of the robot's
      // velocity indefinitely.
      // To mimic 6328's 100% perfect inheritance, set kInertiaInheritance = 1.0 in
      // Constants.
      double effVelX = turretVelX * Constants.Shot.kInertiaInheritance;
      double effVelY = turretVelY * Constants.Shot.kInertiaInheritance;

      // 6328-style Iterative Solver (with dynamic convergence break)
      double previousLookaheadDist = initialDist;

      for (int i = 0; i < Constants.Shot.kMaxTofIterations; i++) {
        double tof =
            isPassing
                ? ShotTables.passingTimeOfFlightS(lookaheadDist)
                : ShotTables.timeOfFlightS(lookaheadDist);
        double offsetX = effVelX * tof;
        double offsetY = effVelY * tof;

        // Virtual lookahead position of the turret
        lookaheadX = turretX + offsetX;
        lookaheadY = turretY + offsetY;

        // Update distance from the LOOKAHEAD position to target
        lookaheadDist = Math.hypot(targetX - lookaheadX, targetY - lookaheadY);

        // Dynamic Break: Banach Fixed-Point early convergence (< 1mm tolerance)
        if (Math.abs(lookaheadDist - previousLookaheadDist) < 0.001) {
          break;
        }
        previousLookaheadDist = lookaheadDist;
      }

      // Final aim angle
      dx = targetX - lookaheadX;
      dy = targetY - lookaheadY;
      turretAngleRad = Math.atan2(dy, dx);
    }

    // ── [Fix] True Vector Separation Control: Radial Effective Distance ──
    // Calculate the projection of the robot's velocity purely along the axis to the
    // target.
    // dxInitial = target - turret, so positive dot product means moving TOWARDS
    // target.
    double dxInitial = targetX - turretX;
    double dyInitial = targetY - turretY;
    double radialVelocity = (dxInitial * turretVelX + dyInitial * turretVelY) / initialDist;

    // Retrieve a base Time of Flight for the interpolation using the raw physical
    // distance
    double baseToF =
        isPassing
            ? ShotTables.passingTimeOfFlightS(initialDist)
            : ShotTables.timeOfFlightS(initialDist);

    // Apply the aerodynamic inheritance decay factor to the radial velocity
    // component
    double effectiveRadialVel = radialVelocity * Constants.Shot.kInertiaInheritance;

    // CRITICAL FIX: Since positive radialVel means moving TOWARDS the target,
    // the ball inherits forward energy. Therefore, the shooter needs to do LESS
    // work,
    // meaning we look up a SHORTER effective distance.
    double effectiveLookupDist = initialDist - (effectiveRadialVel * baseToF);

    // Clamp to physical table boundaries so extreme high-speed retreats don't crash
    // the splines
    effectiveLookupDist =
        Math.max(
            ShotTables.MIN_DISTANCE_M, Math.min(effectiveLookupDist, ShotTables.MAX_DISTANCE_M));

    // ── Step 8: Look up launch parameters (spline-based) ──
    // Use the *effective radial distance* rather than the hypotenuse lookaheadDist
    double hoodAngleRad =
        isPassing
            ? ShotTables.passingHoodAngleRad(effectiveLookupDist)
            : ShotTables.hoodAngleRad(effectiveLookupDist);
    double flywheelSpeedRotPerSec =
        isPassing
            ? ShotTables.passingFlywheelSpeedRotPerSec(effectiveLookupDist)
            : ShotTables.flywheelSpeedRotPerSec(effectiveLookupDist);

    // Clamp to physical limits (e.g. 0 to 80 RPS)
    flywheelSpeedRotPerSec = Math.max(0.0, Math.min(flywheelSpeedRotPerSec, 80.0));

    // ── Step 9: Feedforward derivatives ──
    // Numerical differentiation: (current - previous) / dt, filtered
    double hoodVelocity;

    if (Double.isNaN(mLastHoodAngleRad)) {
      // First call: no derivative available
      hoodVelocity = 0.0;
    } else {
      double rawHoodVelocity = (hoodAngleRad - mLastHoodAngleRad) / Constants.kLooperDt;
      hoodVelocity = mHoodAngleFilter.calculate(rawHoodVelocity);
    }
    mLastHoodAngleRad = hoodAngleRad;

    // ── Build result ──
    boolean isValid =
        !inBadBox
            && (isPassing
                ? (initialDist >= 0.0 && initialDist <= 12.0)
                : (initialDist >= ShotTables.MIN_DISTANCE_M
                    && initialDist <= ShotTables.MAX_DISTANCE_M));

    // edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Shooter/IsPassing",
    // isPassing);
    // edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Shooter/InBadBox",
    // inBadBox);

    // The only object mutation, we pre-allocate and only update to avoid GC
    mCachedParameters.update(
        isValid,
        isPassing,
        turretAngleRad,
        hoodAngleRad,
        hoodVelocity,
        flywheelSpeedRotPerSec,
        lookaheadDist);

    mIsCached = true;
    return mCachedParameters;
  }

  /**
   * Clears cached parameters, forcing recalculation on next call.
   *
   * <p>Call at the start of each control cycle.
   */
  public synchronized void clearCache() {
    mIsCached = false;
  }

  /** Returns cached parameters without recalculating. May return null. */
  public synchronized ShootingParameters getCachedParameters() {
    return mIsCached ? mCachedParameters : null;
  }
}
