package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.GlobalData;
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
  private final LinearFilter mTurretVelocityFilter;

  // --- State Tracking (primitives, no Rotation2d to avoid GC) ---
  private double mLastHoodAngleRad = Double.NaN;
  private double mLastTurretAngleRad = Double.NaN;

  // --- Cached Output ---
  private final ShootingParameters mCachedParameters = new ShootingParameters();
  private boolean mIsCached = false;
  private TargetType mCachedTargetType = TargetType.HUB;
  private TargetType mLastTargetType = TargetType.HUB;

  /** Target selection for calculation. */
  public enum TargetType {
    /** Main goal (Hub/Speaker). */
    HUB,
    /** Passing target (Human Player / Amp zone). */
    PASS
  }

  public static class ShootingParameters {
    public boolean isValid;
    public boolean hasTarget;
    public boolean isPassing;
    public double turretAngleRad;
    public double turretVelocityRadPerSec;
    public double hoodAngleRad;
    public double hoodVelocityRadPerSec;
    public double flywheelSpeedRotPerSec;
    public double effectiveDistanceMeters;
    public double descentAngleRads;

    public void update(
        boolean isValid,
        boolean hasTarget,
        boolean isPassing,
        double turretAngleRad,
        double turretVelocityRadPerSec,
        double hoodAngleRad,
        double hoodVelocityRadPerSec,
        double flywheelSpeedRotPerSec,
        double effectiveDistanceMeters,
        double descentAngleRads) {
      this.isValid = isValid;
      this.hasTarget = hasTarget;
      this.isPassing = isPassing;
      this.turretAngleRad = turretAngleRad;
      this.turretVelocityRadPerSec = turretVelocityRadPerSec;
      this.hoodAngleRad = hoodAngleRad;
      this.hoodVelocityRadPerSec = hoodVelocityRadPerSec;
      this.flywheelSpeedRotPerSec = flywheelSpeedRotPerSec;
      this.effectiveDistanceMeters = effectiveDistanceMeters;
      this.descentAngleRads = descentAngleRads;
    }
  }

  private ShotCalculator() {
    int filterTaps = (int) (Constants.Shot.kVelocityFilterWindowSeconds / Constants.kLooperDt);
    mHoodAngleFilter = LinearFilter.movingAverage(filterTaps);
    mTurretVelocityFilter = LinearFilter.movingAverage(filterTaps);
  }

  /** Convergence tolerance for the [LEGACY] iterative ToF solver [m]. */
  @SuppressWarnings("unused") // Referenced in legacy block comment; kept for fallback
  private static final double TOF_CONVERGENCE_M = 5e-4; // 0.5mm -- far below field accuracy

  /** Speed threshold below which we skip the ToF iteration (static robot). */
  private static final double STATIC_SPEED_THRESHOLD = 0.02; // m/s

  /**
   * Computes shooting parameters for the current robot state.
   *
   * @return The computed shooting parameters for the default target (HUB).
   * @see #calculate(TargetType)
   */
  public ShootingParameters calculate() {
    // 💡 Priority 1: Use actual robot state if user is holding a button
    if (GlobalData.robotState == frc.robot.RobotState.PASS) {
      mLastTargetType = TargetType.PASS;
      return calculate(TargetType.PASS);
    }
    if (GlobalData.robotState == frc.robot.RobotState.SCORE) {
      mLastTargetType = TargetType.HUB;
      return calculate(TargetType.HUB);
    }

    // 💡 Priority 2: Determine default target type based on field position (For
    // pre-aiming)
    Pose2d rawPose;
    if (frc.robot.subsystems.RobotStateEstimator.hasInstance()) {
      rawPose = frc.robot.subsystems.RobotStateEstimator.getInstance().getEstimatedPose();
    } else {
      rawPose = PoseHistory.getInstance().getLatestFieldToVehicle();
    }

    // Use turret field position (not robot center) for hub line check.
    // The ball exits from the turret, so the turret's X coordinate determines
    // whether we are in the scoring zone or the passing zone.
    double poseTheta = rawPose.getRotation().getRadians();
    double rtX = Constants.Shot.kRobotToTurret.getX();
    double rtY = Constants.Shot.kRobotToTurret.getY();
    double turretFieldX = rawPose.getX() + rtX * Math.cos(poseTheta) - rtY * Math.sin(poseTheta);

    boolean isRed = frc.robot.util.geometry.AllianceFlipUtil.shouldFlip();
    double flippedX = isRed ? frc.robot.FieldConstants.fieldLength - turretFieldX : turretFieldX;

    // 💡 [Aerospace Rigor] Hysteresis to prevent target flickering near the Hub
    // Use the Hub center line for the switch point (MA 6328 Strategy)
    double hubLine = frc.robot.FieldConstants.LinesVertical.hubCenter;
    double buffer = 0.25; // 25cm hysteresis zone

    TargetType target;
    if (mLastTargetType == TargetType.HUB) {
      target = (flippedX > hubLine + buffer) ? TargetType.PASS : TargetType.HUB;
    } else {
      target = (flippedX < hubLine - buffer) ? TargetType.HUB : TargetType.PASS;
    }
    mLastTargetType = target;

    return calculate(target);
  }

  /**
   * Computes shooting parameters for the specified target.
   *
   * <p><strong>Zero-GC</strong>: This method allocates no heap objects in the hot path. All
   * geometry is computed with primitive {@code double} values.
   *
   * <p>Memoized per target type per cycle.
   *
   * @param targetType The target to aim for.
   * @return The computed shooting parameters.
   */
  public synchronized ShootingParameters calculate(TargetType targetType) {
    if (mIsCached && mCachedTargetType == targetType) {
      return mCachedParameters;
    }
    mCachedTargetType = targetType;

    // Apply strict Point Shoot override
    if (frc.robot.DashboardState.getInstance().isPointShootDisabled()) {
      double minD = frc.robot.subsystems.shooter.ShotTables.MIN_DISTANCE_M;
      double tof = frc.robot.subsystems.shooter.ShotTables.timeOfFlightS(minD);
      double deltaH = 1.83 - frc.robot.Constants.Shot.kShooterHeightMeters;
      double vzf = (deltaH / tof) - 0.5 * 9.81 * tof;
      double vxy = minD / tof;
      double descentAngleRads = Math.atan2(Math.abs(vzf), vxy);

      mCachedParameters.update(
          true, // isValid
          true, // hasTarget
          false, // isPassing
          0.0, // turretAngleRad (fixed straight ahead)
          0.0, // turretVelocityRadPerSec
          frc.robot.subsystems.shooter.ShotTables.hoodAngleRad(minD),
          0.0, // hoodVelocity
          frc.robot.subsystems.shooter.ShotTables.flywheelSpeedRotPerSec(minD),
          minD,
          descentAngleRads);
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

    // Use pose rotation (which is now strictly synced to gyro in
    // RobotStateEstimator)
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

    boolean isPassing = (targetType == TargetType.PASS);

    // ── Step 5: Get target position ──
    double targetX;
    double targetY;

    if (isPassing) {
      // ── Zero-Allocation Passing Target Interpolator ──
      // Mimics
      // org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator.getPassingTarget()

      // Constants defined directly for zero-GC lookup
      double trackWidthY =
          edu.wpi.first.math.util.Units.inchesToMeters(20.75); // from DriveConstants
      // Swerve.kTrackWidth
      double rightBumpStart = frc.robot.FieldConstants.LinesHorizontal.rightBumpStart;
      double hubPassLine = rightBumpStart - (trackWidthY / 2.0);

      double xPassTarget = 2.108;
      double yPassTarget = 2.168;
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
        double targetOffsetX = effVelX * tof;
        double targetOffsetY = effVelY * tof;

        // 💡 [Aerospace Rigor] Numerical Damping for Stability
        // Long-range passing shots (TOF > 2s) create a positive feedback loop:
        // higher dist -> higher TOF -> higher offset -> higher dist.
        // We damp the update by 50% to ensure convergence (Banach Fixed-Point).
        double damping = isPassing ? 0.5 : 1.0;
        lookaheadX = lookaheadX + (turretX + targetOffsetX - lookaheadX) * damping;
        lookaheadY = lookaheadY + (turretY + targetOffsetY - lookaheadY) * damping;

        // Update distance from the DAMPED LOOKAHEAD position to target
        lookaheadDist = Math.hypot(targetX - lookaheadX, targetY - lookaheadY);

        // Dynamic Break: Convergence check (< 1mm tolerance)
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

    // ── [Fix] True 3D Lookahead Distance for Kinematics Compensation ──
    // Aerodynamic calculations necessitate the true hypotenuse lookahead distance
    // to account for the lateral travel in the 3D parabolic trajectory.
    // Passing table domain [2.0, 12.0]m differs from HUB [0.5, 6.5]m —
    // using the wrong range causes parameter jumps during pre-aim target switching.
    double clampDist =
        isPassing
            ? Math.max(
                ShotTables.MIN_PASSING_DISTANCE_M,
                Math.min(lookaheadDist, ShotTables.MAX_PASSING_DISTANCE_M))
            : Math.max(
                ShotTables.MIN_DISTANCE_M, Math.min(lookaheadDist, ShotTables.MAX_DISTANCE_M));

    // ── Step 8: Look up launch parameters (spline-based) ──
    // Use the full 3D lookahead distance
    double hoodAngleRad =
        isPassing ? ShotTables.passingHoodAngleRad(clampDist) : ShotTables.hoodAngleRad(clampDist);
    double flywheelSpeedRotPerSec =
        isPassing
            ? ShotTables.passingFlywheelSpeedRotPerSec(clampDist)
            : ShotTables.flywheelSpeedRotPerSec(clampDist);

    // Clamp to physical limits (e.g. 0 to 80 RPS)
    flywheelSpeedRotPerSec = Math.max(0.0, Math.min(flywheelSpeedRotPerSec, 80.0));

    // ── Step 9: Feedforward derivatives ──
    // Numerical differentiation: (current - previous) / dt, filtered
    double hoodVelocity;
    double turretVelocityRadPerSec;

    if (Double.isNaN(mLastHoodAngleRad) || Double.isNaN(mLastTurretAngleRad)) {
      // First call: no derivative available
      hoodVelocity = 0.0;
      turretVelocityRadPerSec = 0.0;
    } else {
      double rawHoodVelocity = (hoodAngleRad - mLastHoodAngleRad) / Constants.kLooperDt;
      hoodVelocity = mHoodAngleFilter.calculate(rawHoodVelocity);

      double rawTurretVelocity =
          edu.wpi.first.math.MathUtil.angleModulus(turretAngleRad - mLastTurretAngleRad)
              / Constants.kLooperDt;
      turretVelocityRadPerSec = mTurretVelocityFilter.calculate(rawTurretVelocity);
    }
    mLastHoodAngleRad = hoodAngleRad;
    mLastTurretAngleRad = turretAngleRad;

    // ── Build result ──
    boolean hasTarget = true; // Coordinate-based targets are always pointable
    // Passing has no distance limit — clamp is handled at the lookup layer.
    // HUB shots require distance within the calibrated table range.
    boolean isValid =
        !inBadBox
            && (isPassing
                || (initialDist >= ShotTables.MIN_DISTANCE_M
                    && initialDist <= ShotTables.MAX_DISTANCE_M));

    // ── [Fix] Physical Trajectory Terminal Descent Angle ──
    double physicalTof =
        isPassing
            ? ShotTables.passingTimeOfFlightS(clampDist)
            : ShotTables.timeOfFlightS(clampDist);
    double deltaH = isPassing ? 0.0 : (1.83 - frc.robot.Constants.Shot.kShooterHeightMeters);
    double vzf = (physicalTof > 1e-6) ? ((deltaH / physicalTof) - 0.5 * 9.81 * physicalTof) : 0.0;
    double vxy = (physicalTof > 1e-6) ? (clampDist / physicalTof) : 0.0;
    double descentAngleRads = (vxy > 1e-6) ? Math.atan2(Math.abs(vzf), vxy) : Math.PI / 2.0;

    // The only object mutation, we pre-allocate and only update to avoid GC
    mCachedParameters.update(
        isValid,
        hasTarget,
        isPassing,
        turretAngleRad,
        turretVelocityRadPerSec,
        hoodAngleRad,
        hoodVelocity,
        flywheelSpeedRotPerSec,
        lookaheadDist,
        descentAngleRads);

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
