package frc.robot.lib.math;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Pilot Drive Algorithm Implementation
 *
 * Based on Team 1690's approach: The robot drives on a dynamic circular arc
 * with a (nearly) constant radius towards the target. This solves the issues
 * with traditional Bezier path following while providing smooth, predictable motion.
 *
 * Key concepts:
 * - Dynamic arc calculation based on current position, previous position, and target
 * - Angle bisector with heuristic adjustments
 * - Alpha parameter for different motion characteristics
 */
public class PilotDriveController {

    // ============= CONFIGURATION PARAMETERS =============

    /**
     * Pilot Drive configuration parameters
     */
    public static class PilotConfig {
        // Arc calculation parameters
        public static final double MIN_ARC_RADIUS = 0.3; // meters
        public static final double MAX_ARC_RADIUS = 5.0; // meters
        public static final double DEFAULT_ARC_RADIUS = 1.5; // meters

        // Alpha heuristic parameters (0-1 range)
        public static final double ALPHA_AGGRESSIVE = 0.3;  // Sharp turns, direct approach
        public static final double ALPHA_BALANCED = 0.5;    // Balanced approach
        public static final double ALPHA_SMOOTH = 0.7;      // Smooth, wide turns

        // Dynamic alpha adjustment factors
        public static final double ALPHA_DISTANCE_FACTOR = 0.2;  // How much distance affects alpha
        public static final double ALPHA_ANGLE_FACTOR = 0.3;     // How much angle difference affects alpha

        // Convergence parameters
        public static final double CONVERGENCE_DISTANCE = 1.0; // meters - when to start converging
        public static final double DIRECT_APPROACH_DISTANCE = 0.5; // meters - when to go direct
        public static final double ARRIVAL_TOLERANCE = 0.05; // meters

        // Velocity constraints
        public static final double MAX_VELOCITY = 4.0; // m/s
        public static final double MIN_VELOCITY = 0.5; // m/s
        public static final double CURVATURE_VELOCITY_FACTOR = 2.0; // v = factor / curvature

        // Smoothing parameters
        public static final double HEADING_SMOOTHING = 0.85; // Exponential smoothing factor
        public static final double VELOCITY_SMOOTHING = 0.9;

        // Safety parameters
        public static final double MAX_HEADING_CHANGE_RATE = Math.toRadians(180); // rad/s
        public static final double STALL_DETECTION_TIME = 0.5; // seconds
        public static final double STALL_VELOCITY_THRESHOLD = 0.1; // m/s
    }

    // ============= STATE VARIABLES =============

    public enum DriveMode {
        ARC_FOLLOWING,      // Following calculated arc
        CONVERGING,         // Converging to target
        DIRECT_APPROACH,    // Direct line to target
        FINAL_ALIGNMENT     // Final position and heading alignment
    }

    private DriveMode currentMode;
    private double dynamicAlpha;
    private double currentArcRadius;
    private Translation2d arcCenter;
    private double arcStartAngle;
    private double arcEndAngle;

    // Position tracking
    private Pose2d currentPose;
    private Pose2d previousPose;
    private Translation2d targetPosition;
    private Rotation2d targetHeading;

    // Velocity tracking
    private Translation2d currentVelocity;
    private double smoothedSpeed;
    private double smoothedHeading;

    // Controllers
    private final PIDController speedController;
    private final PIDController headingController;
    private final PIDController positionController;

    // Timing
    private double lastUpdateTime;
    private double stallStartTime;
    private boolean isStalled;

    // Path history for visualization
    private List<ArcSegment> arcHistory;
    private List<Pose2d> poseHistory;

    /**
     * Arc segment for visualization and debugging
     */
    public static class ArcSegment {
        public final Translation2d center;
        public final double radius;
        public final double startAngle;
        public final double endAngle;
        public final double timestamp;

        public ArcSegment(Translation2d center, double radius,
                         double startAngle, double endAngle, double timestamp) {
            this.center = center;
            this.radius = radius;
            this.startAngle = startAngle;
            this.endAngle = endAngle;
            this.timestamp = timestamp;
        }
    }

    // ============= CONSTRUCTOR =============

    public PilotDriveController() {
        this.speedController = new PIDController(2.0, 0.0, 0.1);
        this.headingController = new PIDController(3.0, 0.0, 0.05);
        this.positionController = new PIDController(2.5, 0.0, 0.1);

        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        this.arcHistory = new ArrayList<>();
        this.poseHistory = new ArrayList<>();

        reset();
    }

    // ============= MAIN CONTROL METHOD =============

    /**
     * Calculate robot velocities using the Pilot Drive algorithm
     *
     * @param currentPose Current robot pose
     * @param targetPose Target pose to reach
     * @param heuristicMode Heuristic mode selection (0-2)
     * @return Field-relative chassis speeds
     */
    public ChassisSpeeds calculate(Pose2d currentPose, Pose2d targetPose, int heuristicMode) {
        double currentTime = Timer.getFPGATimestamp();

        // Update state
        updateState(currentPose, targetPose, currentTime);

        // Determine drive mode
        updateDriveMode();

        // Calculate drive vector based on mode
        DriveVector driveVector;
        switch (currentMode) {
            case DIRECT_APPROACH:
                driveVector = calculateDirectApproach();
                break;
            case FINAL_ALIGNMENT:
                driveVector = calculateFinalAlignment();
                break;
            case CONVERGING:
                driveVector = calculateConvergingArc(heuristicMode);
                break;
            case ARC_FOLLOWING:
            default:
                driveVector = calculatePilotArc(heuristicMode);
                break;
        }

        // Apply velocity constraints
        driveVector = applyVelocityConstraints(driveVector);

        // Apply smoothing
        driveVector = applySmooting(driveVector);

        // Update history
        updateHistory(currentTime);

        // Convert to chassis speeds
        return convertToChassisSpeeds(driveVector, currentPose);
    }

    // ============= PILOT DRIVE CORE ALGORITHM =============

    /**
     * Calculate the pilot drive arc based on current, previous, and target positions
     */
    private DriveVector calculatePilotArc(int heuristicMode) {
        Translation2d A = currentPose.getTranslation();  // Current position
        Translation2d F = previousPose != null ?
            previousPose.getTranslation() : A.minus(new Translation2d(0.1, 0)); // Previous position
        Translation2d E = targetPosition;  // End/target position

        // Calculate base vectors
        Translation2d vectorAE = E.minus(A);  // Vector from current to target
        Translation2d vectorAF = A.minus(F);  // Vector from previous to current (motion direction)

        // Handle special case: no previous motion
        if (vectorAF.getNorm() < 0.01) {
            vectorAF = new Translation2d(0.1, 0);
        }

        // Calculate angles
        double angleToTarget = Math.atan2(vectorAE.getY(), vectorAE.getX());
        double motionAngle = Math.atan2(vectorAF.getY(), vectorAF.getX());

        // Calculate angle bisector with dynamic alpha
        dynamicAlpha = calculateDynamicAlpha(A, E, angleToTarget, motionAngle, heuristicMode);

        // Bisector angle calculation (this is the key innovation)
        double bisectorAngle = calculateBisectorAngle(angleToTarget, motionAngle, dynamicAlpha);

        // Calculate arc parameters
        ArcParameters arcParams = calculateArcParameters(A, E, bisectorAngle, vectorAF);

        // Store arc for visualization
        arcCenter = arcParams.center;
        currentArcRadius = arcParams.radius;
        arcStartAngle = arcParams.startAngle;
        arcEndAngle = arcParams.endAngle;

        // Calculate drive direction along the arc
        double driveAngle = calculateDriveAngleOnArc(A, arcParams);

        // Calculate velocity magnitude based on arc curvature
        double targetSpeed = calculateArcVelocity(arcParams.radius);

        return new DriveVector(driveAngle, targetSpeed);
    }

    /**
     * Calculate dynamic alpha based on situation
     */
    private double calculateDynamicAlpha(Translation2d current, Translation2d target,
                                         double angleToTarget, double motionAngle,
                                         int heuristicMode) {
        double distance = current.getDistance(target);
        double angleDifference = Math.abs(wrapAngle(angleToTarget - motionAngle));

        // Base alpha from configuration
        double baseAlpha;
        switch (heuristicMode) {
            case 0: // Aggressive - sharp turns
                baseAlpha = PilotConfig.ALPHA_AGGRESSIVE;
                break;
            case 1: // Balanced
                baseAlpha = PilotConfig.ALPHA_BALANCED;
                break;
            case 2: // Smooth - wide arcs
                baseAlpha = PilotConfig.ALPHA_SMOOTH;
                break;
            default:
                baseAlpha = PilotConfig.ALPHA_BALANCED;
        }

        // Adjust based on distance (closer = more direct)
        double distanceFactor = Math.exp(-distance / 2.0); // Exponential decay
        baseAlpha -= distanceFactor * PilotConfig.ALPHA_DISTANCE_FACTOR;

        // Adjust based on angle difference (larger difference = wider arc)
        double angleFactor = angleDifference / Math.PI; // Normalize to 0-1
        baseAlpha += angleFactor * PilotConfig.ALPHA_ANGLE_FACTOR;

        // Clamp to valid range
        return MathUtil.clamp(baseAlpha, 0.1, 0.9);
    }

    /**
     * Calculate the bisector angle using the alpha heuristic
     */
    private double calculateBisectorAngle(double angleToTarget, double motionAngle, double alpha) {
        // This is the core of the pilot drive algorithm
        // The bisector is weighted between motion direction and target direction

        // Calculate the angular difference
        double angleDiff = wrapAngle(angleToTarget - motionAngle);

        // Apply the alpha heuristic
        // alpha = 0: follow motion direction
        // alpha = 0.5: true bisector
        // alpha = 1: direct to target
        double bisectorOffset = angleDiff * alpha;

        // Add offset to motion angle
        double bisectorAngle = motionAngle + bisectorOffset;

        // Additional heuristic: limit maximum deviation from motion direction
        double maxDeviation = Math.PI / 3; // 60 degrees
        if (Math.abs(bisectorOffset) > maxDeviation) {
            bisectorOffset = Math.signum(bisectorOffset) * maxDeviation;
            bisectorAngle = motionAngle + bisectorOffset;
        }

        return wrapAngle(bisectorAngle);
    }

    /**
     * Calculate arc parameters from bisector angle
     */
    private ArcParameters calculateArcParameters(Translation2d current, Translation2d target,
                                                 double bisectorAngle, Translation2d motionVector) {
        // Calculate perpendicular to bisector (center direction)
        double perpAngle = bisectorAngle + Math.PI / 2;

        // Calculate chord length
        double chordLength = current.getDistance(target);

        // Calculate arc radius based on geometry
        // This creates the "constant radius" behavior mentioned in the description
        double radius;
        if (chordLength < 0.1) {
            radius = PilotConfig.DEFAULT_ARC_RADIUS;
        } else {
            // Use motion vector magnitude to influence radius
            double speed = motionVector.getNorm();
            radius = Math.max(PilotConfig.MIN_ARC_RADIUS,
                              Math.min(PilotConfig.MAX_ARC_RADIUS,
                                      speed * 2.0 + chordLength / 4.0));
        }

        // Calculate arc center
        // The center is perpendicular to the bisector at distance radius
        Translation2d centerOffset = new Translation2d(
            radius * Math.cos(perpAngle),
            radius * Math.sin(perpAngle)
        );
        Translation2d center = current.plus(centerOffset);

        // Calculate start and end angles on the arc
        double startAngle = Math.atan2(
            current.getY() - center.getY(),
            current.getX() - center.getX()
        );

        double endAngle = Math.atan2(
            target.getY() - center.getY(),
            target.getX() - center.getX()
        );

        return new ArcParameters(center, radius, startAngle, endAngle);
    }

    /**
     * Calculate drive angle along the arc (tangent to arc)
     */
    private double calculateDriveAngleOnArc(Translation2d current, ArcParameters arc) {
        // Vector from center to current position
        Translation2d radiusVector = current.minus(arc.center);
        double radiusAngle = Math.atan2(radiusVector.getY(), radiusVector.getX());

        // Determine arc direction (clockwise or counter-clockwise)
        double angleDiff = wrapAngle(arc.endAngle - arc.startAngle);
        boolean clockwise = angleDiff < 0;

        // Tangent is perpendicular to radius
        double tangentAngle;
        if (clockwise) {
            tangentAngle = radiusAngle - Math.PI / 2;
        } else {
            tangentAngle = radiusAngle + Math.PI / 2;
        }

        return wrapAngle(tangentAngle);
    }

    /**
     * Calculate velocity based on arc radius (tighter turns = slower)
     */
    private double calculateArcVelocity(double radius) {
        // Curvature-based velocity
        double curvature = 1.0 / radius;
        double velocityFromCurvature = PilotConfig.CURVATURE_VELOCITY_FACTOR /
                                       Math.max(curvature, 0.2);

        // Distance-based velocity
        double distanceToTarget = currentPose.getTranslation().getDistance(targetPosition);
        double velocityFromDistance = Math.sqrt(2 * 2.0 * distanceToTarget); // Deceleration curve

        // Take minimum and apply constraints
        double targetVelocity = Math.min(velocityFromCurvature, velocityFromDistance);
        targetVelocity = MathUtil.clamp(targetVelocity,
                                        PilotConfig.MIN_VELOCITY,
                                        PilotConfig.MAX_VELOCITY);

        return targetVelocity;
    }

    // ============= CONVERGING MODE =============

    /**
     * Calculate arc for converging mode (smaller radius, more direct)
     */
    private DriveVector calculateConvergingArc(int heuristicMode) {
        // Use more aggressive alpha for convergence
        DriveVector arcVector = calculatePilotArc(heuristicMode);

        // Reduce arc radius for tighter approach
        currentArcRadius *= 0.6;

        // Increase direct approach weight
        Translation2d toTarget = targetPosition.minus(currentPose.getTranslation());
        double directAngle = Math.atan2(toTarget.getY(), toTarget.getX());

        // Blend arc angle with direct angle
        double blendFactor = 0.4; // 40% direct, 60% arc
        double blendedAngle = blendAngles(arcVector.angle, directAngle, blendFactor);

        return new DriveVector(blendedAngle, arcVector.speed * 0.7);
    }

    // ============= DIRECT APPROACH MODE =============

    /**
     * Calculate direct line approach to target
     */
    private DriveVector calculateDirectApproach() {
        Translation2d toTarget = targetPosition.minus(currentPose.getTranslation());
        double distance = toTarget.getNorm();
        double angle = Math.atan2(toTarget.getY(), toTarget.getX());

        // Slow down as approaching target
        double speed = Math.min(PilotConfig.MAX_VELOCITY, distance * 3.0);
        speed = Math.max(PilotConfig.MIN_VELOCITY, speed);

        return new DriveVector(angle, speed);
    }

    // ============= FINAL ALIGNMENT MODE =============

    /**
     * Final position and heading alignment
     */
    private DriveVector calculateFinalAlignment() {
        Translation2d positionError = targetPosition.minus(currentPose.getTranslation());
        double headingError = wrapAngle(targetHeading.getRadians() -
                                       currentPose.getRotation().getRadians());

        // Position correction
        double positionSpeed = positionController.calculate(positionError.getNorm(), 0);
        double positionAngle = Math.atan2(positionError.getY(), positionError.getX());

        // Heading correction (reduced during position correction)
        double headingCorrection = headingController.calculate(headingError, 0);
        headingCorrection *= Math.max(0.2, 1.0 - positionError.getNorm() * 2.0);

        return new DriveVector(positionAngle, positionSpeed, headingCorrection);
    }

    // ============= HELPER METHODS =============

    /**
     * Update internal state
     */
    private void updateState(Pose2d newPose, Pose2d targetPose, double currentTime) {
        // Update poses
        previousPose = currentPose;
        currentPose = newPose;
        targetPosition = targetPose.getTranslation();
        targetHeading = targetPose.getRotation();

        // Update velocity estimate
        if (previousPose != null && lastUpdateTime > 0) {
            double dt = currentTime - lastUpdateTime;
            if (dt > 0) {
                Translation2d deltaPos = currentPose.getTranslation()
                    .minus(previousPose.getTranslation());
                currentVelocity = deltaPos.div(dt);
            }
        }

        // Check for stall
        if (currentVelocity != null && currentVelocity.getNorm() < PilotConfig.STALL_VELOCITY_THRESHOLD) {
            if (!isStalled) {
                stallStartTime = currentTime;
                isStalled = true;
            } else if (currentTime - stallStartTime > PilotConfig.STALL_DETECTION_TIME) {
                // Handle stall - could switch to recovery mode
                SmartDashboard.putBoolean("PilotDrive/Stalled", true);
            }
        } else {
            isStalled = false;
            SmartDashboard.putBoolean("PilotDrive/Stalled", false);
        }

        lastUpdateTime = currentTime;
    }

    /**
     * Determine current drive mode based on distance and conditions
     */
    private void updateDriveMode() {
        double distanceToTarget = currentPose.getTranslation().getDistance(targetPosition);

        if (distanceToTarget < PilotConfig.ARRIVAL_TOLERANCE) {
            currentMode = DriveMode.FINAL_ALIGNMENT;
        } else if (distanceToTarget < PilotConfig.DIRECT_APPROACH_DISTANCE) {
            currentMode = DriveMode.DIRECT_APPROACH;
        } else if (distanceToTarget < PilotConfig.CONVERGENCE_DISTANCE) {
            currentMode = DriveMode.CONVERGING;
        } else {
            currentMode = DriveMode.ARC_FOLLOWING;
        }

        SmartDashboard.putString("PilotDrive/Mode", currentMode.toString());
    }

    /**
     * Apply velocity constraints and safety limits
     */
    private DriveVector applyVelocityConstraints(DriveVector input) {
        // Limit speed
        double constrainedSpeed = MathUtil.clamp(input.speed,
                                                 PilotConfig.MIN_VELOCITY,
                                                 PilotConfig.MAX_VELOCITY);

        // Limit heading change rate
        if (!Double.isNaN(smoothedHeading)) {
            double headingDiff = wrapAngle(input.angle - smoothedHeading);
            double maxChange = PilotConfig.MAX_HEADING_CHANGE_RATE * 0.02; // Convert to per-loop

            if (Math.abs(headingDiff) > maxChange) {
                headingDiff = Math.signum(headingDiff) * maxChange;
            }

            double constrainedAngle = smoothedHeading + headingDiff;
            return new DriveVector(constrainedAngle, constrainedSpeed, input.rotation);
        }

        return new DriveVector(input.angle, constrainedSpeed, input.rotation);
    }

    /**
     * Apply smoothing to reduce jitter
     */
    private DriveVector applySmooting(DriveVector input) {
        // Smooth heading
        if (!Double.isNaN(smoothedHeading)) {
            smoothedHeading = smoothedHeading * PilotConfig.HEADING_SMOOTHING +
                             input.angle * (1.0 - PilotConfig.HEADING_SMOOTHING);
        } else {
            smoothedHeading = input.angle;
        }

        // Smooth speed
        if (!Double.isNaN(smoothedSpeed)) {
            smoothedSpeed = smoothedSpeed * PilotConfig.VELOCITY_SMOOTHING +
                           input.speed * (1.0 - PilotConfig.VELOCITY_SMOOTHING);
        } else {
            smoothedSpeed = input.speed;
        }

        return new DriveVector(smoothedHeading, smoothedSpeed, input.rotation);
    }

    /**
     * Update history for visualization
     */
    private void updateHistory(double timestamp) {
        // Add current arc to history
        if (arcCenter != null && currentArcRadius > 0) {
            arcHistory.add(new ArcSegment(arcCenter, currentArcRadius,
                                         arcStartAngle, arcEndAngle, timestamp));

            // Limit history size
            if (arcHistory.size() > 100) {
                arcHistory.remove(0);
            }
        }

        // Add pose to history
        poseHistory.add(currentPose);
        if (poseHistory.size() > 200) {
            poseHistory.remove(0);
        }
    }

    /**
     * Convert drive vector to chassis speeds
     */
    private ChassisSpeeds convertToChassisSpeeds(DriveVector driveVector, Pose2d currentPose) {
        // Field-relative velocities
        double vx = driveVector.speed * Math.cos(driveVector.angle);
        double vy = driveVector.speed * Math.sin(driveVector.angle);
        double omega = driveVector.rotation;

        // Convert to robot-relative
        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, currentPose.getRotation());
    }

    /**
     * Blend two angles with given weight
     */
    private double blendAngles(double angle1, double angle2, double weight) {
        // Handle angle wrapping properly
        double diff = wrapAngle(angle2 - angle1);
        return wrapAngle(angle1 + diff * weight);
    }

    /**
     * Wrap angle to [-π, π]
     */
    private double wrapAngle(double angle) {
        return MathUtil.inputModulus(angle, -Math.PI, Math.PI);
    }

    /**
     * Reset controller state
     */
    public void reset() {
        currentMode = DriveMode.ARC_FOLLOWING;
        dynamicAlpha = PilotConfig.ALPHA_BALANCED;
        currentArcRadius = PilotConfig.DEFAULT_ARC_RADIUS;
        arcCenter = null;

        currentPose = null;
        previousPose = null;
        currentVelocity = new Translation2d();
        smoothedSpeed = Double.NaN;
        smoothedHeading = Double.NaN;

        lastUpdateTime = 0;
        stallStartTime = 0;
        isStalled = false;

        arcHistory.clear();
        poseHistory.clear();

        speedController.reset();
        headingController.reset();
        positionController.reset();
    }

    /**
     * Get telemetry for debugging
     */
    public PilotTelemetry getTelemetry() {
        return new PilotTelemetry(
            currentMode,
            dynamicAlpha,
            currentArcRadius,
            arcCenter,
            smoothedHeading,
            smoothedSpeed,
            currentPose,
            targetPosition,
            arcHistory.size()
        );
    }

    // ============= HELPER CLASSES =============

    /**
     * Arc parameters
     */
    private static class ArcParameters {
        public final Translation2d center;
        public final double radius;
        public final double startAngle;
        public final double endAngle;

        public ArcParameters(Translation2d center, double radius,
                            double startAngle, double endAngle) {
            this.center = center;
            this.radius = radius;
            this.startAngle = startAngle;
            this.endAngle = endAngle;
        }
    }

    /**
     * Drive vector representation
     */
    private static class DriveVector {
        public final double angle;     // Direction to drive (field-relative)
        public final double speed;     // Speed magnitude
        public final double rotation;  // Rotational velocity

        public DriveVector(double angle, double speed) {
            this(angle, speed, 0);
        }

        public DriveVector(double angle, double speed, double rotation) {
            this.angle = angle;
            this.speed = speed;
            this.rotation = rotation;
        }
    }

    /**
     * Telemetry data
     */
    public static class PilotTelemetry {
        public final DriveMode mode;
        public final double alpha;
        public final double arcRadius;
        public final Translation2d arcCenter;
        public final double heading;
        public final double speed;
        public final Pose2d currentPose;
        public final Translation2d targetPosition;
        public final int arcHistorySize;

        public PilotTelemetry(DriveMode mode, double alpha, double arcRadius,
                             Translation2d arcCenter, double heading, double speed,
                             Pose2d currentPose, Translation2d targetPosition,
                             int arcHistorySize) {
            this.mode = mode;
            this.alpha = alpha;
            this.arcRadius = arcRadius;
            this.arcCenter = arcCenter;
            this.heading = heading;
            this.speed = speed;
            this.currentPose = currentPose;
            this.targetPosition = targetPosition;
            this.arcHistorySize = arcHistorySize;
        }
    }
}
