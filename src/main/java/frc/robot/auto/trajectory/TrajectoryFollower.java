package frc.robot.auto.trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Controller for following pre-planned trajectories.
 *
 * <p><strong>Control Strategy</strong>: PID position feedback + velocity feedforward.
 *
 * <p>Mathematical formulation:
 *
 * <pre>
 * u = K_p * (r - x) + r_dot
 *
 * where:
 *   u     = control output (ChassisSpeeds)
 *   K_p   = proportional gain
 *   r     = reference position from trajectory
 *   x     = current position from estimator
 *   r_dot = reference velocity (feedforward)
 * </pre>
 *
 * <p><strong>Design Rationale (First Principles)</strong>:
 *
 * <ul>
 *   <li><strong>Feedforward Eliminates Lag</strong>: Predictive control based on the trajectory
 *       velocity prevents steady-state tracking error.
 *   <li><strong>PID Handles Disturbances</strong>: Feedback only corrects for external factors
 *       (friction variation, collisions, etc.).
 *   <li><strong>Angle Continuity</strong>: Theta controller uses {@code enableContinuousInput()} to
 *       handle the ±π wraparound.
 * </ul>
 *
 * <p><strong>Output</strong>: Field-relative ChassisSpeeds. The caller must convert to
 * robot-relative if needed.
 *
 * @see Trajectory
 * @see TrajectoryPoint
 */
public class TrajectoryFollower {
  // --- PID Controllers ---
  private final PIDController mXController;
  private final PIDController mYController;
  private final PIDController mThetaController;

  // --- Trajectory State ---
  private Trajectory mCurrentTrajectory;
  private double mStartTimeSeconds;
  private boolean mIsFinished;

  // --- Tunable Parameters ---
  // --- Tunable Parameters ---
  // Moved to Constants.Auto for easy tuning

  /** Constructs a trajectory follower with default PID gains. */
  public TrajectoryFollower() {
    mXController = new PIDController(Constants.Auto.kXP, Constants.Auto.kXI, Constants.Auto.kXD);
    mYController = new PIDController(Constants.Auto.kYP, Constants.Auto.kYI, Constants.Auto.kYD);
    mThetaController =
        new PIDController(Constants.Auto.kThetaP, Constants.Auto.kThetaI, Constants.Auto.kThetaD);

    // Handle angle wraparound at ±π
    mThetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Set tolerances for atSetpoint() checks
    mXController.setTolerance(Constants.Auto.kPositionTolerance);
    mYController.setTolerance(Constants.Auto.kPositionTolerance);
    mThetaController.setTolerance(Constants.Auto.kThetaTolerance);

    mIsFinished = true;
  }

  /**
   * Starts following a new trajectory.
   *
   * <p>Resets PID controllers and records the start time.
   *
   * @param trajectory The trajectory to follow.
   */
  public void start(Trajectory trajectory) {
    mCurrentTrajectory = trajectory;
    mStartTimeSeconds = Timer.getFPGATimestamp();
    mIsFinished = false;

    // Reset PID integrators
    mXController.reset();
    mYController.reset();
    mThetaController.reset();
  }

  /** Stops trajectory following. */
  public void stop() {
    mIsFinished = true;
    mCurrentTrajectory = null;
  }

  /**
   * Computes the control output for the current timestep.
   *
   * <p>Call this once per control cycle (~20ms).
   *
   * @param currentPose The robot's current estimated pose.
   * @return Field-relative ChassisSpeeds command.
   */
  public ChassisSpeeds calculate(Pose2d currentPose) {
    if (mCurrentTrajectory == null || mIsFinished) {
      return new ChassisSpeeds();
    }

    // Compute elapsed time
    double elapsedTime = Timer.getFPGATimestamp() - mStartTimeSeconds;

    // --- Settle Logic ---
    // If we have exceeded the trajectory time, keep using the terminal pose
    // for an extra 0.5s to allow PID to stabilize.
    double totalTime = mCurrentTrajectory.getTotalTimeSeconds();
    double settleTime = 0.5; // seconds

    if (elapsedTime > totalTime + settleTime) {
      mIsFinished = true;
      return new ChassisSpeeds();
    }

    // --- Lookahead Logic ---
    // Instead of sampling at 'elapsedTime', we sample ahead to compensate for lag.
    double sampleTime = elapsedTime + Constants.Auto.kLookaheadTime;

    // Sample reference state (with linear interpolation)
    TrajectoryPoint target = mCurrentTrajectory.sample(sampleTime);
    Pose2d targetPose = target.pose();
    ChassisSpeeds feedforward = target.speeds();

    // Zero feedforward for the last 3 points (User Request)
    // This allows the robot to settle gently using only PID.
    double cutoffTime = mCurrentTrajectory.getTimeAtLastPointMinus(3);
    if (sampleTime > cutoffTime && cutoffTime > 0) {
      feedforward = new ChassisSpeeds(0, 0, 0);
    }

    // --- PID Feedback ---
    double xFeedback = mXController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = mYController.calculate(currentPose.getY(), targetPose.getY());
    double thetaFeedback =
        mThetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // --- Combined Output (Field-Relative) ---
    return new ChassisSpeeds(
        feedforward.vxMetersPerSecond + xFeedback,
        feedforward.vyMetersPerSecond + yFeedback,
        feedforward.omegaRadiansPerSecond + thetaFeedback);
  }

  /**
   * Computes control output with an external timestamp.
   *
   * <p>Use this overload for synchronized multi-sensor fusion.
   *
   * @param currentPose The robot's current estimated pose.
   * @param timestampSeconds External FPGA timestamp.
   * @return Field-relative ChassisSpeeds command.
   */
  public ChassisSpeeds calculate(Pose2d currentPose, double timestampSeconds) {
    if (mCurrentTrajectory == null || mIsFinished) {
      return new ChassisSpeeds();
    }

    double elapsedTime = timestampSeconds - mStartTimeSeconds;
    double totalTime = mCurrentTrajectory.getTotalTimeSeconds();
    double settleTime = 0.5;

    if (elapsedTime > totalTime + settleTime) {
      mIsFinished = true;
      return new ChassisSpeeds();
    }

    TrajectoryPoint target = mCurrentTrajectory.sample(elapsedTime);
    Pose2d targetPose = target.pose();
    ChassisSpeeds feedforward = target.speeds();

    double xFeedback = mXController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = mYController.calculate(currentPose.getY(), targetPose.getY());
    double thetaFeedback =
        mThetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    return new ChassisSpeeds(
        feedforward.vxMetersPerSecond + xFeedback,
        feedforward.vyMetersPerSecond + yFeedback,
        feedforward.omegaRadiansPerSecond + thetaFeedback);
  }

  /**
   * Returns whether the trajectory is complete.
   *
   * @return True if finished or no trajectory is active.
   */
  public boolean isFinished() {
    return mIsFinished;
  }

  /**
   * Returns the elapsed time since trajectory start.
   *
   * @return Elapsed time in seconds.
   */
  public double getElapsedTime() {
    if (mCurrentTrajectory == null) {
      return 0;
    }
    return Timer.getFPGATimestamp() - mStartTimeSeconds;
  }

  /**
   * Returns the start time of the trajectory.
   *
   * @return Start time in FPGA seconds.
   */
  public double getStartTimeSeconds() {
    return mStartTimeSeconds;
  }

  /**
   * Returns the current target pose (for telemetry).
   *
   * @return The reference pose at the current time.
   */
  public Pose2d getTargetPose() {
    if (mCurrentTrajectory == null || mIsFinished) {
      return new Pose2d();
    }
    double elapsedTime = Timer.getFPGATimestamp() - mStartTimeSeconds;
    return mCurrentTrajectory.sample(elapsedTime).pose();
  }

  /**
   * Sets the X-axis PID gains.
   *
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   */
  public void setXPID(double p, double i, double d) {
    mXController.setPID(p, i, d);
  }

  /**
   * Sets the Y-axis PID gains.
   *
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   */
  public void setYPID(double p, double i, double d) {
    mYController.setPID(p, i, d);
  }

  /**
   * Sets the heading PID gains.
   *
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   */
  public void setThetaPID(double p, double i, double d) {
    mThetaController.setPID(p, i, d);
  }
}
