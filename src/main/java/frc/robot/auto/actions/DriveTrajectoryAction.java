package frc.robot.auto.actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.PoseHistory;
import frc.robot.auto.trajectory.Trajectory;
import frc.robot.auto.trajectory.TrajectoryFollower;
import frc.robot.subsystems.RobotStateEstimator;
import frc.robot.subsystems.drive.Drive;

/**
 * Action that follows a pre-planned trajectory using a standard PID + Feedforward strategy.
 *
 * <p><strong>First Principles & Aerospace Rigor</strong>: While less complex than MPC, this action
 * provides high-performance tracking by combining velocity feedforward (to eliminate phase lag)
 * with PID feedback (to correct for disturbances).
 */
public class DriveTrajectoryAction implements Action {
  // --- Systems ---
  private final Drive mDrive;
  private final Trajectory mTrajectory;
  private final boolean mResetOdometry;
  private final TrajectoryFollower mFollower;

  // --- Runtime State ---
  private final edu.wpi.first.wpilibj.Timer mTimer = new edu.wpi.first.wpilibj.Timer();
  private double mLastUpdateTime = -1.0;
  private double mLagOffset = 0.0;
  private boolean mIsFinished = false;

  /**
   * Creates a trajectory following action.
   *
   * @param trajectory The trajectory to follow.
   * @param resetOdometry If true, resets odometry to trajectory start pose.
   */
  public DriveTrajectoryAction(Trajectory trajectory, boolean resetOdometry) {
    mDrive = Drive.getInstance();
    mTrajectory = trajectory;
    mResetOdometry = resetOdometry;
    mFollower = new TrajectoryFollower();
  }

  public DriveTrajectoryAction(Trajectory trajectory) {
    this(trajectory, false);
  }

  @Override
  public void start() {
    // 1. Reset Odometry (if requested)
    if (mResetOdometry) {
      Pose2d initialPose = mTrajectory.getInitialPose();
      mDrive.resetOdometry(initialPose);
      PoseHistory.getInstance().reset(0.0, initialPose);
    }

    // 2. Start Follower
    mFollower.start(mTrajectory);

    mTimer.reset();
    mTimer.start();
    mLastUpdateTime = mTimer.get();
    mLagOffset = 0.0;
    mDrive.startPathFollowing();
  }

  @Override
  public void update() {
    if (mIsFinished) return;

    double currentTime = mTimer.get();

    // Safety: Catch-up prevention
    if (mLastUpdateTime > 0 && (currentTime - mLastUpdateTime) > 0.1) {
      double lag = currentTime - mLastUpdateTime;
      // System.err.println("[9427-TRAJ] WARNING: Large lag detected (" + (int) (lag *
      // 1000) + "ms). Skipping catch-up.");
      mLagOffset += lag;
    }
    mLastUpdateTime = currentTime;

    // Adjusted time for trajectory sampling
    double timeSinceStart = currentTime - mLagOffset;

    // Check for completion
    if (timeSinceStart > mTrajectory.getTotalTimeSeconds()) {
      // System.err.println("[9427-TRAJ] Path complete at t=" + timeSinceStart);
      mIsFinished = true;
      return;
    }

    // Get current pose from the authoritative state estimator
    Pose2d currentPose = RobotStateEstimator.getInstance().getEstimatedPose();

    // Calculate control output using PID + Feedforward
    // TrajectoryFollower inside uses Timer.getFPGATimestamp() for its internal
    // clock
    // if we use start(trajectory). But we want deterministic lag-handling.
    // So we use the specific calculate(pose, adjustedTime) overload.
    // Note: TrajectoryFollower.calculate(pose, time) expects ABSOLUTE FPGA
    // timestamp.
    // We need to convert our action-relative timeSinceStart to absolute.
    double absoluteTargetTime = mFollower.getStartTimeSeconds() + timeSinceStart;

    ChassisSpeeds fieldSpeeds = mFollower.calculate(currentPose, absoluteTargetTime);

    // Convert to robot-relative for the drive system
    ChassisSpeeds finalSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vxMetersPerSecond,
            fieldSpeeds.vyMetersPerSecond,
            fieldSpeeds.omegaRadiansPerSecond,
            currentPose.getRotation());

    // Debugging (10Hz) - Commented out to clean up console as per user request
    /*
     * if (mTimer.hasElapsed(mLastLogTime + 0.1)) {
     * Pose2d refPose = mTrajectory.sample(timeSinceStart).pose();
     * System.err.printf(
     * "[9427-TRAJ] t=%.2f | REF:(%.2f,%.2f,%.1f°) | CUR:(%.2f,%.2f,%.1f°) | CMD:(%.2f,%.2f,%.2f)%n"
     * ,
     * timeSinceStart,
     * refPose.getX(),
     * refPose.getY(),
     * Math.toDegrees(refPose.getRotation().getRadians()),
     * currentPose.getX(),
     * currentPose.getY(),
     * Math.toDegrees(currentPose.getRotation().getRadians()),
     * fieldSpeeds.vxMetersPerSecond,
     * fieldSpeeds.vyMetersPerSecond,
     * fieldSpeeds.omegaRadiansPerSecond);
     * mLastLogTime = currentTime;
     * }
     */

    mDrive.setPathFollowingSpeeds(finalSpeeds);
  }

  // private double mLastLogTime = 0.0; // Unused after log cleanup

  @Override
  public boolean isFinished() {
    return mIsFinished || mFollower.isFinished();
  }

  @Override
  public void done() {
    System.err.println("[9427-TRAJ] Path Follower Done.");
    mFollower.stop();
    mDrive.stop();
  }
}
