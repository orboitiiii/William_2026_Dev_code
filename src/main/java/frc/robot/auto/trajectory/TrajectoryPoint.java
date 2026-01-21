package frc.robot.auto.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Immutable trajectory waypoint with pose and velocity.
 *
 * <p><strong>Design Rationale (First Principles)</strong>: A complete description of the robot's
 * kinematic state requires:
 *
 * <ul>
 *   <li><strong>Position (x, y, θ)</strong>: For PID feedback control
 *   <li><strong>Velocity (vx, vy, ω)</strong>: For feedforward control
 *   <li><strong>Timestamp</strong>: For trajectory interpolation and synchronization
 * </ul>
 *
 * <p><strong>Coordinate Frame</strong>: All values are in the field coordinate system (origin at
 * field corner, +X toward opposing alliance, +Y left, +θ CCW).
 *
 * @param timeSeconds Relative time from trajectory start (seconds).
 * @param pose Robot pose in field coordinates.
 * @param speeds Robot velocity in field coordinates.
 */
public record TrajectoryPoint(double timeSeconds, Pose2d pose, ChassisSpeeds speeds) {

  /**
   * Factory method for creating a point from raw CSV values.
   *
   * @param time Time in seconds.
   * @param x X position in meters.
   * @param y Y position in meters.
   * @param yawRad Heading in radians.
   * @param vx X velocity in m/s.
   * @param vy Y velocity in m/s.
   * @param omega Angular velocity in rad/s.
   * @return A new TrajectoryPoint instance.
   */
  public static TrajectoryPoint fromCsvRow(
      double time, double x, double y, double yawRad, double vx, double vy, double omega) {
    return new TrajectoryPoint(
        time, new Pose2d(x, y, new Rotation2d(yawRad)), new ChassisSpeeds(vx, vy, omega));
  }

  /**
   * Linearly interpolates between this point and another.
   *
   * @param other The target point (higher timestamp).
   * @param t Interpolation factor [0, 1]. 0 = this, 1 = other.
   * @return The interpolated trajectory point.
   */
  public TrajectoryPoint interpolate(TrajectoryPoint other, double t) {
    // Time interpolation
    double newTime = timeSeconds + (other.timeSeconds - timeSeconds) * t;

    // Pose interpolation (WPILib handles angle wraparound)
    Pose2d newPose = pose.interpolate(other.pose, t);

    // Velocity linear interpolation
    ChassisSpeeds newSpeeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond
                + (other.speeds.vxMetersPerSecond - speeds.vxMetersPerSecond) * t,
            speeds.vyMetersPerSecond
                + (other.speeds.vyMetersPerSecond - speeds.vyMetersPerSecond) * t,
            speeds.omegaRadiansPerSecond
                + (other.speeds.omegaRadiansPerSecond - speeds.omegaRadiansPerSecond) * t);

    return new TrajectoryPoint(newTime, newPose, newSpeeds);
  }
}
