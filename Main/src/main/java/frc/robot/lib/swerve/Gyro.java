package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Interface for a gyroscope implementation. This allows swapping gyros (e.g., Pigeon2, NavX)
 * without changing subsystem code.
 */
public interface Gyro {
  /**
   * Gets the current yaw (rotation) of the robot as a Rotation2d. This is the fused heading.
   *
   * @return The current yaw.
   */
  Rotation2d getYaw();

  /**
   * Gets the current yaw rate (rotation speed) in radians per second.
   *
   * @return The current yaw rate.
   */
  double getYawRateRadPerSec();

  /** Resets the gyro's current yaw angle to zero. */
  void resetYaw();
}
