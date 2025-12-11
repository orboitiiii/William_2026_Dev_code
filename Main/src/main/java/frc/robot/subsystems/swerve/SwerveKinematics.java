package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/** Custom Swerve Kinematics (1690/Orbit Style). Strictly avoids WPILib SwerveDriveKinematics. */
public class SwerveKinematics {
  private final Translation2d[] mModuleLocations;

  public SwerveKinematics() {
    // Access via Constants (assuming standard rect 4-wheel)
    // FL, FR, BL, BR
    mModuleLocations =
        new Translation2d[] {
          new Translation2d(Constants.Swerve.kWheelBase / 2.0, Constants.Swerve.kTrackWidth / 2.0),
          new Translation2d(Constants.Swerve.kWheelBase / 2.0, -Constants.Swerve.kTrackWidth / 2.0),
          new Translation2d(-Constants.Swerve.kWheelBase / 2.0, Constants.Swerve.kTrackWidth / 2.0),
          new Translation2d(-Constants.Swerve.kWheelBase / 2.0, -Constants.Swerve.kTrackWidth / 2.0)
        };
  }

  /** Inverse Kinematics: ChassisSpeeds -> Module States v_module = v_robot + omega x r */
  public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds speeds) {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      // v_x = v_rx - omega * r_y
      // v_y = v_ry + omega * r_x
      double vx =
          speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * mModuleLocations[i].getY();
      double vy =
          speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * mModuleLocations[i].getX();

      double speed = Math.hypot(vx, vy);
      Rotation2d angle = new Rotation2d(vx, vy);

      states[i] = new SwerveModuleState(speed, angle);
    }

    // De-saturate wheel speeds
    desaturateWheelSpeeds(states, Constants.Swerve.kMaxDriveVelocity);

    return states;
  }

  /**
   * Forward Kinematics (Odometry Step 1.1): Translation Decomposition Calculates the robot-relative
   * movement delta based on module deltas.
   *
   * @param moduleDeltas Array of module displacements (in meters)
   * @param moduleAngles Array of module angles (during the movement)
   * @return Translation2d representing robot-relative delta (dx, dy)
   */
  public Translation2d estimateRobotRelativeMove(double[] moduleDeltas, Rotation2d[] moduleAngles) {
    double totalCos = 0.0;
    double totalSin = 0.0;

    // "Vector Averaging" for Translation
    // Delta x_robot = 1/4 * Sum(Delta_d * cos(theta))
    // Delta y_robot = 1/4 * Sum(Delta_d * sin(theta))
    // This assumes the rotation component sums to zero at the geometric center (Topological
    // property of Swerve).
    // Valid for symmetric swerve mainly? Yes, 1690 uses this assumption.

    for (int i = 0; i < 4; i++) {
      totalCos += moduleDeltas[i] * moduleAngles[i].getCos();
      totalSin += moduleDeltas[i] * moduleAngles[i].getSin();
    }

    return new Translation2d(totalCos / 4.0, totalSin / 4.0);
  }

  private void desaturateWheelSpeeds(SwerveModuleState[] states, double maxVelocity) {
    double maxSpeed = 0.0;
    for (SwerveModuleState state : states) {
      maxSpeed = Math.max(maxSpeed, Math.abs(state.speedMetersPerSecond));
    }

    if (maxSpeed > maxVelocity) {
      double scale = maxVelocity / maxSpeed;
      for (SwerveModuleState state : states) {
        state.speedMetersPerSecond *= scale;
      }
    }
  }
}
