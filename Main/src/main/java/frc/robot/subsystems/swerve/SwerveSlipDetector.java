package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveSlipDetector {

  public static double getSlipScalingFactor(
      SwerveModuleState[] prevSetpoints, SwerveModuleState[] idealStates, double dt) {
    double maxRequiredAccel = 0.0;

    for (int i = 0; i < 4; i++) {
      // Current Velocity Vector
      Translation2d vCurrent =
          new Translation2d(prevSetpoints[i].speedMetersPerSecond, prevSetpoints[i].angle);

      // Target Velocity Vector
      Translation2d vTarget =
          new Translation2d(idealStates[i].speedMetersPerSecond, idealStates[i].angle);

      // Acceleration = (V_target - V_current) / dt
      Translation2d accel = vTarget.minus(vCurrent).div(dt);

      if (accel.getNorm() > maxRequiredAccel) {
        maxRequiredAccel = accel.getNorm();
      }
    }

    if (maxRequiredAccel > Constants.Swerve.kMaxDriveAcceleration) {
      return Constants.Swerve.kMaxDriveAcceleration / maxRequiredAccel;
    }

    return 1.0;
  }
}
