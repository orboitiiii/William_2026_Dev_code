package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

/**
 * 1690-Style Kinematic Feasibility Filter. This class ensures that the requested module states are
 * physically possible given the robot's mass, motor torque, and tire friction.
 */
public class SwerveSetpointGenerator {
  private final SwerveDriveKinematics mKinematics;

  // Store the LAST commanded state to calculate acceleration
  private SwerveModuleState[] mPrevSetpoint;

  public SwerveSetpointGenerator(SwerveDriveKinematics kinematics) {
    mKinematics = kinematics;
    // Initialize with zeros
    mPrevSetpoint = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      mPrevSetpoint[i] = new SwerveModuleState(0.0, new Rotation2d());
    }
  }

  /** Main function: Takes desired speeds, returns FEASIBLE module states. */
  public SwerveModuleState[] generateSetpoint(ChassisSpeeds desiredSpeeds) {

    SwerveModuleState[] idealStates = mKinematics.toSwerveModuleStates(desiredSpeeds);

    for (int i = 0; i < 4; i++) {
      idealStates[i].optimize(mPrevSetpoint[i].angle);
    }

    SwerveModuleState[] feasibleStates = enforceFeasibility(idealStates, Constants.kLooperDt);

    mPrevSetpoint = feasibleStates;

    return feasibleStates;
  }

  private SwerveModuleState[] enforceFeasibility(SwerveModuleState[] idealStates, double dt) {
    SwerveModuleState[] feasibleStates = new SwerveModuleState[4];

    // Check for slip
    double scaleFactor = SwerveSlipDetector.getSlipScalingFactor(mPrevSetpoint, idealStates, dt);

    // Apply Scaling and Calculate Final States
    for (int i = 0; i < 4; i++) {
      // Re-calculate accel vector (redundant calculation but cleaner separation)
      Translation2d vCurrent =
          new Translation2d(mPrevSetpoint[i].speedMetersPerSecond, mPrevSetpoint[i].angle);
      Translation2d vTarget =
          new Translation2d(idealStates[i].speedMetersPerSecond, idealStates[i].angle);

      Translation2d accel = vTarget.minus(vCurrent).div(dt);
      Translation2d validAccel = accel.times(scaleFactor);

      Translation2d vNext = vCurrent.plus(validAccel.times(dt));

      Rotation2d nextAngle = mPrevSetpoint[i].angle;
      if (vNext.getNorm() > 1e-6) {
        nextAngle = vNext.getAngle();
      }

      feasibleStates[i] = new SwerveModuleState(vNext.getNorm(), nextAngle);
    }

    return feasibleStates;
  }

  public void reset() {
    for (int i = 0; i < 4; i++) {
      mPrevSetpoint[i] = new SwerveModuleState(0.0, new Rotation2d());
    }
  }
}
