package frc.robot.subsystems.driveTrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.swerve.Gyro;

/**
 * * Concrete implementation of the Gyro interface for the CTRE Pigeon 2. First Principles: Provides
 * raw kinematic data (Pose and Twist) for the odometry estimator.
 */
public class Pigeon2Gyro implements Gyro {

  private final Pigeon2 pigeon;
  private final com.ctre.phoenix6.StatusSignal<Angle> yawSignal;
  private final com.ctre.phoenix6.StatusSignal<AngularVelocity> yawRateSignal;

  public Pigeon2Gyro(int canId, String canBus) {
    this.pigeon = new Pigeon2(canId, canBus);

    // Configuration
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.MountPoseYaw = 0;
    config.MountPose.MountPosePitch = 0;
    config.MountPose.MountPoseRoll = 0;
    pigeon.getConfigurator().apply(config);

    // Signal Optimization (Phoenix 6)
    this.yawSignal = pigeon.getYaw();
    this.yawRateSignal = pigeon.getAngularVelocityZWorld(); // Correct physical quantity

    // Update frequency matches the robot loop (typically 50Hz, but 100Hz gives better integration)
    this.yawSignal.setUpdateFrequency(100);
    this.yawRateSignal.setUpdateFrequency(100);

    resetYaw();
  }

  @Override
  public Rotation2d getYaw() {
    // Refresh ensures we get the latest data frame from the CAN bus
    yawSignal.refresh();
    return Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
  }

  @Override
  public double getYawRateRadPerSec() {
    yawRateSignal.refresh();
    // Convert explicitly to Radians per Second
    return yawRateSignal.getValue().in(edu.wpi.first.units.Units.RadiansPerSecond);
  }

  @Override
  public void resetYaw() {
    pigeon.setYaw(0.0);
  }
}
