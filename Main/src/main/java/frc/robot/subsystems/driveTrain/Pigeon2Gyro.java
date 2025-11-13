package frc.robot.subsystems.driveTrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.lib.swerve.Gyro;

/** Concrete implementation of the Gyro interface for the CTRE Pigeon 2. Uses Phoenix 6 API. */
public class Pigeon2Gyro implements Gyro {

  private final Pigeon2 pigeon;
  // private final com.ctre.phoenix6.StatusSignal<Double> yawRateSignal;
  private final com.ctre.phoenix6.StatusSignal<Angle> yawSignal;

  /**
   * Creates a new Pigeon2Gyro.
   *
   * @param canId The CAN ID of the Pigeon 2.
   * @param canBus The CAN bus (e.g., "rio")
   * @param reversed Whether the gyro is mounted upside down.
   */
  public Pigeon2Gyro(int canId, String canBus, boolean reversed) {
    this.pigeon = new Pigeon2(canId, canBus);

    // Apply configuration
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.MountPoseYaw = 0;
    config.MountPose.MountPosePitch = 0;
    config.MountPose.MountPoseRoll = 0;
    // Set 'reversed' based on constant
    // config.MountPose.InvertMountPose = reversed;
    pigeon.getConfigurator().apply(config);

    // Set signals for efficient periodic reading
    // We get yaw from FusedHeading
    this.yawSignal = pigeon.getYaw();
    // Yaw rate is from GyroZ
    // this.yawRateSignal = pigeon.getY

    // Optimize update frequency
    this.yawSignal.setUpdateFrequency(100); // 100 Hz
    // this.yawRateSignal.setUpdateFrequency(100); // 100 Hz

    // Reset yaw on startup
    resetYaw();
  }

  @Override
  public Rotation2d getYaw() {
    // Refresh and get the latest value
    // WPILib convention is CCW positive. Pigeon 2 (fused) is CCW positive.
    yawSignal.refresh();
    return Rotation2d.fromDegrees(yawSignal.getValueAsDouble());
  }

  @Override
  public double getYawRateRadPerSec() {
    // // Refresh and get the latest value
    // // GyroZ is in deg/sec, convert to rad/sec
    // do not use this !
    yawSignal.refresh();
    return yawSignal.getValueAsDouble();
  }

  @Override
  public void resetYaw() {
    // Set the fused heading to 0
    pigeon.setYaw(0.0);
  }
}
