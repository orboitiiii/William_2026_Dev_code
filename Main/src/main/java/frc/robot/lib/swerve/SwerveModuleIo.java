package frc.robot.lib.swerve;

/**
 * Interface for a Swerve Module's IO (Input/Output). This abstracts the hardware (SparkMax, Falcon)
 * from the logical SwerveModule. Renamed to SwerveModuleIO for naming convention.
 */
public interface SwerveModuleIO {
  /**
   * @return Current module azimuth angle in radians (absolute, field-aligned zero).
   */
  double getSteerAngleRad();

  /**
   * @return Current wheel linear speed at tread (m/s).
   */
  double getDriveVelocityMps();

  /**
   * @return Wheel linear displacement since last call (m).
   */
  double getDriveDeltaPosM();

  /** Commands the steering angle setpoint (rad). Let the motor close the loop. */
  void setSteerTargetAngleRad(double targetAngleRad);

  /** Commands the drive motor voltage (V), typically feedforward + small P. */
  void setDriveVoltage(double volts);
}
