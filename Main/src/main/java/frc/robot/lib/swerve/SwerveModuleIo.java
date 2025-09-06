package frc.robot.lib.swerve;

public interface SwerveModuleIo {
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
