package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public final class SwerveConfig {
  /** Control loop period (s). Typical FRC value is 0.02 s. */
  public final double dt;

  /** Wheelbase length (front-back distance, m). */
  public final double wheelBase;

  /** Track width (left-right distance, m). */
  public final double trackWidth;

  /** Max wheel linear speed (m/s). Used for normalizing module demands. */
  public final double maxWheelSpeed;

  /** Low-speed forward accel limit (m/s^2); decreases with speed. */
  public final double aMaxForward0;

  /** Lateral accel limit (m/s^2) to protect stability. */
  public final double aMaxLateral;

  /** Total accel limit (m/s^2) to mitigate wheel skid. */
  public final double aMaxSkid;

  /** Angular acceleration limit (rad/s^2). */
  public final double aMaxOmega;

  /** Drive feedforward: V = kS*sgn(v) + kV*v + kA*a. */
  public final double kS;

  public final double kV;
  public final double kA;

  /** Steering angle P (rad error -> motor controller). */
  public final double kPAngle;

  /** Drive speed P (m/s error -> additive volts). */
  public final double kPSpeed;

  public SwerveConfig(
      double dt,
      double wheelBase,
      double trackWidth,
      double maxWheelSpeed,
      double aMaxForward0,
      double aMaxLateral,
      double aMaxSkid,
      double aMaxOmega,
      double kS,
      double kV,
      double kA,
      double kPAngle,
      double kPSpeed) {
    this.dt = dt;
    this.wheelBase = wheelBase;
    this.trackWidth = trackWidth;
    this.maxWheelSpeed = maxWheelSpeed;
    this.aMaxForward0 = aMaxForward0;
    this.aMaxLateral = aMaxLateral;
    this.aMaxSkid = aMaxSkid;
    this.aMaxOmega = aMaxOmega;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kPAngle = kPAngle;
    this.kPSpeed = kPSpeed;
  }

  /** Returns module positions relative to robot center in meters (FL, FR, RL, RR). */
  public Translation2d[] moduleTranslations() {
    double l = wheelBase / 2.0;
    double w = trackWidth / 2.0;
    return new Translation2d[] {
      new Translation2d(+l, +w), // FL
      new Translation2d(+l, -w), // FR
      new Translation2d(-l, +w), // RL
      new Translation2d(-l, -w) // RR
    };
  }
}
