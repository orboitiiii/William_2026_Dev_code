package frc.robot.subsystems.driveTrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.lib.swerve.SwerveModuleIo;
import frc.robot.lib.swerve.SwerveUtil;

public final class SparkMaxModuleIo implements SwerveModuleIo {
  private final SparkMax drive;
  private final SparkMax turn;

  private final RelativeEncoder driveEnc;
  private final SparkAbsoluteEncoder turnAbs;
  private final SparkClosedLoopController turnPid;

  private final double angleOffsetRad;
  private double prevDrivePosMeters = 0.0;

  /**
   * @param driveCanId
   * @param turnCanId
   * @param angleOffsetRad
   */
  public SparkMaxModuleIo(int driveCanId, int turnCanId, double angleOffsetRad) {

    this.drive = new SparkMax(driveCanId, MotorType.kBrushless);
    this.turn = new SparkMax(turnCanId, MotorType.kBrushless);

    drive.configure(
        SwerveConfigs.MAXSwerveModule.drivingConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    turn.configure(
        SwerveConfigs.MAXSwerveModule.turningConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.driveEnc = drive.getEncoder();
    this.turnPid = turn.getClosedLoopController();
    this.turnAbs = turn.getAbsoluteEncoder();

    this.angleOffsetRad = angleOffsetRad;

    this.prevDrivePosMeters = driveEnc.getPosition();

    driveEnc.setPosition(0);
  }

  @Override
  public double getSteerAngleRad() {
    double rawRad = turnAbs.getPosition();
    return SwerveUtil.wrap(rawRad - angleOffsetRad);
  }

  @Override
  public double getDriveVelocityMps() {
    return driveEnc.getVelocity();
  }

  @Override
  public double getDriveDeltaPosM() {
    double pos = driveEnc.getPosition();
    double delta = pos - prevDrivePosMeters;
    prevDrivePosMeters = pos;
    return delta;
  }

  @Override
  public void setSteerTargetAngleRad(double targetAngleRad) {
    double twoPi = 2.0 * Math.PI;
    double absTarget = targetAngleRad + angleOffsetRad;
    absTarget = absTarget % twoPi;
    if (absTarget < 0) absTarget += twoPi;

    turnPid.setReference(absTarget, ControlType.kPosition);
  }

  @Override
  public void setDriveVoltage(double volts) {
    drive.setVoltage(volts);
  }
}
