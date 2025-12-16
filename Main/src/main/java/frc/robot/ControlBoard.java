package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class ControlBoard {
  private static ControlBoard mInstance = null;

  public static ControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new ControlBoard();
    }
    return mInstance;
  }

  private final XboxController mController;
  private final double kDeadband = 0.1;

  private ControlBoard() {
    mController = new XboxController(0); // Port 0
  }

  public Translation2d getTranslation() {
    // PS5 Left Stick
    // Y is Forward (Standard FRC: Forward is +X)
    // X is Left/Right (Standard FRC: Left is +Y)
    // Controller: Push Up = -1.0, Push Down = 1.0 -> We want +X for Forward -> Invert Y
    // Controller: Push Left = -1.0, Push Right = 1.0 -> We want +Y for Left -> Invert X

    double forward = -mController.getLeftY();
    double strafe = -mController.getLeftX();

    forward = MathUtil.applyDeadband(forward, kDeadband);
    strafe = MathUtil.applyDeadband(strafe, kDeadband);

    // Optional: Apply Square curve for finer control
    forward = Math.copySign(forward * forward, forward);
    strafe = Math.copySign(strafe * strafe, strafe);

    return new Translation2d(forward, strafe);
  }

  public double getRotation() {
    // PS5 Right Stick X
    // Right is +1.0 -> We want CCW Positive (Left Turn) -> Invert
    double rot = -mController.getRightX();

    rot = MathUtil.applyDeadband(rot, kDeadband);

    // Square curve
    rot = Math.copySign(rot * rot, rot);

    return rot;
  }

  public boolean getZeroGyro() {
    return mController.getAButton(); // Options button to reset gyro
  }
}
