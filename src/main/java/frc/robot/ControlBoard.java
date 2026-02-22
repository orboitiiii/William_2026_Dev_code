package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;

/**
 * Operator control interface for the robot.
 *
 * <p>This singleton encapsulates all driver input processing, including:
 *
 * <ul>
 *   <li>Joystick deadband and response curve application
 *   <li>Coordinate frame transformation (controller → robot)
 *   <li>Button debouncing for discrete actions
 * </ul>
 *
 * <p><strong>Controller Mapping</strong>: Xbox/PS5 controller on USB port 0.
 *
 * <p><strong>Coordinate Convention</strong>:
 *
 * <ul>
 *   <li>Left Stick Y (forward) → Robot +X (forward)
 *   <li>Left Stick X (right) → Robot -Y (left is positive in FRC)
 *   <li>L2 Trigger (Unpressed → Pressed) → Robot +ω (Turn Left / CCW)
 *   <li>R2 Trigger (Unpressed → Pressed) → Robot -ω (Turn Right / CW)
 * </ul>
 */
public class ControlBoard {
  private static ControlBoard mInstance = null;

  /**
   * Returns the singleton instance.
   *
   * @return The global ControlBoard instance.
   */
  public static ControlBoard getInstance() {
    if (mInstance == null) {
      mInstance = new ControlBoard();
    }
    return mInstance;
  }

  private final PS5Controller mController;

  /** Deadband threshold to eliminate joystick drift. */
  private final double kDeadband = 0.1;

  private ControlBoard() {
    mController = new PS5Controller(0);
  }

  /**
   * Returns the desired translation vector from the left joystick.
   *
   * <p>Applies deadband and square response curve for fine control at low speeds.
   *
   * <p><strong>Output Range</strong>: [-1, 1] for both X and Y components. Scale by max velocity in
   * the caller.
   *
   * @return Translation2d with X = forward, Y = strafe left.
   */
  public Translation2d getTranslation() {
    // Controller: Up = -1, Down = +1; We want Forward = +X
    // Controller: Left = -1, Right = +1; We want Left = +Y
    double forward = -mController.getLeftY() * 0.75;
    double strafe = -mController.getLeftX() * 0.75;

    forward = MathUtil.applyDeadband(forward, kDeadband);
    strafe = MathUtil.applyDeadband(strafe, kDeadband);

    // Square curve preserves sign while reducing sensitivity at low inputs
    forward = Math.copySign(forward * forward, forward);
    strafe = Math.copySign(strafe * strafe, strafe);

    return new Translation2d(forward, strafe);
  }

  /**
   * Returns the desired rotation rate from the right joystick.
   *
   * <p>Applies deadband and square response curve.
   *
   * <p><strong>Output Range</strong>: [-1, 1]. Positive = CCW rotation.
   *
   * @return Normalized rotation command.
   */
  public double getRotation() {
    double leftInput = mController.getL2Axis();
    double rightInput = mController.getR2Axis();

    double rot = (leftInput - rightInput);
    rot = MathUtil.applyDeadband(rot, kDeadband);

    rot = Math.copySign(rot * rot, rot);

    rot = rot * 0.60;

    return rot;
  }

  /**
   * Returns true when the gyro zero button is pressed.
   *
   * <p>Uses A button. Resets the robot's heading to zero.
   *
   * @return True if gyro reset is requested.
   */
  public boolean getZeroGyro() {
    return mController.getTriangleButton();
  }

  // --- Test Mode Controls ---

  /**
   * Returns true on the rising edge of the calibrate button press.
   *
   * <p>Used in Test Mode to trigger swerve module zero-point calibration. In Test Mode, the A
   * button semantics change from "Zero Gyro" to "Calibrate Swerve".
   *
   * @return True on button press (edge-detected to prevent repeated triggers).
   */
  public boolean getCalibrateButton() {
    return mController.getCrossButtonPressed();
  }

  /**
   * Returns true while the drive motor test button is held.
   *
   * <p>Used in Test Mode to run all drive motors at a fixed duty cycle for diagnostic purposes
   * (verifying motor direction, encoder feedback, etc.).
   *
   * @return True while Y button is held.
   */
  public boolean getTestDriveButton() {
    return mController.getTriangleButton();
  }

  // --- Intake Controls ---

  /**
   * Returns true on the rising edge of R1 button press.
   *
   * <p>Used to toggle Intake operation. Edge-detected to prevent repeated triggers while button is
   * held.
   *
   * @return True on Circle button press (rising edge only).
   */
  public boolean getIntakeToggle() {
    return mController.getCircleButtonPressed();
  }

  // --- Intake Pivot Controls ---

  /**
   * Returns true while R1 button is held.
   *
   * <p>Used for Intake Pivot forward (positive) rotation. The pivot rotates forward as long as the
   * button is held.
   *
   * @return True while R1 is held.
   */
  public boolean getPivotForward() {
    return mController.getR1Button();
  }

  /**
   * Returns true while L1 button is held.
   *
   * <p>Used for Intake Pivot reverse (negative) rotation. The pivot rotates backward as long as the
   * button is held.
   *
   * @return True while L1 is held.
   */
  public boolean getPivotReverse() {
    return mController.getL1Button();
  }

  // --- Indexer Controls ---

  /**
   * Returns true while the Square button is held.
   *
   * <p>Used to run the Indexer. Both Side Roller and Straight Roller will spin while the button is
   * pressed.
   *
   * @return True while Square button is held.
   */
  public boolean getIndexerButton() {
    return mController.getSquareButton();
  }

  public boolean getSideRollerButton() {
    return mController.getL1Button();
  }

  public boolean getStraightRollerButton() {
    return mController.getR1Button();
  }

  // --- Shooter Controls ---

  /**
   * Returns true while the Cross button is held.
   *
   * <p>Used to run the Shooter. Both flywheels will spin while the button is pressed.
   *
   * @return True while Cross button is held.
   */
  public boolean getShooterButton() {
    return mController.getCrossButton();
  }

  // --- Odometry Characterizer Controls (Test Mode) ---

  /** Edge state for recording toggle. */
  private boolean mLastSquareStateRecording = false;

  /**
   * Returns true on the rising edge of Square button press.
   *
   * <p>Used in Test Mode to start/stop odometry recording.
   *
   * <p><strong>Note</strong>: This method conflicts with {@link #getIndexerButton()} in normal
   * operation. It should only be used in Test Mode where Indexer control is disabled.
   *
   * @return True on button press (edge-detected).
   */
  public boolean getRecordingToggle() {
    boolean current = mController.getSquareButton();
    boolean result = current && !mLastSquareStateRecording;
    mLastSquareStateRecording = current;
    return result;
  }

  /**
   * Returns true on the rising edge of Circle button press.
   *
   * <p>Used in Test Mode to end the current trial and reset odometry.
   *
   * @return True on button press (edge-detected).
   */
  public boolean getEndTrialButton() {
    return mController.getCircleButtonPressed();
  }

  /**
   * Returns true on the rising edge of Options button press.
   *
   * <p>Used in Test Mode to export all trial data to CSV.
   *
   * @return True on button press (edge-detected).
   */
  public boolean getExportButton() {
    return mController.getOptionsButtonPressed();
  }

  public boolean getScoreFuelButton() {
    return mController.getR1Button();
  }

  public boolean getPassButton() {
    return mController.getTriangleButton();
  }

  public boolean getClimbButton() {
    return mController.getOptionsButton();
  }

  public boolean getCalibrationModeButton() {
    return mController.getCreateButton();
  }

  public boolean getIntakeButton() {
    return mController.getCircleButton();
  }

  public boolean getSysIdQuasistaticForward() {
    return mController.getTriangleButton();
  }

  public boolean getSysIdQuasistaticReverse() {
    return mController.getCircleButton();
  }

  public boolean getSysIdDynamicForward() {
    return mController.getSquareButton();
  }

  public boolean getSysIdDynamicReverse() {
    return mController.getCrossButton();
  }

  // --- Convenience Button Accessors (for Test Mode SysId) ---

  public boolean getTriangleButton() {
    return mController.getTriangleButton();
  }

  public boolean getSquareButton() {
    return mController.getSquareButton();
  }

  public boolean getSquareButtonPressed() {
    return mController.getSquareButtonPressed();
  }

  public boolean getCircleButtonPressed() {
    return mController.getCircleButtonPressed();
  }

  public boolean getCrossButton() {
    return mController.getCrossButton();
  }

  public boolean getCircleButton() {
    return mController.getCircleButton();
  }

  public boolean getR1Button() {
    return mController.getR1Button();
  }

  public boolean getL1Button() {
    return mController.getL1Button();
  }

  public boolean getR1ButtonPressed() {
    return mController.getR1ButtonPressed();
  }
}
