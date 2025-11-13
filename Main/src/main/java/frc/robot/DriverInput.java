package frc.robot;

/**
 * A struct-like class to hold all raw data from the driver controller. This object is created by
 * the SubsystemManager and used by Robot.java to determine the global RobotIntentState.
 */
public class DriverInput {

  // Stick inputs (already deadbanded and scaled)
  public final double driveVx; // Forward/Backward
  public final double driveVy; // Strafe Left/Right
  public final double driveOmega; // Rotate

  // Button inputs (raw)
  public final boolean isCrossPressed;
  public final boolean isCirclePressed;
  public final boolean isTrianglePressed;
  public final boolean isSquarePressed;

  // Add other buttons as needed (e.g., L1, R1, options)

  public DriverInput(
      double driveVx,
      double driveVy,
      double driveOmega,
      boolean isCrossPressed,
      boolean isCirclePressed,
      boolean isTrianglePressed,
      boolean isSquarePressed) {
    this.driveVx = driveVx;
    this.driveVy = driveVy;
    this.driveOmega = driveOmega;
    this.isCrossPressed = isCrossPressed;
    this.isCirclePressed = isCirclePressed;
    this.isTrianglePressed = isTrianglePressed;
    this.isSquarePressed = isSquarePressed;
  }

  /**
   * Helper method to determine if the driver is actively trying to drive.
   *
   * @return true if any drive stick is outside its deadband.
   */
  public boolean isDriving() {
    // We can check against a small epsilon, as the values are already deadbanded.
    return Math.abs(driveVx) > 1e-4 || Math.abs(driveVy) > 1e-4 || Math.abs(driveOmega) > 1e-4;
  }
}
