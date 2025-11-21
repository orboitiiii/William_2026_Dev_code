package frc.robot.subsystems.driveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.core.RobotIntentState;
import frc.robot.lib.core.RobotOperatingContext;
import frc.robot.lib.core.StateDrivenSubsystem;
import frc.robot.lib.swerve.Gyro;
import frc.robot.lib.swerve.SwerveController;
import java.util.Optional;

public final class SwerveSubsystem extends StateDrivenSubsystem<SwerveSubsystem.State> {

  public enum State {
    OPEN_LOOP_TELEOP,
    X_LOCK,
    DISABLED
  }

  private final SwerveController controller;
  private final Gyro gyro;

  public SwerveSubsystem(SwerveController controller, Gyro gyro) {
    this.controller = controller;
    this.gyro = gyro;
  }

  public Pose2d getPose() {
    return controller.getEstimatedRobotPose();
  }

  public void updateOdometry(
      final Optional<Pose2d> visionPose,
      final Optional<Double> visionFom,
      final double avgTagAreaPercentage) { // You can use this to adjust FoM dynamically

    // Dynamic FoM logic (Optional: Calculate here before passing to controller)
    Optional<Double> finalFom = visionFom;
    if (visionPose.isPresent() && visionFom.isEmpty()) {
      // Simple heuristic example
      double heuristicFom = (avgTagAreaPercentage > 0.5) ? 0.1 : 0.9;
      finalFom = Optional.of(heuristicFom);
    }

    // Call the Fuse Update Function
    controller.updatePoseEstimation(
        gyro.getYaw(), gyro.getYawRateRadPerSec(), visionPose, finalFom);
  }

  @Override
  protected State transition(RobotIntentState desired, RobotOperatingContext ctx) {
    switch (desired) {
      case DISABLED:
        return State.DISABLED;
      case TRAVEL:
        return State.OPEN_LOOP_TELEOP;
      default:
        return State.DISABLED;
    }
  }

  @Override
  protected void handle(State state, RobotOperatingContext ctx) {
    switch (state) {
      case DISABLED:
        controller.stopAllModules();
        break;
      case X_LOCK:
        controller.engageXLock();
        break;
      case OPEN_LOOP_TELEOP:
        // Pure Control logic, no state estimation here (done in updateOdometry)
        controller.executeFieldRelativeCommand(
            ctx.driverChassisSpeeds,
            getPose().getRotation() // Use estimated rotation for field-relative
            );
        break;
    }
  }
}
