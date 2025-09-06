package frc.robot.subsystems.driveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.lib.core.RobotContext;
import frc.robot.lib.core.RobotIntentState;
import frc.robot.lib.core.SubsystemNode;
import frc.robot.lib.swerve.SwerveController;

/** Swerve subsystem implementing desired->system conversion and operate() dispatch. */
public final class SwerveSubsystem extends SubsystemNode<SwerveSubsystem.State> {
  public enum State {
    OPEN_LOOP_TELEOP,
    DRIVE_TO,
    X_LOCK,
    DISABLED
  }

  private final SwerveController ctrl;

  public SwerveSubsystem(SwerveController ctrl) {
    this.ctrl = ctrl;
  }

  public Pose2d getPose() {
    return ctrl.getRobotPose();
  }

  @Override
  protected State transition(RobotIntentState desired, RobotContext ctx) {
    switch (desired) {
      case DISABLED:
        return State.DISABLED;
      case TRAVAL:
      default:
        return State.OPEN_LOOP_TELEOP;
    }
  }

  @Override
  protected void handle(State state, RobotContext ctx) {
    switch (state) {
      case DISABLED:
        ctrl.stop();
        break;
      case X_LOCK:
        ctrl.holdXLock();
        break;
      case OPEN_LOOP_TELEOP:
      default:
        {
          ChassisSpeeds cmd = ctx.driverDesireSpeeds.get();
          ctrl.executeFieldCommand(cmd, ctx.robotPose.get().getRotation());
          break;
        }
    }
  }
}
