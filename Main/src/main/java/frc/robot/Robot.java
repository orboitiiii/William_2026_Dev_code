package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.lib.core.RobotIntentState;
import frc.robot.lib.core.RobotOperatingContext;
import frc.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {

  private final SubsystemManager subs = new SubsystemManager();
  public RobotOperatingContext robotContext;
  public RobotIntentState desired;

  @Override
  public void robotInit() {
    // Initialization logic if needed
  }

  @Override
  public void robotPeriodic() {
    // Update Sensors & Odometry (State Estimation)
    subs.periodic();

    // Read Inputs & Context (Perception)
    robotContext = subs.readContext();
  }

  @Override
  public void teleopInit() {
    desired = decideRobotState(robotContext);
  }

  @Override
  public void teleopPeriodic() {
    // Determine Intent (Planning)
    desired = decideRobotState(robotContext);

    // Execute Subsystems (Control)
    subs.operate(desired, robotContext);
  }

  @Override
  public void disabledInit() {
    desired = RobotIntentState.DISABLED;
  }

  @Override
  public void disabledPeriodic() {
    subs.operate(desired, robotContext);
  }

  public RobotIntentState decideRobotState(RobotOperatingContext ctx) {
    if (DriverStation.isDisabled()) return RobotIntentState.DISABLED;
    return RobotIntentState.TRAVEL;
  }
}
