// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.lib.core.RobotContext;
import frc.robot.lib.core.RobotIntentState;
import frc.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {

  private final SubsystemManager subs = new SubsystemManager();
  public RobotContext robotContext;
  public RobotIntentState desired;

  public Robot() {}

  @Override
  public void robotInit() {}

  @Override
  public void driverStationConnected() {}

  @Override
  public void robotPeriodic() {
    robotContext = subs.readContext();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    desired = decideRobotState(robotContext);
  }

  @Override
  public void teleopPeriodic() {
    desired = decideRobotState(robotContext);
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

  @Override
  public void disabledExit() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public RobotIntentState decideRobotState(RobotContext ctx) {
    if (DriverStation.isDisabled()) return RobotIntentState.DISABLED;
    return RobotIntentState.TRAVAL;
  }
}
