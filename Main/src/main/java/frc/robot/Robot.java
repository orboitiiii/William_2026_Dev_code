package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.framework.Looper;
import frc.robot.framework.SubsystemManager;
import frc.robot.subsystems.Drive;

public class Robot extends TimedRobot {
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  private final Drive mDrive = Drive.getInstance();
  private final DashboardState mDashboard = DashboardState.getInstance();

  public Robot() {
    // Register subsystems
    mSubsystemManager.setSubsystems(mDrive);

    // Register loops
    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    // mSubsystemManager.registerDisabledLoops(mDisabledLooper); // If implemented

    // Register SubsystemManager itself if needed, or just looper details
    mEnabledLooper.register(mSubsystemManager);
  }

  @Override
  public void robotInit() {
    // Zero sensors on start
    mSubsystemManager.getSubsystems().forEach(s -> s.zeroSensors());
  }

  @Override
  public void robotPeriodic() {
    // The Rigorous Cycle: Read -> Process (in Looper) -> Write
    mSubsystemManager.readPeriodicInputs();
    mSubsystemManager.outputTelemetry();
    mSubsystemManager.writePeriodicOutputs();

    // mEnabledLooper.outputToSmartDashboard(); // Removed in favor of single struct
    // RobotState.getInstance().outputToSmartDashboard(); // Removed in favor of single struct

    // Update Global Dashboard State
    mDashboard.matchTime = Timer.getMatchTime();
    mDashboard.isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // Publish All
    mDashboard.publish();
  }

  @Override
  public void teleopInit() {
    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStart(Timer.getFPGATimestamp());
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void autonomousInit() {
    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStart(Timer.getFPGATimestamp());
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void disabledInit() {
    mEnabledLooper.onStop(Timer.getFPGATimestamp());
    mDisabledLooper.onStart(Timer.getFPGATimestamp());

    mSubsystemManager.stop();
  }

  @Override
  public void disabledPeriodic() {
    // mSubsystemManager.readPeriodicInputs(); // Optional
  }
}
