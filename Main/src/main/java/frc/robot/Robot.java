package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.framework.Looper;
import frc.robot.framework.SubsystemManager;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

  private final Shooter mShooter = Shooter.getInstance();
  private final Drive mDrive = Drive.getInstance();

  public Robot() {
    // Register subsystems
    mSubsystemManager.setSubsystems(mDrive, mShooter);

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

    // Looper is running in background (notifier), or we can run it here manually if we didn't use
    // Notifier.
    // Our Looper uses Notifier, so it runs parallel.
    // Note: 254 sometimes runs enabledLooper.onLoop() here for strictly synchronous behavior.
    // But our Looper.java uses a Notifier.
    // To strictly follow Read-Process-Write, we need to ensure thread safety or synchronization.
    // Standard 254: Read (here), Loop (Notifier), Write (here).

    mSubsystemManager.outputTelemetry();
    mSubsystemManager.writePeriodicOutputs();

    mEnabledLooper.outputToSmartDashboard();
    RobotState.getInstance().outputToSmartDashboard();
  }

  @Override
  public void teleopInit() {
    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStart(Timer.getFPGATimestamp());
  }

  @Override
  public void teleopPeriodic() {
    // Teleop specific code

    // Drive Control (Split Arcade / Field Relative)
    // Note: In a real robot, we would use a ControlBoard/Joystick class
    // For now, assume placeholders
    // mDrive.setTeleopInputs(-controller.getLeftY(), -controller.getLeftX(),
    // -controller.getRightX(), true);
  }

  @Override
  public void autonomousInit() {
    mDisabledLooper.onStop(Timer.getFPGATimestamp());
    mEnabledLooper.onStart(Timer.getFPGATimestamp());
  }

  @Override
  public void autonomousPeriodic() {
    // Auto logic
  }

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
