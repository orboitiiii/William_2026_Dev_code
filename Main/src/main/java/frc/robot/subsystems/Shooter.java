package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.framework.requests.Request;

/**
 * Example Shooter Subsystem demonstrating the Fusion Architecture.
 *
 * <p>Implemented using: 1. 254's Loop/Subsystem pattern 2. Orbit's StateMachine logic (implied in
 * handleState) 3. 1678's Request system
 */
public class Shooter extends Subsystem {
  // Singleton Instance
  private static Shooter mInstance = null;

  public static Shooter getInstance() {
    if (mInstance == null) {
      mInstance = new Shooter();
    }
    return mInstance;
  }

  // System States
  public enum State {
    IDLE,
    SPINNING_UP,
    SHOOTING,
  }

  // Hardware / Inputs
  public static class PeriodicIO {
    // Inputs
    public double current_rpm;
    public boolean is_at_speed;

    // Outputs
    public double demand_rpm;
  }

  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private State mState = State.IDLE;
  private Request mCurrentRequest = null;

  private Shooter() {}

  @Override
  public void readPeriodicInputs() {
    // Mock hardware read
    // mPeriodicIO.current_rpm = mCANTalon.getVelocity();
  }

  @Override
  public void writePeriodicOutputs() {
    // Mock hardware write
    // mCANTalon.set(mPeriodicIO.demand_rpm);
  }

  @Override
  public void checkSystem() {
    // System diagnostic check
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putString("Shooter/State", mState.toString());
    SmartDashboard.putNumber("Shooter/RPM", mPeriodicIO.current_rpm);
  }

  @Override
  public void stop() {
    mPeriodicIO.demand_rpm = 0;
    mState = State.IDLE;
  }

  @Override
  public void zeroSensors() {
    // Zero encoders if needed
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            mState = State.IDLE;
          }

          @Override
          public void onLoop(double timestamp) {
            handleState();
          }

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  // --- Logic ---

  private void handleState() {
    switch (mState) {
      case IDLE:
        mPeriodicIO.demand_rpm = 0;
        break;
      case SPINNING_UP:
        mPeriodicIO.demand_rpm = 3000;
        // Transition logic
        // if (isAtSpeed()) mState = State.SHOOTING;
        break;
      case SHOOTING:
        mPeriodicIO.demand_rpm = 3000;
        break;
    }

    // Handle External Requests (1678 Style)
    if (mCurrentRequest != null) {
      mCurrentRequest.act();
      if (mCurrentRequest.isFinished()) {
        mCurrentRequest = null;
      }
    }
  }

  // --- Request API ---

  public void setOpenLoop(double rpm) {
    // Override state or set a manual request
  }
}
