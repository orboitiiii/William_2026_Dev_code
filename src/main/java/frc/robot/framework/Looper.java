package frc.robot.framework;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * High-frequency periodic loop executor.
 *
 * <p>The Looper runs registered {@link ILoop} instances at 100Hz using a WPILib {@link Notifier}.
 * This provides more deterministic timing than the TimedRobot's 50Hz periodic callbacks.
 *
 * <p><strong>Architecture</strong>:
 *
 * <pre>
 * Robot.java
 *    │
 *    ├── mEnabledLooper (100Hz during teleop/auto)
 *    │      └── Subsystem loops (Drive, Superstructure, etc.)
 *    │
 *    └── mDisabledLooper (100Hz during disabled)
 *           └── Diagnostic/calibration loops
 * </pre>
 *
 * <p><strong>Thread Safety</strong>: The Notifier runs in a separate thread. Access to shared state
 * must be synchronized. The {@code mTaskRunningLock} protects the loop list and running flag.
 *
 * <p><strong>Timing Specification</strong>: 100Hz = 10ms period.
 *
 * <p><strong>Attribution</strong>: Based on Team 254's Looper architecture.
 *
 * @see ILoop
 */
public class Looper implements ILoop {
  /** Loop period in seconds (100Hz = 0.01s). */
  public static final double kPeriod = 0.01;

  private boolean mRunning;

  private final Notifier mNotifier;
  private final List<ILoop> mLoops;
  private final Object mTaskRunningLock = new Object();
  private double mTimestamp = 0;

  double mDt = 0;

  /** Runnable executed by the Notifier thread. */
  private final Runnable mRunnable =
      new Runnable() {
        @Override
        public void run() {
          synchronized (mTaskRunningLock) {
            if (mRunning) {
              double now = Timer.getFPGATimestamp();

              for (ILoop loop : mLoops) {
                loop.onLoop(now);
              }

              mDt = now - mTimestamp;
              mTimestamp = now;
            }
          }
        }
      };

  /**
   * Constructs an empty Looper.
   *
   * <p>Use {@link #register(ILoop)} to add loops before starting.
   */
  public Looper() {
    mNotifier = new Notifier(mRunnable);
    mRunning = false;
    mLoops = new ArrayList<>();
  }

  @Override
  public synchronized void onStart(double timestamp) {
    if (!mRunning) {
      System.out.println("Starting loops");
      synchronized (mTaskRunningLock) {
        mTimestamp = timestamp;
        for (ILoop loop : mLoops) {
          loop.onStart(timestamp);
        }
        mRunning = true;
      }
      mNotifier.startPeriodic(kPeriod);
    }
  }

  @Override
  public synchronized void onLoop(double timestamp) {
    synchronized (mTaskRunningLock) {
      for (ILoop loop : mLoops) {
        loop.onLoop(timestamp);
      }
    }
  }

  @Override
  public synchronized void onStop(double timestamp) {
    if (mRunning) {
      System.out.println("Stopping loops");
      mNotifier.stop();
      synchronized (mTaskRunningLock) {
        mRunning = false;
        for (ILoop loop : mLoops) {
          System.out.println("Stopping " + loop);
          loop.onStop(timestamp);
        }
      }
    }
  }

  /**
   * Publishes loop timing to telemetry.
   *
   * <p>Exposes the measured dt for monitoring control loop performance.
   */
  public void outputToSmartDashboard() {
    // DashboardState.getInstance().looperDt = mDt;
  }

  /**
   * Registers a loop to be executed.
   *
   * <p>Should be called during robot initialization, before the looper starts.
   *
   * @param loop The loop to register.
   */
  public synchronized void register(ILoop loop) {
    synchronized (mTaskRunningLock) {
      mLoops.add(loop);
    }
  }
}
