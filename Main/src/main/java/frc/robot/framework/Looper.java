package frc.robot.framework;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are
 * started when the robot powers up and stopped after the match.
 *
 * <p>Based on Team 254's Looper architecture.
 *
 * @author Team 254
 */
public class Looper implements ILoop {
  public static final double kPeriod = 0.01; // 100Hz

  private boolean mRunning;

  private final Notifier mNotifier;
  private final List<ILoop> mLoops;
  private final Object mTaskRunningLock = new Object();
  private double mTimestamp = 0;
  private double mDt = 0;

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

  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("looper_dt", mDt);
  }

  public synchronized void register(ILoop loop) {
    synchronized (mTaskRunningLock) {
      mLoops.add(loop);
    }
  }
}
