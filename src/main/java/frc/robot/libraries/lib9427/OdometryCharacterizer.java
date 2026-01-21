package frc.robot.libraries.lib9427;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveIO;
import java.util.ArrayList;
import java.util.List;

/**
 * Odometry Error Characterization System (High-Performance DataLog Version).
 *
 * <p>Manages recording of odometry data during test runs using WPILib DataLogManager. This system
 * enables systematic characterization of odometry error sources with zero to minimal performance
 * impact on the control loop.
 *
 * <p><strong>DataLog Implementation</strong>:
 *
 * <ul>
 *   <li>Uses {@link DataLogManager} for async binary logging.
 *   <li>Uses {@link edu.wpi.first.util.struct.Struct} for efficient serialization (no string
 *       concat).
 *   <li>Eliminates GC pressure and loop overruns associated with CSV writing.
 * </ul>
 *
 * <p><strong>Usage Flow</strong>:
 *
 * <ol>
 *   <li>Position robot at marked origin (0, 0, 0)
 *   <li>Call {@link #startRecording()} to begin data collection (starts DataLog entry)
 *   <li>Call {@link #update} every control cycle (appends binary struct to log)
 *   <li>Drive robot through test maneuver and return to origin
 *   <li>Call {@link #endTrialAndReset(Pose2d)} to log error and finish entry
 *   <li>Data is automatically saved to USB/Flash as .wpilog
 * </ol>
 */
public class OdometryCharacterizer {

  private static OdometryCharacterizer mInstance;

  /**
   * Returns the singleton instance.
   *
   * @return The global OdometryCharacterizer instance.
   */
  public static OdometryCharacterizer getInstance() {
    if (mInstance == null) {
      mInstance = new OdometryCharacterizer();
    }
    return mInstance;
  }

  // DataLog Entries
  private StructLogEntry<OdometryDataPoint> mDataLogEntry;
  private StructLogEntry<OdometryError> mErrorLogEntry;
  private StringLogEntry mTrialEventLog;

  private final List<OdometryTrial> mTrials = new ArrayList<>();
  private OdometryTrial mCurrentTrial = null;
  private boolean mIsRecording = false;
  private int mNextTrialId = 1;

  // Previous values for derivative computation
  private double mPrevTimestamp = 0;
  private double mPrevOmega = 0;
  private double[] mPrevWheelDriveVel = new double[4];
  private double[] mPrevWheelSteerAngle = new double[4];
  private double[] mPrevWheelSteerVel = new double[4];

  // Setpoints cache
  private double[] mDriveSetpoints = new double[4];
  private double[] mSteerSetpoints = new double[4];

  private OdometryCharacterizer() {
    // Initialize DataLog (starts logging if not already started)
    DataLogManager.start();
    DataLog log = DataLogManager.getLog();

    // Define log entries with prefixes
    mDataLogEntry = StructLogEntry.create(log, "OdoChar/TimeSeries", OdometryDataPoint.struct);
    mErrorLogEntry = StructLogEntry.create(log, "OdoChar/TrialErrors", OdometryError.struct);
    mTrialEventLog = new StringLogEntry(log, "OdoChar/Events");
  }

  /** Starts recording a new trial. */
  public void startRecording() {
    mCurrentTrial = new OdometryTrial(mNextTrialId++);
    mIsRecording = true;
    mPrevTimestamp = 0;

    mTrialEventLog.append("START_TRIAL_" + mCurrentTrial.getTrialId());
    System.out.println("[OdoChar] Started recording trial " + mCurrentTrial.getTrialId());
  }

  /**
   * Updates the recording with current sensor data.
   *
   * @param inputs The drive subsystem inputs.
   * @param currentPose The current odometry pose.
   */
  public void update(DriveIO.DriveIOInputs inputs, Pose2d currentPose) {
    if (!mIsRecording || mCurrentTrial == null) {
      return;
    }

    double timestamp = inputs.timestamp;
    double dt = mPrevTimestamp > 0 ? timestamp - mPrevTimestamp : 0.02;
    if (dt <= 0) dt = 0.02; // Prevent divide by zero

    // Compute derived kinematic data (same logic as before)
    double vx = 0, vy = 0;
    if (mCurrentTrial.getDataPoints().size() > 0) {
      OdometryDataPoint prev =
          mCurrentTrial.getDataPoints().get(mCurrentTrial.getDataPoints().size() - 1);
      vx = (currentPose.getX() - prev.poseX()) / dt;
      vy = (currentPose.getY() - prev.poseY()) / dt;
    }

    double omega = inputs.gyroYawVelocityRadPerSec;
    double alpha = (omega - mPrevOmega) / dt;

    double[] wheelDriveVel = new double[4];
    double[] wheelDriveAccel = new double[4];
    double[] wheelSteerAngle = new double[4];
    double[] wheelSteerVel = new double[4];
    double[] wheelSteerAccel = new double[4];
    double[] wheelVx = new double[4];
    double[] wheelVy = new double[4];
    double[] driveError = new double[4];
    double[] steerError = new double[4];

    for (int i = 0; i < 4; i++) {
      wheelDriveVel[i] =
          inputs.driveVelocityRotationsPerSec[i] * Constants.Swerve.Control.kWheelCircumference;
      wheelDriveAccel[i] = (wheelDriveVel[i] - mPrevWheelDriveVel[i]) / dt;

      wheelSteerAngle[i] = inputs.steerPositionRotations[i] * 2 * Math.PI;
      wheelSteerVel[i] = (wheelSteerAngle[i] - mPrevWheelSteerAngle[i]) / dt;
      wheelSteerAccel[i] = (wheelSteerVel[i] - mPrevWheelSteerVel[i]) / dt;

      wheelVx[i] = wheelDriveVel[i] * Math.cos(wheelSteerAngle[i]);
      wheelVy[i] = wheelDriveVel[i] * Math.sin(wheelSteerAngle[i]);

      driveError[i] = mDriveSetpoints[i] - wheelDriveVel[i];
      steerError[i] = normalizeAngle(mSteerSetpoints[i] - wheelSteerAngle[i]);
    }

    // Build data point
    OdometryDataPoint point =
        OdometryDataPoint.builder()
            .timestamp(timestamp)
            .dtActual(dt)
            .pose(currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians())
            .robotVelocity(vx, vy, omega)
            .robotAcceleration(inputs.accelMetersPerSec2[0], inputs.accelMetersPerSec2[1], alpha)
            .wheelDriveVelocity(wheelDriveVel)
            .wheelDriveAccel(wheelDriveAccel)
            .wheelSteerAngle(wheelSteerAngle)
            .wheelSteerVelocity(wheelSteerVel)
            .wheelSteerAccel(wheelSteerAccel)
            .wheelVxVy(wheelVx, wheelVy)
            .electrical(
                RobotController.getBatteryVoltage(),
                inputs.driveCurrentAmps,
                inputs.steerCurrentAmps,
                inputs.driveSupplyVoltage)
            .controlSetpoints(mDriveSetpoints, mSteerSetpoints)
            .controlErrors(driveError, steerError)
            .computeDerivedMetrics()
            .build();

    // 1. Log directly to binary file (FAST)
    mDataLogEntry.append(point);

    // 2. Keep in memory if needed for immediate processing (optional, can be
    // removed for max perf)
    mCurrentTrial.addDataPoint(point);

    // Update previous values
    mPrevTimestamp = timestamp;
    mPrevOmega = omega;
    System.arraycopy(wheelDriveVel, 0, mPrevWheelDriveVel, 0, 4);
    System.arraycopy(wheelSteerAngle, 0, mPrevWheelSteerAngle, 0, 4);
    System.arraycopy(wheelSteerVel, 0, mPrevWheelSteerVel, 0, 4);
  }

  /**
   * Sets the current control setpoints for error tracking.
   *
   * @param driveSetpoints Drive velocity setpoints (m/s) per module.
   * @param steerSetpoints Steer angle setpoints (rad) per module.
   */
  public void setSetpoints(double[] driveSetpoints, double[] steerSetpoints) {
    System.arraycopy(driveSetpoints, 0, mDriveSetpoints, 0, 4);
    System.arraycopy(steerSetpoints, 0, mSteerSetpoints, 0, 4);
  }

  /**
   * Ends the current trial, records the error, and prepares for the next trial.
   *
   * @param finalPose The pose when returning to origin.
   * @return The recorded error.
   */
  public OdometryError endTrialAndReset(Pose2d finalPose) {
    if (mCurrentTrial == null) {
      return new OdometryError(0, 0, 0);
    }

    OdometryError error =
        new OdometryError(finalPose.getX(), finalPose.getY(), finalPose.getRotation().getRadians());

    // Log the error event
    mErrorLogEntry.append(error);
    mTrialEventLog.append(
        String.format(
            "END_TRIAL_%d,ERROR,%.4f,%.4f,%.4f",
            mCurrentTrial.getTrialId(), error.dx(), error.dy(), error.dTheta()));

    mCurrentTrial.setFinalError(error);
    mTrials.add(mCurrentTrial);

    System.out.println("[OdoChar] Trial " + mCurrentTrial.getTrialId() + " complete: " + error);

    mCurrentTrial = null;
    mIsRecording = false;

    return error;
  }

  /** Cancels the current recording. */
  public void cancelRecording() {
    if (mCurrentTrial != null) {
      mTrialEventLog.append("CANCEL_TRIAL_" + mCurrentTrial.getTrialId());
      System.out.println("[OdoChar] Cancelled trial " + mCurrentTrial.getTrialId());
    }
    mCurrentTrial = null;
    mIsRecording = false;
  }

  /**
   * No-op method for compatibility (DataLog saves automatically).
   *
   * @param baseFilename Ignored.
   */
  public void exportAllTrials(String baseFilename) {
    System.out.println(
        "[OdoChar] DataLog is active. Files are automatically saved to USB/Flash as .wpilog");
    mTrialEventLog.append("MARKER_EXPORT_REQUESTED");
  }

  /**
   * Returns whether recording is active.
   *
   * @return True if currently recording.
   */
  public boolean isRecording() {
    return mIsRecording;
  }

  private static double normalizeAngle(double angle) {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
  }
}
