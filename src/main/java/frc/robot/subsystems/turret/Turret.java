package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.DashboardState;
import frc.robot.PoseHistory;
import frc.robot.framework.CSVLogWriter;
import frc.robot.framework.ILoop;
import frc.robot.framework.Looper;
import frc.robot.framework.Subsystem;
import frc.robot.framework.SysIdRoutine;

/** 航空航天級砲塔控制類 (Aerospace-Grade Turret Control) 集成 254 (預測)、2910 (解耦) 與 Ironclad (動態調優) 最佳實踐。 */
public class Turret extends Subsystem {

  private static Turret mInstance;

  public static Turret getInstance() {
    if (mInstance == null) {
      mInstance = new Turret();
    }
    return mInstance;
  }

  // --- 物理邊界 ---
  private static final double kMinAngle = Constants.Turret.kMinAngleRads;
  private static final double kMaxAngle = Constants.Turret.kMaxAngleRads;
  private static final double kTrackOverlapMargin = Constants.Turret.kTrackOverlapMarginRads;
  private static final double kTrackCenter = (kMinAngle + kMaxAngle) / 2.0;
  private static final double kTrackMin = kTrackCenter - Math.PI - kTrackOverlapMargin;
  private static final double kTrackMax = kTrackCenter + Math.PI + kTrackOverlapMargin;

  public enum ShootState {
    ACTIVE_SHOOTING,
    TRACKING,
    ROBOT_RELATIVE
  }

  // --- 硬體層 ---
  private final TurretIO mIO;
  private final TurretIO.TurretIOInputs mInputs = new TurretIO.TurretIOInputs();

  // --- 控制鏈 ---
  private TrapezoidProfile.State mSetpoint = new TrapezoidProfile.State(0.0, 0.0);
  private final TrapezoidProfile mProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              Constants.Turret.kMaxVelocityRadPerSec,
              Constants.Turret.kMaxAccelerationRadPerSecSq));

  private final SysIdRoutine mSysIdRoutine;

  private double mGoalAngleRadsField = 0.0;
  private double mLastGoalAngleRads = 0.0;
  private double mRobotRelativeGoalRads = 0.0;
  private double mLastVelocitySetpoint = 0.0;
  private ShootState mShootState = ShootState.TRACKING;
  private boolean mZeroed = false;
  private boolean mAtGoal = false;
  private boolean mOpenLoopMode = false;
  private boolean mSysIdActive = false;

  private Turret() {
    if (Constants.kHasTurret) {
      mIO = new TurretIOReal();
    } else {
      mIO = new TurretIO() {};
    }

    double crtAngle = mIO.initializeAbsolutePosition();

    Timer.delay(0.05);

    if (!Double.isNaN(crtAngle)) {
      mZeroed = true;
      mSetpoint = new TrapezoidProfile.State(crtAngle, 0.0);
      mLastGoalAngleRads = crtAngle;
    }

    mSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config()
                .setSubsystemName("Turret")
                .setRampRate(Volts.of(0.5))
                .setStepVoltage(Volts.of(2.0))
                .setTimeout(Seconds.of(10.0))
                .setLogWriter(new CSVLogWriter("sysid_Turret")));
  }

  @Override
  public void registerEnabledLoops(Looper enabledLooper) {
    enabledLooper.register(
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            synchronized (Turret.this) {
              mSetpoint = new TrapezoidProfile.State(getPositionRads(), 0.0);
              mLastGoalAngleRads = getPositionRads();
            }
          }

          @Override
          public void onLoop(double timestamp) {}

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  private Rotation2d getPredictedRobotHeading() {
    edu.wpi.first.math.geometry.Pose2d rawPose;
    if (frc.robot.subsystems.RobotStateEstimator.hasInstance()) {
      rawPose = frc.robot.subsystems.RobotStateEstimator.getInstance().getEstimatedPose();
    } else {
      rawPose = PoseHistory.getInstance().getLatestFieldToVehicle();
    }

    ChassisSpeeds velocity = PoseHistory.getInstance().getLatestFieldVelocity();
    Rotation2d currentHeading = rawPose.getRotation();

    double lookaheadTime = Constants.Turret.kLookaheadSeconds;

    return currentHeading.plus(
        Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lookaheadTime));
  }

  // removed duplicates

  private double selectBestAngle(double robotRelativeGoalRads) {
    double minLegal = (mShootState == ShootState.ACTIVE_SHOOTING) ? kMinAngle : kTrackMin;
    double maxLegal = (mShootState == ShootState.ACTIVE_SHOOTING) ? kMaxAngle : kTrackMax;

    double bestAngle = Double.NaN;
    double minDistance = Double.MAX_VALUE;

    for (int i = -1; i <= 1; i++) {
      double candidate = robotRelativeGoalRads + Math.PI * 2.0 * i;
      if (candidate >= minLegal && candidate <= maxLegal) {
        double dist = Math.abs(candidate - mLastGoalAngleRads);
        if (dist < minDistance) {
          minDistance = dist;
          bestAngle = candidate;
        }
      }
    }
    return Double.isNaN(bestAngle)
        ? MathUtil.clamp(robotRelativeGoalRads, kMinAngle, kMaxAngle)
        : bestAngle;
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    if (mOpenLoopMode || DriverStation.isDisabled() || !mZeroed) {
      if (!mOpenLoopMode) mIO.stop();
      mAtGoal = false;
      return;
    }

    Rotation2d predictedHeading = getPredictedRobotHeading();
    ChassisSpeeds chassisSpeeds = PoseHistory.getInstance().getLatestFieldVelocity();
    double chassisOmega = chassisSpeeds.omegaRadiansPerSecond;

    var shotParams = frc.robot.GlobalData.currentShotParams;

    double robotRelativeGoal;
    double feedforwardOmega = 0.0;

    if (mShootState == ShootState.ROBOT_RELATIVE) {
      robotRelativeGoal = mRobotRelativeGoalRads;
      feedforwardOmega = 0.0;
    } else {
      robotRelativeGoal =
          MathUtil.angleModulus(mGoalAngleRadsField - predictedHeading.getRadians());
      feedforwardOmega = -chassisOmega;

      if (mShootState == ShootState.ACTIVE_SHOOTING && shotParams != null && shotParams.hasTarget) {
        // Feedforward completely decouples robot frame rotation AND adds physical
        // target lateral translation angle rate
        feedforwardOmega += shotParams.turretVelocityRadPerSec;
      }
    }

    double targetAngle = selectBestAngle(robotRelativeGoal);
    mLastGoalAngleRads = targetAngle;

    mSetpoint =
        mProfile.calculate(
            Constants.kLooperDt,
            mSetpoint,
            new TrapezoidProfile.State(targetAngle, feedforwardOmega));

    double currentPos = getPositionRads();
    double velocitySetpoint = mSetpoint.velocity;
    double velocityRotPerSec = velocitySetpoint / (2 * Math.PI);

    // Motor hardware controller handles kP, kD, kS, kV based on the velocity
    // setpoint at 1000Hz.
    // We only need to provide the acceleration feedforward (kA) explicitly as
    // arbitrary voltage
    // since we do the profiling on the RoboRIO.
    double accelerationRotPerSecSq =
        (velocityRotPerSec - mLastVelocitySetpoint) / Constants.kLooperDt;
    double ffVolts = accelerationRotPerSecSq * Constants.Turret.kA;
    mLastVelocitySetpoint = velocityRotPerSec;

    // By passing velocitySetpoint != 0.0, the hardware D-term correctly tracks
    // error derivative,
    // eliminating viscous drag.
    mIO.setPositionSetpoint(mSetpoint.position, velocitySetpoint, ffVolts);

    double error = Math.abs(targetAngle - currentPos);

    double exitTolerance = Constants.Turret.kPositionToleranceRadsExit;

    if (shotParams != null && shotParams.isValid && shotParams.hasTarget && !shotParams.isPassing) {
      edu.wpi.first.math.geometry.Pose2d rawPose;
      if (frc.robot.subsystems.RobotStateEstimator.hasInstance()) {
        rawPose = frc.robot.subsystems.RobotStateEstimator.getInstance().getEstimatedPose();
      } else {
        rawPose = PoseHistory.getInstance().getLatestFieldToVehicle();
      }

      double estX = rawPose.getX();
      double estY = rawPose.getY();
      double estTheta = rawPose.getRotation().getRadians();

      double rtX = frc.robot.Constants.Shot.kRobotToTurret.getX();
      double rtY = frc.robot.Constants.Shot.kRobotToTurret.getY();

      double cosTheta = Math.cos(estTheta);
      double sinTheta = Math.sin(estTheta);

      double turretX = estX + rtX * cosTheta - rtY * sinTheta;
      double turretY = estY + rtX * sinTheta + rtY * cosTheta;

      double targetX = frc.robot.Constants.getHubCenter().getX();
      double targetY = frc.robot.Constants.getHubCenter().getY();

      double distanceL = Math.hypot(targetX - turretX, targetY - turretY);

      if (distanceL > 0.5) {

        double descentAngleFactor = Math.abs(Math.sin(shotParams.descentAngleRads));
        double theoreticalAllowance = Math.atan(0.454971 / distanceL);
        double trueHubAllowance = theoreticalAllowance * descentAngleFactor;

        exitTolerance = Math.min(trueHubAllowance, Constants.Turret.kPositionToleranceRadsExit);
      }
    }

    // While the turret is slewing fast the position may momentarily cross the
    // tolerance band, but the ball would miss because the turret keeps moving
    // during time-of-flight. Gate on turret angular velocity to prevent this.
    // boolean turretSettled =
    // Math.abs(mInputs.velocityRadsPerSec) <
    // Constants.Turret.kVelocityToleranceRadPerSec;

    if (mAtGoal) {
      mAtGoal = error < exitTolerance;
    } else {
      mAtGoal = error < Constants.Turret.kPositionToleranceRadsEnter;
    }
  }

  // --- Subsystem Overrides & Core API ---
  public synchronized void setFieldRelativeTarget(double angleRads) {
    mGoalAngleRadsField = angleRads;
    mOpenLoopMode = false;
  }

  public synchronized void setTargetAngle(double angleRads) {
    setFieldRelativeTarget(angleRads);
  }

  public synchronized void setRobotRelativeTarget(Rotation2d angle) {
    mRobotRelativeGoalRads = angle.getRadians();
    mShootState = ShootState.ROBOT_RELATIVE;
    mOpenLoopMode = false;
  }

  public synchronized void setShootState(ShootState state) {
    mShootState = state;
  }

  public synchronized void setOpenLoopVoltage(double volts) {
    mOpenLoopMode = true;
    mIO.setVoltage(volts);
  }

  public void startSysId(SysIdRoutine.TestType type, SysIdRoutine.Direction direction) {
    mSysIdActive = true;
    mOpenLoopMode = true;
    mSysIdRoutine.start(type, direction);
  }

  public void stopSysId() {
    if (mSysIdActive) {
      mSysIdRoutine.stop();
      mSysIdActive = false;
      mIO.setVoltage(0);
      mLastGoalAngleRads = getPositionRads();
      mGoalAngleRadsField = mLastGoalAngleRads;
      mOpenLoopMode = false;
    }
  }

  public void updateSysId() {
    if (mSysIdActive) {
      double timestamp = Timer.getFPGATimestamp();
      double positionRotations = mInputs.positionRads / (2 * Math.PI);
      double velocityRotPerSec = mInputs.velocityRadsPerSec / (2 * Math.PI);
      mSysIdRoutine.update(timestamp, positionRotations, velocityRotPerSec);
      mIO.setVoltage(mSysIdRoutine.getOutputVoltage());
    }
  }

  public boolean isSysIdActive() {
    return mSysIdActive;
  }

  public synchronized double getPositionRads() {
    return mInputs.positionRads;
  }

  public synchronized Rotation2d getAngle() {
    return Rotation2d.fromRadians(mInputs.positionRads);
  }

  public synchronized double getVelocityRadsPerSec() {
    return mInputs.velocityRadsPerSec;
  }

  public synchronized boolean isAtGoal() {
    return mAtGoal;
  }

  public synchronized boolean isZeroed() {
    return mZeroed;
  }

  public synchronized void zero() {
    mZeroed = true;
    mSetpoint = new TrapezoidProfile.State(getPositionRads(), 0.0);
    mLastGoalAngleRads = getPositionRads();
  }

  @Override
  public void travelOperate() {
    mShootState = ShootState.TRACKING;
    var params = frc.robot.GlobalData.currentShotParams;
    if (params != null && params.hasTarget) {
      setFieldRelativeTarget(params.turretAngleRad);
    }
  }

  @Override
  public void intakeOperate() {
    mShootState = ShootState.TRACKING;
    var params = frc.robot.GlobalData.currentShotParams;
    if (params != null && params.hasTarget) {
      setFieldRelativeTarget(params.turretAngleRad);
    }
  }

  @Override
  public void scoreOperate() {
    var params = frc.robot.GlobalData.currentShotParams;
    if (params != null && params.hasTarget) {
      setFieldRelativeTarget(params.turretAngleRad);
      mShootState = ShootState.ACTIVE_SHOOTING;
    } else {
      mShootState = ShootState.TRACKING;
    }
  }

  /** SCORE_LOCKED: 砲台維持當前角度不追蹤 shotParams，直接標記為 ACTIVE_SHOOTING。用於自動階段起始射擊，砲台已在正確方位時直射。 */
  public void scoreLockedOperate() {
    // 強制將目標設為當前位置，鎖定砲台。
    // 需要加上當前機器人朝向(predictedHeading)以抵消 writePeriodicOutputs 中的角度減法。
    mGoalAngleRadsField = getPositionRads() + getPredictedRobotHeading().getRadians();
    mShootState = ShootState.ACTIVE_SHOOTING;
  }

  @Override
  public void passOperate() {
    var params = frc.robot.GlobalData.currentShotParams;
    if (params != null && params.hasTarget) {
      setFieldRelativeTarget(params.turretAngleRad);
      mShootState = ShootState.ACTIVE_SHOOTING;
    } else {
      mShootState = ShootState.TRACKING;
    }
  }

  @Override
  public void climbOperate() {
    // Park turret at robot-relative -90° (right side) to clear the
    // climb mechanism path and avoid mechanical interference.
    setRobotRelativeTarget(Rotation2d.fromDegrees(-90.0));
  }

  @Override
  public void handleTestMode(frc.robot.ControlBoard control) {
    // 1. SysId 測試 (優先級最高，對應搖桿設定的形狀按鍵)
    boolean runSysId = false;
    if (control.getSysIdQuasistaticForward()) {
      startSysId(SysIdRoutine.TestType.QUASISTATIC, SysIdRoutine.Direction.FORWARD);
      runSysId = true;
    } else if (control.getSysIdQuasistaticReverse()) {
      startSysId(SysIdRoutine.TestType.QUASISTATIC, SysIdRoutine.Direction.REVERSE);
      runSysId = true;
    } else if (control.getSysIdDynamicForward()) {
      startSysId(SysIdRoutine.TestType.DYNAMIC, SysIdRoutine.Direction.FORWARD);
      runSysId = true;
    } else if (control.getSysIdDynamicReverse()) {
      startSysId(SysIdRoutine.TestType.DYNAMIC, SysIdRoutine.Direction.REVERSE);
      runSysId = true;
    } else {
      stopSysId();
    }

    if (runSysId) {
      updateSysId();
      return;
    }

    // 2. Voltage 手動開環控制 (對應板機/右搖桿，當有操控時直接給定電壓)
    double manualVolts = control.getRotation() * 12.0; // max 12V
    if (Math.abs(manualVolts) > 0.1) {
      setOpenLoopVoltage(manualVolts);
      return;
    }

    // 3. PID 角度歸零與測試 (利用按鈕)
    if (control.getClimbButton()) { // 對應 Options 按鍵，用來呼叫 zero()
      zero();
      setOpenLoopVoltage(0.0);
    } else if (control.getL1Button()) { // L1 切換 0 度
      setTargetAngle(Math.toRadians(0));
    } else if (control.getR1Button()) { // R1 切換 30 度
      setTargetAngle(Math.toRadians(-180));
    } else {
      setOpenLoopVoltage(0.0);
    }
  }

  @Override
  public void readPeriodicInputs() {
    mIO.updateInputs(mInputs);
  }

  @Override
  public void stop() {
    mIO.stop();
    mOpenLoopMode = true;
  }

  @Override
  public void zeroSensors() {
    zero();
  }

  @Override
  public boolean checkConnectionActive() {
    return mIO.getMotorFirmwareVersion() != 0;
  }

  @Override
  public boolean checkConnectionPassive() {
    return mInputs.motorConnected;
  }

  @Override
  public boolean checkSanityPassive() {
    double pos = mInputs.positionRads;
    return pos >= kMinAngle - 0.1 && pos <= kMaxAngle + 0.1;
  }

  @Override
  public void outputTelemetry() {
    DashboardState dashboardState = DashboardState.getInstance();
    dashboardState.turretAngle = Math.toDegrees(getPositionRads());
    dashboardState.turretOK = checkConnectionPassive() && checkSanityPassive();

    SmartDashboard.putNumber("Turret/PositionDeg", Math.toDegrees(getPositionRads()));
    SmartDashboard.putBoolean("Turret/Zeroed", mZeroed);
    SmartDashboard.putBoolean("Turret/AtGoal", mAtGoal);

    // --- CRT Calibration Tools ---
    SmartDashboard.putNumber("Turret/CRT/MotorEncoder_AbsRot", mInputs.motorEncoderAbsPosRotations);
    SmartDashboard.putNumber("Turret/CRT/AuxEncoder_AbsRot", mInputs.auxEncoderAbsPosRotations);

    // 我們需要的是純粹的物理絕對位置 [0, 1) 來作為 offset
    SmartDashboard.putNumber(
        "Turret/CRT/Suggested_MotorOffset", -mInputs.motorEncoderAbsPosRotations);
    SmartDashboard.putNumber("Turret/CRT/Suggested_AuxOffset", -mInputs.auxEncoderAbsPosRotations);

    SmartDashboard.putNumber(
        "Turret/CRT/Active_MotorOffset", Constants.Turret.kMotorEncoderOffsetRotations);
    SmartDashboard.putNumber(
        "Turret/CRT/Active_AuxOffset", Constants.Turret.kAuxEncoderOffsetRotations);
  }
}
