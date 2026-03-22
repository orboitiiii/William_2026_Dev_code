package frc.robot;

import frc.robot.subsystems.shooter.ShotCalculator;

/**
 * Global shared data repository (Orbit 1690 Pattern).
 *
 * <p>Central location for all data shared across subsystems. Every subsystem reads from this class
 * to determine its behavior. The {@link RobotState} enum is the primary mechanism for coordinating
 * subsystem actions.
 *
 * <p><strong>Usage</strong>:
 *
 * <ul>
 *   <li>{@code GlobalData.robotState} is set once per cycle by {@code getRobotState()} or auto
 *       logic.
 *   <li>All subsystems read {@code GlobalData.robotState} in their {@code *Operate()} methods.
 *   <li>Shared sensor data (e.g., shot parameters) is updated before subsystem operate calls.
 * </ul>
 *
 * <p><strong>Thread Safety</strong>: Fields are written on the Looper thread and read on the same
 * thread, so no synchronization is needed for normal operation.
 */
public class GlobalData {
  private GlobalData() {} // Prevent instantiation

  // ============================================================
  // CORE STATE
  // ============================================================

  /** Current robot behavioral state. Set by getRobotState() or auto state machine. */
  public static volatile RobotState robotState = RobotState.TRAVEL;

  /** True when running in autonomous mode. */
  public static volatile boolean isAutonomous = false;

  /** True when running in test mode. */
  public static volatile boolean isTestMode = false;

  /**
   * Pending initial pose for autonomous mode. This is set in robotInit() and consumed by
   * RobotStateEstimator in the first valid 250Hz OdometrySnapshot cycle to ensure accurate module
   * angles for reset.
   */
  public static volatile edu.wpi.first.math.geometry.Pose2d pendingInitialPose = null;

  // ============================================================
  // GAME PIECE & INTAKE STATE
  // ============================================================

  /** True when the robot is holding a game piece. */
  public static volatile boolean hasGamePiece = true;

  /** True when the driver manually toggled the intake pivot down (R1). */
  public static volatile boolean pivotWantsDown = false;

  public enum IntakeActiveState {
    IDLE,
    FORWARD,
    REVERSE
  }

  /** Current wanted state of the Intake Wheel (Circle=Forward, Square=Reverse). */
  public static volatile IntakeActiveState intakeWheelWantedState = IntakeActiveState.IDLE;

  /**
   * Wanted voltage for the Intake Wheel during autonomous. Defaults to 6.0V, but can be overridden
   * by specific auto routines (e.g., set to 8.0V for shooting).
   */
  public static volatile double autoIntakeVoltage = 6.0;

  // ============================================================
  // HUB GAME STATE
  // ============================================================

  /** Updated each cycle locally indicating if the physical Hub is open for scoring. */
  public static volatile boolean isHubActive = false;

  /** Remaining seconds the Hub is in its current state. */
  public static volatile double hubActiveRemainingTime = 0.0;

  // ============================================================
  // SHOT CALCULATOR SHARED OUTPUT
  // ============================================================

  /**
   * Cached shooting parameters from ShotCalculator. Updated each cycle when robotState == SCORE.
   * Consumed by Shooter, Hood, and Turret in their scoreOperate() methods.
   */
  public static volatile ShotCalculator.ShootingParameters currentShotParams = null;

  // ============================================================
  // DRIVE CONSTRAINTS
  // ============================================================

  /** Speed limit factor for drive (0.0 to 1.0). */
  public static volatile double driveSpeedLimit = 1.0;

  /** Whether border/boundary protection is enabled. */
  public static volatile boolean borderProtectionEnabled = false;

  // ============================================================
  // KINEMATICS STATE
  // ============================================================

  /**
   * Current chassis linear speed in meters per second. Updated directly by the
   * single-source-of-truth RobotStateEstimator.
   */
  public static volatile double chassisSpeedMetersPerSec = 0.0;
}
