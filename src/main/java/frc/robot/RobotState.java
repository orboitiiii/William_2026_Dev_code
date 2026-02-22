package frc.robot;

/**
 * Global Robot State enum (Orbit 1690 Pattern).
 *
 * <p>Defines all high-level behavioral states the robot can be in. Every subsystem reads this state
 * via {@link GlobalData#robotState} and acts accordingly using per-state operate methods.
 *
 * <p>In teleop, the state is determined by {@code getRobotState()} based on joystick buttons. In
 * autonomous, the state is determined by the auto state machine.
 *
 * <p><strong>Priority Order</strong> (highest to lowest):
 *
 * <ol>
 *   <li>{@link #DISABLED} - Safety state
 *   <li>{@link #CALIBRATE} - Calibration mode
 *   <li>{@link #CLIMB} - Non-interruptible climbing
 *   <li>{@link #SCORE} - Shooting/scoring
 *   <li>{@link #PASS} - Passing to teammate
 *   <li>{@link #INTAKE} - Collecting game pieces
 *   <li>{@link #TRAVEL} - Default driving (lowest priority)
 * </ol>
 */
public enum RobotState {
  /** Default driving mode - no mechanisms active. */
  TRAVEL,

  /** Intake mode - collecting game pieces from the field. */
  INTAKE,

  /** Scoring mode - shooting game pieces at the target. */
  SCORE,

  /** Passing mode - passing game pieces to a teammate. */
  PASS,

  /** Score Test mode - fixed velocity shot test. */
  SCORE_TEST,

  /** Climbing mode - ascending the chain/cage. */
  CLIMB,

  /** Calibration mode - swerve module offset calibration. */
  CALIBRATE,

  /** Disabled mode - all mechanisms stopped. */
  DISABLED
}
