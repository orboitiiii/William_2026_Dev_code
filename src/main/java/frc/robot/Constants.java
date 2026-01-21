package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.libraries.lib9427.utils.FlippingUtil;

/**
 * Robot-wide constants and physical parameters.
 *
 * <p>This class defines the "First Principles" of the robot's physical existence, including
 * dimensions, mass properties, and electromechanical limits.
 *
 * <p>All units are in the International System of Units (SI) (Meters, Kilograms, Seconds) unless
 * explicitly stated otherwise.
 *
 * <p><strong>Engineering Note:</strong> Constants in this file serve as the single source of truth
 * for the robot's mathematical model. Discrepancies between these values and physical reality will
 * result in control loop errors (e.g., Odometry drift).
 */
public final class Constants {

  /**
   * The fundamental control loop period in seconds.
   *
   * <p>Corresponds to a 50Hz update rate, the standard cycle time for the RoboRIO FPGA and CAN bus
   * utilization optimization.
   */
  public static final double kLooperDt = 0.02;

  public static final double FIELDLENGTH = Units.inchesToMeters(651.22);

  public static final double FIELDWIDTH = Units.inchesToMeters(317.69);

  /**
   * Gets the current alliance from the Driver Station, defaulting to Blue if not connected.
   *
   * <p><strong>Safety:</strong> Prevents null pointer exceptions or "Red-by-default" accidents when
   * DS is offline.
   *
   * @return The active Alliance (Blue if Optional is empty).
   */
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  /** The fixed Blue Alliance Hub Center pose. */
  private static final Pose2d BLUE_HUB_CENTER =
      new Pose2d(
          Units.inchesToMeters(182.11), Units.inchesToMeters(158.845), Rotation2d.fromDegrees(0.0));

  /**
   * Gets the center of the Hub (Speaker target) for the current alliance.
   *
   * <p><strong>Dynamic Addressing:</strong> Ensures correct targeting even if alliance changes
   * during runtime (e.g., practice matches).
   *
   * @return Pose2d of the Hub center.
   */
  public static Pose2d getHubCenter() {
    return getAlliance() == Alliance.Blue
        ? BLUE_HUB_CENTER
        : FlippingUtil.flipFieldPose(BLUE_HUB_CENTER);
  }

  /**
   * Checks if the robot is within the "Wing" / Alliance Zone.
   *
   * <p>The Zone Threshold is defined as X < 158.61 inches on the Blue side.
   *
   * <p><strong>Logic:</strong> Normalizes the input pose to the Blue Alliance perspective using
   * {@link FlippingUtil}. This ensures the check is mathematically identical regardless of actual
   * alliance color.
   *
   * @param pose The current robot pose.
   * @return true if the robot is within the alliance zone.
   */
  public static boolean isInAllianceZone(Pose2d pose) {
    Pose2d normalizedPose = pose;
    if (getAlliance() == Alliance.Red) {
      normalizedPose = FlippingUtil.flipFieldPose(pose);
    }
    return normalizedPose.getX() <= Units.inchesToMeters(158.61);
  }

  public static final double HUBHEIGHT = Units.inchesToMeters(72.00);

  /**
   * Constants specific to the Swerve Drive subsystem.
   *
   * <p>Includes geometry, kinematic constraints, and motor configurations for the SDS MK5N modules.
   */
  public static final class Swerve {

    // Dimensions

    /**
     * The center-to-center distance between the left and right modules.
     *
     * <p>Physical property derived from the CAD chassis frame.
     */
    public static final double kTrackWidth = Units.inchesToMeters(20.75);

    /**
     * The center-to-center distance between the front and back modules.
     *
     * <p>Physical property derived from the CAD chassis frame.
     */
    public static final double kWheelBase = Units.inchesToMeters(20.75);

    /**
     * The nominal radius of the drive wheels.
     *
     * <p>Based on standard 4-inch billet wheels. Note that tread wear effectively reduces this
     * radius over time, affecting odometry accuracy.
     */
    public static final double kWheelRadius = Units.inchesToMeters(2.0);

    // Module Locations (Center based)

    /**
     * Location of the Front-Left module relative to the robot center.
     *
     * <p>Format: (+X, +Y) in the body frame (Standard FRC coordinate system).
     */
    public static final Translation2d kFLPos =
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0);

    /**
     * Location of the Front-Right module relative to the robot center.
     *
     * <p>Format: (+X, -Y) in the body frame.
     */
    public static final Translation2d kFRPos =
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0);

    /**
     * Location of the Back-Left module relative to the robot center.
     *
     * <p>Format: (-X, +Y) in the body frame.
     */
    public static final Translation2d kBLPos =
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0);

    /**
     * Location of the Back-Right module relative to the robot center.
     *
     * <p>Format: (-X, -Y) in the body frame.
     */
    public static final Translation2d kBRPos =
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0);

    // Physical Limits (The First Principles)

    /**
     * The total mass of the robot in Kilograms.
     *
     * <p>Includes chassis, mechanism, battery, and bumpers.
     *
     * <p>Used for Force-to-Acceleration conversion (F = ma) in simulation and feedforward.
     */
    public static final double kRobotMass = 15.0;

    /**
     * The static Coefficient of Friction (CoF) between the wheels and the field carpet.
     *
     * <p>Determines the maximum achievable friction force before slipping.
     *
     * <p>Source: Empirical FRC community data for Blue Nitrile/Colson on standard FRC carpet.
     */
    public static final double kCoF = 0.90;

    /**
     * The maximum theoretical friction force the robot can exert on the floor.
     *
     * <p>Formula: F_fric = mu * N = kCoF * (mass * g)
     */
    public static final double kMaxFrictionForce = kRobotMass * 9.81 * kCoF;

    /**
     * The maximum linear acceleration the robot can achieve without wheel slip.
     *
     * <p>Formula: a_max = F_fric / mass
     *
     * <p>Value represents the physical traction limit (~10.8 m/s^2).
     */
    public static final double kMaxDriveAcceleration = kMaxFrictionForce / kRobotMass;

    // Motor Limits (Falcon 500 / Kraken)

    /**
     * The maximum linear velocity of the robot in meters per second.
     *
     * <p>Derived from motor free speed and gear ratio, reduced by efficiency losses.
     */
    public static final double kMaxDriveVelocity = 4.5;

    /**
     * The maximum angular velocity of the robot in radians per second.
     *
     * <p>Theoretical limit based on differential drive kinematics.
     */
    public static final double kMaxAngularVelocity = 10.0;

    // Gear Ratios

    /**
     * Configuration profiles for Swerve Drive Specialties (SDS) MK5N modules.
     *
     * <p>Defines the mechanical reduction ratios for drive and steer motors.
     *
     * @see <a href="https://www.swervedrivespecialties.com/">SDS Documentation</a>
     */
    public static enum SwerveModuleType {
      /** L1 Ratio: Drive 8.14:1, Steer 26.09:1. */
      MK5N_L1(8.14, 287.0 / 11.0),
      /** L2 Ratio: Drive 6.75:1, Steer 26.09:1. */
      MK5N_L2(6.75, 287.0 / 11.0),
      /** L3 Ratio: Drive 6.12:1, Steer 26.09:1. High speed configuration. */
      MK5N_L3(6.12, 287.0 / 11.0);

      /** Mechanical reduction ratio from motor rotation to wheel rotation. */
      public final double driveRatio;

      /** Mechanical reduction ratio from motor rotation to module azimuth rotation. */
      public final double steerRatio;

      SwerveModuleType(double drive, double steer) {
        this.driveRatio = drive;
        this.steerRatio = steer;
      }
    }

    // Active Configuration

    /** The selected swerve module configuration for this robot. */
    public static final SwerveModuleType kModuleType = SwerveModuleType.MK5N_L3;

    /** Drive gear ratio derived from the selected module type. */
    public static final double kDriveGearRatio = kModuleType.driveRatio;

    /** Steer gear ratio derived from the selected module type. */
    public static final double kSteerGearRatio = kModuleType.steerRatio;

    // --- Motor Models for Physics Simulation & Control ---

    /**
     * Kraken X60 Motor Model.
     *
     * <p>Specs: 12V, 7.09Nm Stall Torque, 366A Stall Current, 6000 RPM Free Speed.
     *
     * <p>Source: WestCoastProducts 2024/2025 Specs.
     */
    public static final DCMotor kDriveGearbox =
        new DCMotor(12.0, 7.09, 366.0, 0.4, 6000.0, 1).withReduction(kDriveGearRatio);

    /**
     * Kraken X44 Motor Model.
     *
     * <p>Specs: 12V, 3.82Nm Stall Torque, 236A Stall Current, 7200 RPM Free Speed.
     *
     * <p>Source: WestCoastProducts Pre-release Specs.
     */
    public static final DCMotor kSteerGearbox =
        new DCMotor(12.0, 3.82, 236.0, 0.5, 7200.0, 1).withReduction(kSteerGearRatio);

    // CAN IDs & Offsets

    // Front Left Module
    /** CAN ID for Front-Left Drive Motor. */
    public static final int kFLDriveId = 1;

    /** CAN ID for Front-Left Steer Motor. */
    public static final int kFLSteerId = 2;

    /** CAN ID for Front-Left Absolute Encoder (CANcoder). */
    public static final int kFLEncoderId = 1;

    /**
     * Magnet offset for Front-Left absolute encoder in rotations.
     *
     * <p>Aligns the wheel to zero degrees (straight forward) relative to the chassis.
     */
    public static final double kFLOffset = 0.20043945;

    // Front Right Module
    /** CAN ID for Front-Right Drive Motor. */
    public static final int kFRDriveId = 7;

    /** CAN ID for Front-Right Steer Motor. */
    public static final int kFRSteerId = 8;

    /** CAN ID for Front-Right Absolute Encoder (CANcoder). */
    public static final int kFREncoderId = 4;

    /** Magnet offset for Front-Right absolute encoder per Swerve Generator. */
    public static final double kFROffset = 0.30126953;

    // Back Left Module
    /** CAN ID for Back-Left Drive Motor. */
    public static final int kBLDriveId = 3;

    /** CAN ID for Back-Left Steer Motor. */
    public static final int kBLSteerId = 4;

    /** CAN ID for Back-Left Absolute Encoder (CANcoder). */
    public static final int kBLEncoderId = 2;

    /** Magnet offset for Back-Left absolute encoder per Swerve Generator. */
    public static final double kBLOffset = 0.22753906;

    // Back Right Module
    /** CAN ID for Back-Right Drive Motor. */
    public static final int kBRDriveId = 5;

    /** CAN ID for Back-Right Steer Motor. */
    public static final int kBRSteerId = 6;

    /** CAN ID for Back-Right Absolute Encoder (CANcoder). */
    public static final int kBREncoderId = 3;

    /** Magnet offset for Back-Right absolute encoder per Swerve Generator. */
    public static final double kBROffset = -0.44799805;

    /** CAN ID for the Pigeon 2.0 IMU. */
    public static final int kPigeonId = 15;

    /**
     * Closed-loop control constants for the Swerve Drive system.
     *
     * <p>Contains PID gains, Feedforward constants, and Current Limits.
     */
    public static final class Control {
      // Helper for conversions

      /** The linear distance modeled for one rotation of the wheel. Used for conversions. */
      public static final double kWheelCircumference = 2 * Math.PI * kWheelRadius;

      /**
       * Drive position coefficient for odometry correction.
       *
       * <p>Compensates for systematic errors in wheel odometry such as:
       *
       * <ul>
       *   <li>Wheel diameter manufacturing tolerance
       *   <li>Tire compression under load
       *   <li>Gear ratio measurement error
       * </ul>
       *
       * <p>Calibration: Drive the robot a known distance (e.g., 5m), then: {@code
       * kDrivePositionCoefficient = actual_distance / reported_distance}
       *
       * <p>Default: 1.0 (no correction)
       */
      public static final double kDrivePositionCoefficient = 1.16935483871;

      // --- Drive Motor Gains ---

      /**
       * Drive Motor Proportional Gain (kP).
       *
       * <p>Units: DutyCycle / (Rotations/s error). Adapted for Phoenix 6 usage from SysId analysis.
       *
       * <p>SysId Raw Result: kP = 0.0021
       */
      public static final double kDrivekP = 0.0021 * kWheelCircumference;

      /**
       * Drive Motor Integral Gain (kI).
       *
       * <p>SysId Raw Result: kI = 0.9015
       */
      public static final double kDrivekI = 0.0;

      /** Drive Motor Derivative Gain (kD). */
      public static final double kDrivekD = 0.0;

      /**
       * Drive Motor Velocity Feedforward Gain (kV).
       *
       * <p>Units: Voltage / (Meters/s).
       *
       * <p>SysId Raw Result: kV = 2.2096 V/(m/s)
       */
      public static final double kDrivekV = 2.2096 * kWheelCircumference;

      /** Drive Motor Supply Current Limit in Amps. Limits current to prevent brownouts. */
      public static final double kDriveSupplyCurrentLimit = 60.0;

      /** Enable state for Drive Motor Supply Current Limit. */
      public static final boolean kDriveSupplyCurrentLimitEnable = true;

      /**
       * Drive Motor Stator Current Limit in Amps.
       *
       * <p>Limits motor torque output to prevent wheel slip. Team 254 uses 80A as the slip
       * threshold. This is more responsive than supply current limiting for traction control.
       */
      public static final double kDriveStatorCurrentLimit = 80.0;

      /** Enable state for Drive Motor Stator Current Limit. */
      public static final boolean kDriveStatorCurrentLimitEnable = true;

      /**
       * Closed-loop ramp period in seconds.
       *
       * <p>Limits the rate of voltage/torque change during closed-loop control. This prevents
       * initialization overshoot by limiting dV/dt during startup.
       *
       * <p>First Principles: Instantaneous torque changes cause wheel slip, encoder miscounts, and
       * odometry drift. A 20ms ramp (50Hz bandwidth) matches the control loop frequency while
       * allowing responsive operation.
       *
       * <p>Source: Team 254 CompTunerConstants uses 0.01-0.02s ramps for all motors.
       */
      public static final double kClosedLoopRampPeriod = 0.02;

      /**
       * Simulation loop period in seconds.
       *
       * <p>This constant defines the time step for physics simulation in a simulated environment.
       * It should match the period of the main robot loop for accurate simulation.
       */
      public static final double kSimLoopPeriod = 0.02;

      // --- Steer Motor Gains ---

      /**
       * Steer Motor Proportional Gain (kP).
       *
       * <p>Derived from Position LQR analysis for optimal tracking.
       *
       * <p>Raw LQR Result: kP = 2.7330 (Adjusted to 7.0 for response tuning).
       *
       * <p>Converted for Phoenix 6 (Rotations based).
       */
      public static final double kSteerkP = 11.0000 * (2 * Math.PI);

      /** Steer Motor Integral Gain (kI). Raw LQR Result: kI = 1.5786. */
      public static final double kSteerkI = 0.0000 * (2 * Math.PI);

      /** Steer Motor Derivative Gain (kD). Raw LQR Result: kD = 0.0007. */
      public static final double kSteerkD = 0.0000 * (2 * Math.PI);

      /** Steer Motor Supply Current Limit in Amps. */
      public static final double kSteerSupplyCurrentLimit = 40.0;

      /** Enable state for Steer Motor Supply Current Limit. */
      public static final boolean kSteerSupplyCurrentLimitEnable = true;
    }
  }

  /**
   * Constants for the Intake Wheel subsystem.
   *
   * <p>Defines motor configuration and control parameters for the single-motor intake roller.
   */
  public static final class Intake {
    /** CAN ID for the intake wheel motor (TalonFX). */
    public static final int kIntakeMotorId = 20;

    /**
     * Voltage output for intake operation.
     *
     * <p>Using voltage control instead of duty cycle ensures consistent motor output regardless of
     * battery voltage fluctuations. From First Principles: at fixed voltage, the motor produces
     * consistent torque (V = IR -> I = V/R -> tau = kI).
     */
    public static final double kIntakeVoltage = -6.0;

    /** Supply current limit in Amps. Prevents motor stall damage and brownouts. */
    public static final double kSupplyCurrentLimit = 30.0;

    /** Enable supply current limiting. */
    public static final boolean kSupplyCurrentLimitEnable = true;

    /**
     * Whether the intake motor should be inverted.
     *
     * <p>Adjusted based on physical mounting orientation to ensure positive voltage results in game
     * piece intake.
     */
    public static final boolean kInverted = true;
  }

  /**
   * Constants for the Intake Pivot (Four-Bar Linkage) subsystem.
   *
   * <p>Defines motor configuration and control parameters for the dual-motor pivot mechanism.
   *
   * <p><strong>Motor Configuration</strong>:
   *
   * <ul>
   *   <li>Right Motor (ID 30): Leader, CCW positive.
   *   <li>Left Motor (ID 31): Follower, CW positive (inverted to oppose leader).
   * </ul>
   *
   * <p><strong>First Principles</strong>: In a four-bar linkage driven from both sides, both motors
   * must apply torque in the same "effective" rotational direction relative to the linkage. Since
   * the motors are physically mirrored, the left motor must spin opposite to the right motor.
   */
  public static final class IntakePivot {
    // --- CAN IDs ---
    /** CAN ID for the right (leader) pivot motor. */
    public static final int kRightMotorId = 30;

    /** CAN ID for the left (follower) pivot motor. */
    public static final int kLeftMotorId = 31;

    // --- Mechanical Ratios ---
    /**
     * Gear ratio from motor rotation to mechanism output.
     *
     * <p>Adjust this value based on actual gearbox and belt/chain reduction. Example: 36:1
     * planetary + 2:1 belt = 72:1 total.
     */
    public static final double kGearRatio = 36.0;

    // --- Position Control Gains (Slot 0) ---
    /**
     * Proportional gain for position control.
     *
     * <p>Units: Voltage / Rotation of error. Start conservative and tune on hardware.
     */
    public static final double kP = 50.0;

    /** Integral gain for position control. Typically 0 for position loops. */
    public static final double kI = 0.0;

    /** Derivative gain for position control. Provides damping. */
    public static final double kD = 1.0;

    /**
     * Static friction feedforward.
     *
     * <p>Units: Volts. Overcomes static friction before motion begins.
     */
    public static final double kS = 0.25;

    /**
     * Velocity feedforward.
     *
     * <p>Units: Volts / (Rotations/sec). Compensates for back-EMF at velocity.
     */
    public static final double kV = 0.12;

    /**
     * Gravity feedforward.
     *
     * <p>Units: Volts. Compensates for gravity torque at horizontal position. For arm mechanisms,
     * this should be tuned with the arm horizontal.
     */
    public static final double kG = 0.3;

    // --- Motion Magic Parameters ---
    /**
     * Cruise velocity for Motion Magic.
     *
     * <p>Units: Mechanism rotations per second.
     */
    public static final double kCruiseVelocity = 2.0;

    /**
     * Acceleration for Motion Magic.
     *
     * <p>Units: Mechanism rotations per second squared.
     */
    public static final double kAcceleration = 4.0;

    // --- Current Limits ---
    /** Supply current limit in Amps. Prevents brownouts. */
    public static final double kSupplyCurrentLimit = 40.0;

    /** Enable supply current limiting. */
    public static final boolean kSupplyCurrentLimitEnable = true;

    /** Stator current limit in Amps. Limits motor torque output. */
    public static final double kStatorCurrentLimit = 60.0;

    /** Enable stator current limiting. */
    public static final boolean kStatorCurrentLimitEnable = true;

    /**
     * Voltage output for manual control (R1/L1 buttons).
     *
     * <p>Used during open-loop jogging. Keep this value conservative to prevent mechanical damage.
     */
    public static final double kManualVoltage = 1.0;
  }

  /** Autonomous trajectory following parameters. */
  public static final class Auto {
    /**
     * Lookahead time for trajectory tracking.
     *
     * <p>First Principles: System response is not instantaneous. Targeting a future point (t + dt)
     * compensates for control loop latency and motor response time (approx 20ms).
     */
    public static final double kLookaheadTime = 0.02; // 20ms (next cycle)

    // X Controller Gains
    public static final double kXP = 0.0;
    public static final double kXI = 0.0;
    public static final double kXD = 0.0;

    // Y Controller Gains
    public static final double kYP = 0.0;
    public static final double kYI = 0.0;
    public static final double kYD = 0.0;

    // Theta Controller Gains
    public static final double kThetaP = 0.0;
    public static final double kThetaI = 0.0;
    public static final double kThetaD = 0.0;

    /** Position error tolerance in meters. */
    public static final double kPositionTolerance = 0.01;

    /** Heading error tolerance in radians (3 degrees). */
    public static final double kThetaTolerance = Math.toRadians(0.50);
  }
}
