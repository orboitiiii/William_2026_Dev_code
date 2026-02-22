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

  // --- Mechanism Presence Flags ---
  // Set to false if the mechanism is physically removed from the robot.
  public static final boolean kHasHood = true;
  public static final boolean kHasTurret = true;
  public static final boolean kHasIntakeWheel = true;

  /** CANivore Bus Name (Empty = RoboRIO Default) */
  public static final String kCANBusName = "9427";

  /** Drivetrain CAN Bus Name (Empty = RoboRIO Default) */
  public static final String kDriveCANBusName = "";

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
    public static final double kRobotMass = 40.0;

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
    public static final int kFLDriveId = 7;

    /** CAN ID for Front-Left Steer Motor. */
    public static final int kFLSteerId = 8;

    /** CAN ID for Front-Left Absolute Encoder (CANcoder). */
    public static final int kFLEncoderId = 4;

    /**
     * Magnet offset for Front-Left absolute encoder in rotations.
     *
     * <p>Aligns the wheel to zero degrees (straight forward) relative to the chassis.
     */
    public static final double kFLOffset = 0.04418945;

    // Front Right Module
    /** CAN ID for Front-Right Drive Motor. */
    public static final int kFRDriveId = 5;

    /** CAN ID for Front-Right Steer Motor. */
    public static final int kFRSteerId = 6;

    /** CAN ID for Front-Right Absolute Encoder (CANcoder). */
    public static final int kFREncoderId = 3;

    /** Magnet offset for Front-Right absolute encoder per Swerve Generator. */
    public static final double kFROffset = 0.30468750;

    // Back Left Module
    /** CAN ID for Back-Left Drive Motor. */
    public static final int kBLDriveId = 1;

    /** CAN ID for Back-Left Steer Motor. */
    public static final int kBLSteerId = 2;

    /** CAN ID for Back-Left Absolute Encoder (CANcoder). */
    public static final int kBLEncoderId = 1;

    /** Magnet offset for Back-Left absolute encoder per Swerve Generator. */
    public static final double kBLOffset = -0.05322266;

    // Back Right Module
    /** CAN ID for Back-Right Drive Motor. */
    public static final int kBRDriveId = 3;

    /** CAN ID for Back-Right Steer Motor. */
    public static final int kBRSteerId = 4;

    /** CAN ID for Back-Right Absolute Encoder (CANcoder). */
    public static final int kBREncoderId = 2;

    /** Magnet offset for Back-Right absolute encoder per Swerve Generator. */
    public static final double kBROffset = -0.02783203;

    /** CAN ID for the Pigeon 2.0 IMU. */
    public static final int kPigeonId = 25;

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
    public static final int kIntakeMotorId = 30;

    /** CAN ID for the intake follower motor (TalonFX). */
    public static final int kIntakeFollowerId = 9;

    /**
     * Voltage output for intake operation.
     *
     * <p>Using voltage control instead of duty cycle ensures consistent motor output regardless of
     * battery voltage fluctuations. From First Principles: at fixed voltage, the motor produces
     * consistent torque (V = IR -> I = V/R -> tau = kI).
     */
    public static final double kIntakeVoltage = 3.0;

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
    public static final int kRightMotorId = 32;

    /** CAN ID for the left (follower) pivot motor. */
    public static final int kLeftMotorId = 31;

    // --- Mechanical Ratios ---
    /**
     * Gear ratio from motor rotation to mechanism output.
     *
     * <p>Adjust this value based on actual gearbox and belt/chain reduction. Value:
     * 6.666666666666667
     */
    public static final double kGearRatio = 6.666666666666667;

    /** Min Angle (Reverse Soft Limit) in degrees. */
    public static final double kMinAngle = 23.0;

    /** Max Angle (Forward Soft Limit) in degrees. */
    public static final double kMaxAngle = 90.0;

    // --- Position Control Gains (Slot 0) ---
    /**
     * Proportional gain for position control.
     *
     * <p>Units: Voltage / Rotation of error. Start conservative and tune on hardware.
     */
    public static final double kP = 24.0;

    /** Integral gain for position control. Typically 0 for position loops. */
    public static final double kI = 0.0;

    /** Derivative gain for position control. Provides damping. */
    public static final double kD = 0.00;

    /**
     * Static friction feedforward.
     *
     * <p>Units: Volts. Overcomes static friction before motion begins.
     */
    /**
     * Static friction feedforward.
     *
     * <p>Units: Volts. Overcomes static friction before motion begins. SysId Result (2026-02-13):
     * 0.15 V
     */
    public static final double kS = 0.15;

    /**
     * Velocity feedforward.
     *
     * <p>Units: Volts / (Rotations/sec). Compensates for back-EMF at velocity. SysId Result
     * (2026-02-13): 0.83 V/(rot/s)
     */
    public static final double kV = 0.83;

    /**
     * Acceleration feedforward.
     *
     * <p>Units: Volts / (Rotations/sec^2). Compensates for inertia during acceleration. SysId
     * Result (2026-02-13): Insufficient data, set to 0.0 for safety.
     */
    public static final double kA = 0.10;

    /**
     * Gravity Feedforward Map.
     *
     * <p>Format: {Angle in Degrees, kG Coefficient}. The kG value is interpolated based on angle,
     * then used in: FF = kG * cos(theta).
     */
    public static final double[][] kGravityMap = {
      {23.0, 0.95},
      {50.0, 1.00},
      {63.0, 1.00},
      {67.0, 1.50},
      {70.0, 1.15},
      {80.0, 2.05},
      {88.0, 2.05},
      {90.0, 1.55}
    };

    // --- Motion Magic Parameters ---
    /**
     * Cruise velocity for Motion Magic.
     *
     * <p>Units: Mechanism rotations per second. Set to 1.5 rot/s for safe operation.
     */
    /**
     * Cruise velocity for Motion Magic (UP Direction).
     *
     * <p>Units: Mechanism rotations per second. Fast up (8.0 rot/s).
     */
    public static final double kCruiseVelocityUp = 20.0;

    /**
     * Acceleration for Motion Magic (UP Direction).
     *
     * <p>Units: Mechanism rotations per second squared. Fast up (8.0 rot/s^2).
     */
    public static final double kAccelerationUp = 20.0;

    /**
     * Cruise velocity for Motion Magic (DOWN Direction).
     *
     * <p>Units: Mechanism rotations per second. Now matched to fast UP speed.
     */
    public static final double kCruiseVelocityDown = 20.0;

    /**
     * Acceleration for Motion Magic (DOWN Direction).
     *
     * <p>Units: Mechanism rotations per second squared. Now matched to fast UP accel.
     */
    public static final double kAccelerationDown = 20.0;

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

    /**
     * Extra downward voltage applied when the pivot is deployed and wheels are intaking.
     *
     * <p>Used to ensure the intake firmly presses against the floor/bumpers to pick up game pieces.
     * Positive voltage moves the pivot UP, so this value is subtracted from the output.
     */
    public static final double kIntakeDownPressureVolts = 0.50;
  }

  /**
   * Constants for the Indexer subsystem.
   *
   * <p>Defines motor configuration and control parameters for the dual-motor indexer mechanism.
   *
   * <p><strong>Motor Configuration</strong>:
   *
   * <ul>
   *   <li>Side Roller Motor: Transfers game pieces laterally.
   *   <li>Straight Roller Motor: Transfers game pieces in a linear path.
   * </ul>
   *
   * <p>Both motors use Krakon X44 (7200 RPM free speed, 3.82 Nm stall torque).
   */
  public static final class Indexer {
    // --- CAN IDs ---
    /** CAN ID for the side roller motor (Krakon X44). */
    public static final int kSideRollerMotorId = 35;

    /** CAN ID for the straight roller motor (Krakon X44). */
    public static final int kStraightRollerMotorId = 36;

    // --- Motor Inversion ---
    /**
     * Whether the side roller motor should be inverted.
     *
     * <p>Adjust based on physical mounting orientation to ensure positive voltage results in game
     * piece transfer towards the shooter.
     */
    public static final boolean kSideRollerInverted = false;

    /**
     * Whether the straight roller motor should be inverted.
     *
     * <p>Adjust based on physical mounting orientation.
     */
    public static final boolean kStraightRollerInverted = true;

    // --- Voltage Control (Legacy) ---
    /**
     * @deprecated Use Velocity Control instead.
     */
    @Deprecated public static final double kSideRollerVoltage = -3.0;

    /**
     * @deprecated Use Velocity Control instead.
     */
    @Deprecated public static final double kStraightRollerVoltage = -2.0;

    // --- Velocity Control ---
    public static final double kSideRollerTargetVelocity = -25.0;
    public static final double kStraightRollerTargetVelocity = -20.0;

    // Side Roller Gains (Placeholder - waiting for SysId)
    public static final double kSidekS = 0.0;
    public static final double kSidekV = 0.15; // Estimate
    public static final double kSidekA = 0.0;
    public static final double kSidekP = 0.0;

    // Straight Roller Gains (Placeholder - waiting for SysId)
    public static final double kStraightkS = 0.0;
    public static final double kStraightkV = 0.15; // Estimate
    public static final double kStraightkA = 0.0;
    public static final double kStraightkP = 0.0;

    // --- Current Limits ---
    /** Supply current limit in Amps per motor. Prevents motor stall damage and brownouts. */
    public static final double kSupplyCurrentLimit = 30.0;

    /** Enable supply current limiting. */
    public static final boolean kSupplyCurrentLimitEnable = true;
  }

  /**
   * Constants for the Shooter subsystem.
   *
   * <p>Defines motor configuration and control parameters for the dual-motor shooter mechanism.
   *
   * <p><strong>Motor Configuration</strong>:
   *
   * <ul>
   *   <li>Right Motor: Leader, Clockwise Positive (inverted).
   *   <li>Left Motor: Follower, Opposed alignment for mirrored counter-rotation.
   * </ul>
   *
   * <p><strong>First Principles</strong>: In a dual-flywheel shooter, both flywheels spin inward
   * toward the game piece. Since the motors face each other (mirrored mounting), the Opposed
   * follower mode inverts the follower voltage, achieving synchronized inward rotation.
   */
  public static final class Shooter {
    // --- CAN IDs ---
    /** CAN ID for the right (leader) shooter motor (KrakenX60). */
    public static final int kRightMotorId = 50;

    /** CAN ID for the left (follower) shooter motor (KrakenX60). */
    public static final int kLeftMotorId = 51;

    // --- Voltage Control ---
    /**
     * Voltage output for shooter operation.
     *
     * <p>Using voltage control instead of duty cycle ensures consistent motor output regardless of
     * battery voltage fluctuations.
     */
    public static final double kShooterVoltage = -10.0;

    // --- SysId Parameters (from characterization) ---
    // Date: 2026-02-13
    // Source: OLS regression on Quasistatic/Dynamic test data (SysId)

    /**
     * Static friction voltage (kS).
     *
     * <p>SysId Result (2026-02-15): 0.20460 V
     */
    public static final double kS = 0.20460;

    /**
     * Velocity gain (kV).
     *
     * <p>Units: Volts / (rotations per second). SysId Result (2026-02-15): 0.11442 V/(rot/s)
     */
    public static final double kV = 0.11442;

    /**
     * Acceleration gain (kA).
     *
     * <p>Units: Volts / (rotations per second squared). SysId Result (2026-02-15): 0.01998
     * V/(rot/s^2)
     */
    public static final double kA = 0.01998;

    /**
     * Target velocity for closed-loop speed control.
     *
     * <p>Derived from steady-state velocity at 4V during Dynamic test. Used for Cross button
     * velocity mode.
     */
    public static final double kTargetVelocity = 70.0; // rot/s

    /**
     * Proportional gain for velocity control.
     *
     * <p>Units: Volts / (rotations per second error). Provides feedback correction.
     */
    public static final double kP = 0.50;

    // --- Current Limits ---
    /** Supply current limit in Amps per motor. Prevents brownouts. */
    public static final double kSupplyCurrentLimit = 40.0;

    /** Enable supply current limiting. */
    public static final boolean kSupplyCurrentLimitEnable = true;

    /** Stator current limit in Amps per motor. Limits motor torque output. */
    public static final double kStatorCurrentLimit = 80.0;

    /** Enable stator current limiting. */
    public static final boolean kStatorCurrentLimitEnable = true;
  }

  /**
   * Constants for the Turret subsystem.
   *
   * <p>The turret is a continuous rotation mechanism with ±210° range, driven by a single KrakenX60
   * motor with an absolute encoder for position feedback.
   */
  public static final class Turret {
    /** CAN ID for the turret motor (KrakenX60). */
    public static final int kMotorId = 40;

    // --- CRT Absolute Positioning Gear & Encoder Parameters ---

    /** Number of teeth on the motor gear (meshes directly with 340T turret ring). */
    public static final int kMotorGearTeeth = 19;

    /**
     * Number of teeth on the auxiliary gear (meshes directly with 340T turret ring, used for CRT
     * positioning).
     */
    public static final int kAuxGearTeeth = 20;

    /** Number of teeth on the turret ring gear. */
    public static final int kTurretRingTeeth = 87;

    /** Motor shaft CANcoder CAN ID. */
    public static final int kMotorEncoderId = 41;

    /** Auxiliary shaft CANcoder CAN ID. */
    public static final int kAuxEncoderId = 42;

    /**
     * MagnetOffset for the motor shaft CANcoder (rotations).
     *
     * <p>Calibration: Rotate turret to zero (facing robot forward), read raw CANcoder value,
     * negated value is the offset.
     */
    public static final double kMotorEncoderOffsetRotations = -0.30517578125;

    /**
     * MagnetOffset for the auxiliary shaft CANcoder (rotations).
     *
     * <p>Calibration same as above.
     */
    public static final double kAuxEncoderOffsetRotations = -0.817626953125;

    /**
     * SensorToMechanismRatio: Motor Rotations / Turret Rotations.
     *
     * <p>Gear Ratio = 340T(Turret) / 41T(Motor Gear) ≈ 8.293
     */
    public static final double kGearRatio = (double) kTurretRingTeeth / kMotorGearTeeth;

    // --- Angle Limits ---

    /** Minimum turret angle in radians (CCW limit). */
    public static final double kMinAngleRads = Math.toRadians(-330);

    /** Maximum turret angle in radians (CW limit). */
    public static final double kMaxAngleRads = Math.toRadians(45);

    /**
     * Overlap margin for tracking mode.
     *
     * <p><strong>Physics Note</strong>: When tracking a target while the robot rotates, the turret
     * needs to temporarily exceed the ±180° range to maintain continuous tracking without reversing
     * direction.
     */
    public static final double kTrackOverlapMarginRads = Math.toRadians(10.0);

    // --- PID Gains ---

    /** Proportional gain for position control. */
    public static final double kP = 100.0;

    /** Integral gain for position control. */
    public static final double kI = 2.0;

    /** Derivative gain for position control. */
    public static final double kD = 0.5;

    /** Static friction compensation voltage. */
    public static final double kS = 0.80092;

    /** Velocity feedforward gain (V / (rot/s)). */
    public static final double kV = 0.88925;

    /** Acceleration feedforward gain (V / (rot/s^2)). */
    public static final double kA = 0.00476;

    // --- Motion Profile ---

    /** Maximum angular velocity in radians per second. */
    public static final double kMaxVelocityRadPerSec = Math.toRadians(360.0);

    /** Maximum angular acceleration in radians per second squared. */
    public static final double kMaxAccelerationRadPerSecSq = Math.toRadians(720.0);

    // --- Current Limits ---

    /** Supply current limit in Amps. */
    public static final double kSupplyCurrentLimit = 30.0;

    /** Stator current limit in Amps. */
    public static final double kStatorCurrentLimit = 60.0;

    // --- Tolerances ---

    /** Position tolerance for "at goal" detection in radians. */
    public static final double kPositionToleranceRads = Math.toRadians(2.0);

    /**
     * Velocity tolerance for "at goal" detection in rad/s. (Unused for shoot-on-the-move due to
     * high noise)
     */
    public static final double kVelocityToleranceRadPerSec = Math.toRadians(5.0);
  }

  /**
   * Constants for the Hood subsystem.
   *
   * <p>The hood is a limited-travel mechanism that adjusts the projectile launch angle. Higher
   * angles produce steeper trajectories for longer shots.
   */
  public static final class Hood {
    /** CAN ID for the hood motor. */
    public static final int kMotorId = 45;

    /**
     * Gear ratio from motor to hood output.
     *
     * <p>Motor rotations * kGearRatio = Hood rotations.
     */
    /**
     * Gearbox reduction (Input / Output).
     *
     * <p>1:5 Planetary Gearbox.
     */
    public static final double kMotorGearboxRatio = 6.6666666667;

    /** Pinion gear teeth count (Driving gear). */
    public static final double kPinionTeeth = 40.0;

    /** Virtual sector gear teeth count (Driven gear). */
    public static final double kSectorTeeth = 340.0;

    /**
     * Total gear reduction from motor to hood.
     *
     * <p>Formula: $$\Delta \theta_{load} = \frac{\Delta R_{motor}}{G_{box}} \times
     * \frac{N_{pinion}}{N_{sector\_virtual}} \times 360^\circ$$
     *
     * <p>Phoenix 6 SensorToMechanismRatio (Sensor Rotations / Mechanism Rotations): Ratio = G_box *
     * (N_sector / N_pinion) Calculation: 5.0 * (340.0 / 40.0) = 42.5
     */
    public static final double kGearRatio = kMotorGearboxRatio * (kSectorTeeth / kPinionTeeth);

    // --- Angle Limits ---

    /**
     * Initial hood angle in degrees (at motor position = 0).
     *
     * <p>Physical Zero Reference: When motor encoder reads 0 rotations, hood is at 65°.
     */
    public static final double kInitialAngleDegrees = 83.0;

    /**
     * Maximum hood angle in radians (initial position, highest elevation).
     *
     * <p>Corresponds to motor position = 0.
     */
    public static final double kMaxAngleRads = Math.toRadians(kInitialAngleDegrees);

    /**
     * Minimum hood angle in radians (lowest position after motor rotation).
     *
     * <p>TODO: Measure the actual minimum angle based on mechanical travel limit.
     */
    public static final double kMinAngleRads = Math.toRadians(60.0);

    // --- PID Gains ---

    /** Proportional gain for position control. */
    public static final double kP = 165.0;

    /** Derivative gain for position control. */
    public static final double kD = 0.0;

    /**
     * Static friction compensation voltage.
     *
     * <p>SysId Result: Average of forward (0.3759V) and reverse (0.4497V) quasistatic tests.
     */
    public static final double kS = 0.24624;

    /**
     * Velocity feedforward gain.
     *
     * <p>SysId Result: Average of forward (4.6979) and reverse (4.2875) quasistatic tests.
     *
     * <p>Units: Volts / (mechanism rotations per second).
     */
    public static final double kV = 4.29546;

    /**
     * Acceleration feedforward gain (negligible for high gear ratio).
     *
     * <p>SysId Result: ~0.0366 (very small due to 42.5:1 reduction).
     */
    public static final double kA = 0.04424;

    /**
     * Gravity feedforward.
     *
     * <p>Units: Volts. Compensates for gravity torque at horizontal position. Uses Arm_Cosine mode:
     * kG * cos(position_in_rotations * 2π). Calculated from SysId Quasistatic tests at different
     * angles.
     */
    public static final double kG = 0.0;

    /**
     * Robot acceleration compensation gain.
     *
     * <p>Units: Volts / (Meters/sec^2).
     *
     * <p>Compensates for inertial forces on the hood when the robot accelerates.
     *
     * <p>Formula: FF_acc = kARobot * Accel_Robot * sin(HoodAngle)
     */
    public static final double kARobot = 0.0366;

    // --- MotionMagic Parameters ---

    /**
     * MotionMagic cruise velocity in mechanism rotations per second.
     *
     * <p>At kV=4.49, 0.5 rot/s uses ~2.7V (safe under 5V limit).
     */
    public static final double kMotionMagicCruiseVelocity = 0.5;

    /**
     * MotionMagic acceleration in mechanism rotations per second squared.
     *
     * <p>Smooth ramp-up/ramp-down for large angle changes.
     */
    public static final double kMotionMagicAcceleration = 1.0;

    /**
     * Threshold for switching between MotionMagic and PD control.
     *
     * <p>If |target - current| >= this threshold, use MotionMagic. Otherwise, use PD + FF.
     */
    public static final double kMotionMagicThresholdRads = Math.toRadians(10.0);

    // --- Current Limits ---

    /** Supply current limit in Amps. */
    public static final double kSupplyCurrentLimit = 20.0;

    /** Stator current limit in Amps. */
    public static final double kStatorCurrentLimit = 40.0;

    // --- Tolerances ---

    /** Position tolerance for "at goal" detection in radians. */
    public static final double kPositionToleranceRads = Math.toRadians(0.50);
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
    public static final double kXP = 1.0;
    public static final double kXI = 0.0;
    public static final double kXD = 0.0;

    // Y Controller Gains
    public static final double kYP = 1.0;
    public static final double kYI = 0.0;
    public static final double kYD = 0.0;

    // Theta Controller Gains
    public static final double kThetaP = 4.0;
    public static final double kThetaI = 0.0;
    public static final double kThetaD = 0.0;

    /** Position error tolerance in meters. */
    public static final double kPositionTolerance = 0.01;

    /** Heading error tolerance in radians (3 degrees). */
    public static final double kThetaTolerance = Math.toRadians(0.50);
  }

  /**
   * Constants for the Shoot-On-Move (ShotCalculator) system.
   *
   * <p>These parameters control the physics-based shooting compensation algorithm that enables
   * accurate target acquisition while the robot is moving.
   *
   * <p><strong>Reference</strong>: Based on Team 6328's 2024 ShotCalculator architecture.
   */
  public static final class Shot {
    /**
     * Translation from robot center to turret rotation axis.
     *
     * <p><strong>Coordinate Frame</strong>: Robot-relative (+X forward, +Y left).
     *
     * <p><strong>Measurement</strong>: Measure from the geometric center of the robot (midpoint of
     * drivetrain) to the turret azimuth rotation axis.
     *
     * <p>TODO: Update this value based on CAD measurements.
     */
    public static final Translation2d kRobotToTurret = new Translation2d(0.2302, 0.1552);

    /**
     * Control loop phase delay compensation in seconds.
     *
     * <p><strong>Physics Note</strong>: The robot's pose estimate lags reality due to sensor
     * latency and processing time. This value advances the pose estimate by the expected delay to
     * improve aiming accuracy.
     *
     * <p>Typical value: 30-50ms for CAN + odometry latency.
     */
    public static final double kPhaseDelaySeconds = 0.03;

    /**
     * Minimum effective shooting distance in meters.
     *
     * <p><strong>Constraint</strong>: Below this distance, the ballistic solver cannot find a valid
     * trajectory (projectile energy too high for close range). Matches {@code
     * ShotTables.MIN_DISTANCE_M}.
     */
    public static final double kMinDistanceMeters = 0.50;

    /**
     * Maximum effective shooting distance in meters.
     *
     * <p><strong>Constraint</strong>: Beyond this distance, projectile energy at max RPM is
     * insufficient for reliable scoring. Matches {@code ShotTables.MAX_DISTANCE_M}.
     */
    public static final double kMaxDistanceMeters = 7.00;

    /**
     * Velocity filter time window in seconds.
     *
     * <p><strong>Signal Processing</strong>: This moving average filter smooths out high-frequency
     * noise in the turret/hood velocity calculations. A 100ms window (5 samples at 50Hz) provides
     * good noise rejection while maintaining responsiveness.
     */
    public static final double kVelocityFilterWindowSeconds = 0.4;

    /**
     * Inertia Inheritance Coefficient (0.0 to 1.0).
     *
     * <p>Due to aerodynamic drag during the instant the ball leaves the shooter, it does not
     * inherit 100% of the robot's velocity. Tune this down if high-speed shooting overcompensates
     * (overshoots less or undershoots). 1.0 = Perfect vacuum physics. 0.5 = 50% retained.
     */
    public static final double kInertiaInheritance = 1.0;

    /**
     * Maximum number of Time-of-Flight iterations.
     *
     * <p><strong>Convergence</strong>: The iterative ToF solver typically converges within 5-10
     * iterations. Additional iterations provide diminishing returns and add computational cost.
     */
    public static final int kMaxTofIterations = 20;
  }
}
