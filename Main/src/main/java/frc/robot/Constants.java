package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {
  public static final double kLooperDt = 0.01; // 100Hz

  public static final class Swerve {
    public static final String kCanivoreBusName = "canivore"; // Set to "" if using Rio bus

    // Dimensions
    public static final double kTrackWidth = Units.inchesToMeters(20.75); // TODO: Measure
    public static final double kWheelBase = Units.inchesToMeters(20.75); // TODO: Measure
    public static final double kWheelRadius = Units.inchesToMeters(2.0);

    // Swerve Hardware (MK5n / MK4i L3 assumed - ADJUST RATIOS)
    public static final double kDriveGearRatio = 6.12; // L3
    public static final double kSteerGearRatio = 15.43; // MK4i

    // Speeds
    public static final double kMaxDriveVelocity = 4.5; // m/s (approx 14-15 ft/s)
    public static final double kMaxAngularVelocity = Math.PI * 4.0; // rad/s

    // Anti-Slip & Dynamics
    public static final double kRobotMass = 50.0; // kg - TODO: Measure
    public static final double kNormalFrictionCoefficient = 1.1; // Slicks on carpet - TODO: Tune
    public static final double kMaxAccelForce = kRobotMass * 9.81 * kNormalFrictionCoefficient;

    // Swerve Module Locations (FL, FR, BL, BR)
    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    // CAN IDs
    public static final int kFLDriveId = 1;
    public static final int kFLSteerId = 2;
    public static final int kFLEncoderId = 3;

    public static final int kFRDriveId = 4;
    public static final int kFRSteerId = 5;
    public static final int kFREncoderId = 6;

    public static final int kBLDriveId = 7;
    public static final int kBLSteerId = 8;
    public static final int kBLEncoderId = 9;

    public static final int kBRDriveId = 10;
    public static final int kBRSteerId = 11;
    public static final int kBREncoderId = 12;

    // Offsets
    public static final double kFLOffset = 0.0;
    public static final double kFROffset = 0.0;
    public static final double kBLOffset = 0.0;
    public static final double BROffset = 0.0;

    // Sensors
    public static final int kPigeonId = 13;

    // 1690 Physics Constants
    public static final double kSkidThreshold = 0.5; // m/s
    public static final double kCollisionThreshold = 2.0; // Gs
  }
}
