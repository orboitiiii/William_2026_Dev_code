package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final double kLooperDt = 0.02; // 50Hz Loop (Standard FRC)

  public static final class Swerve {
    public static final String kCanivoreBusName = "canivore";

    // Dimensions
    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    public static final double kWheelBase = Units.inchesToMeters(20.75);
    public static final double kWheelRadius = Units.inchesToMeters(2.0);

    // Module Locations (Center based)
    public static final Translation2d kFLPos =
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0);
    public static final Translation2d kFRPos =
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0);
    public static final Translation2d kBLPos =
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0);
    public static final Translation2d kBRPos =
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0);

    // Physical Limits (The First Principles)
    public static final double kRobotMass = 55.0; // kg (Approx 120lbs + battery + bumpers)
    public static final double kCoF = 1.1; // Coefficient of Friction (Carpet ~1.1 to 1.3)
    public static final double kMaxFrictionForce = kRobotMass * 9.81 * kCoF; // F = mu * m * g
    public static final double kMaxDriveAcceleration =
        kMaxFrictionForce / kRobotMass; // a = F/m (~10.8 m/s^2)

    // Motor Limits (Falcon 500 / Kraken)
    public static final double kMaxDriveVelocity = 4.5; // m/s
    public static final double kMaxAngularVelocity = 10.0; // rad/s

    // Gear Ratios
    public static enum SwerveModuleType {
      MK5N_L1(8.14, 287.0 / 11.0),
      MK5N_L2(6.75, 287.0 / 11.0),
      MK5N_L3(6.12, 287.0 / 11.0);

      public final double driveRatio;
      public final double steerRatio;

      SwerveModuleType(double drive, double steer) {
        this.driveRatio = drive;
        this.steerRatio = steer;
      }
    }

    // Active Configuration (User Select Here)
    public static final SwerveModuleType kModuleType = SwerveModuleType.MK5N_L3; // Default to L2

    public static final double kDriveGearRatio = kModuleType.driveRatio;
    public static final double kSteerGearRatio = kModuleType.steerRatio;

    // CAN IDs & Offsets (Keep your existing values)
    public static final int kFLDriveId = 5;
    public static final int kFLSteerId = 1;
    public static final int kFLEncoderId = 9;
    public static final double kFLOffset = 0.0;

    public static final int kFRDriveId = 7;
    public static final int kFRSteerId = 3;
    public static final int kFREncoderId = 11;
    public static final double kFROffset = 0.0;

    public static final int kBLDriveId = 8;
    public static final int kBLSteerId = 4;
    public static final int kBLEncoderId = 12;
    public static final double kBLOffset = 0.0;

    public static final int kBRDriveId = 6;
    public static final int kBRSteerId = 2;
    public static final int kBREncoderId = 10;
    public static final double BROffset = 0.0;

    public static final int kPigeonId = 15;
  }
}
