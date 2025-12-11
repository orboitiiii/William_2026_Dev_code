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
    public static final double kDriveGearRatio = 6.12;
    public static final double kSteerGearRatio = 15.43;

    // CAN IDs & Offsets (Keep your existing values)
    public static final int kFLDriveId = 1;
    public static final int kFLSteerId = 2;
    public static final int kFLEncoderId = 3;
    public static final double kFLOffset = 0.0;

    public static final int kFRDriveId = 4;
    public static final int kFRSteerId = 5;
    public static final int kFREncoderId = 6;
    public static final double kFROffset = 0.0;

    public static final int kBLDriveId = 7;
    public static final int kBLSteerId = 8;
    public static final int kBLEncoderId = 9;
    public static final double kBLOffset = 0.0;

    public static final int kBRDriveId = 10;
    public static final int kBRSteerId = 11;
    public static final int kBREncoderId = 12;
    public static final double BROffset = 0.0;

    public static final int kPigeonId = 13;
  }
}
