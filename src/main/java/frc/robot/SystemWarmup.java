package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * JIT Warmup Utility for eliminating first-loop latency spikes.
 *
 * <p><strong>Problem</strong>: The JVM's Just-In-Time (JIT) compiler initially runs bytecode in
 * interpreted mode, which is slow. Code paths that are "hot" (frequently executed) get compiled to
 * native code, but this compilation happens lazily during the first few executions. On the RoboRIO,
 * this manifests as a "first loop spike" where the initial control cycle takes 50-120ms instead of
 * the expected 20ms.
 *
 * <p><strong>Solution</strong>: Force class loading and JIT compilation during robotInit() by
 * executing representative code paths before the match starts. This includes:
 *
 * <ul>
 *   <li>Geometry/Math classes (Pose2d, Rotation2d, etc.)
 *   <li>Kinematics calculations (SwerveDriveKinematics, Odometry)
 *   <li>Subsystem class loading
 * </ul>
 *
 * <p><strong>Timing</strong>: This runs during robotInit(), which has no time constraint. The match
 * begins only after the FMS signals autonomous start.
 *
 * @see <a href="https://wiki.openjdk.org/display/HotSpot/TieredCompilation">JVM Tiered
 *     Compilation</a>
 */
public class SystemWarmup {

  /**
   * Executes the warmup routine.
   *
   * <p>Triggers class loading and JIT compilation for performance-critical paths. Should be called
   * once during robotInit().
   */
  public static void warmup() {
    System.out.println("[SystemWarmup] Starting Warmup Routine...");

    // Warmup Geometry & Math classes
    Rotation2d r = new Rotation2d(1.0);
    r.plus(new Rotation2d(0.5));
    Pose2d p = new Pose2d();
    p.exp(new edu.wpi.first.math.geometry.Twist2d(0.1, 0.1, 0.1));

    // Warmup Swerve kinematics (allocations and computations)
    SwerveModulePosition[] modulePositions =
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    edu.wpi.first.math.geometry.Translation2d t =
        new edu.wpi.first.math.geometry.Translation2d(0.2, 0.2);
    edu.wpi.first.math.kinematics.SwerveDriveKinematics kinematics =
        new edu.wpi.first.math.kinematics.SwerveDriveKinematics(t, t, t, t);

    edu.wpi.first.math.kinematics.SwerveDriveOdometry odometry =
        new edu.wpi.first.math.kinematics.SwerveDriveOdometry(
            kinematics, new Rotation2d(), modulePositions, new Pose2d());

    // Hot-loop simulation to trigger C1/C2 JIT compilation
    int iterations = 1000;
    edu.wpi.first.math.kinematics.ChassisSpeeds dummySpeeds =
        new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 0.0, 0.1);

    for (int i = 0; i < iterations; i++) {
      kinematics.toSwerveModuleStates(dummySpeeds);
      odometry.update(new Rotation2d(0.0001 * i), modulePositions);
      RobotState.getInstance().addFieldToVehicleObservation(0.0 + i, new Pose2d());
    }

    // Force subsystem class loading
    try {
      Class.forName("frc.robot.subsystems.drive.Drive");
      Class.forName("frc.robot.subsystems.drive.DriveIOReal");
      Class.forName("frc.robot.subsystems.swerve.SwerveModule");
    } catch (ClassNotFoundException e) {
      e.printStackTrace();
    }

    System.out.println("[SystemWarmup] Warmup Complete.");
  }
}
