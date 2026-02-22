package frc.robot.simulation;

import frc.robot.libraries.lib9427.BallisticSolver;
import frc.robot.libraries.lib9427.BallisticSolver.ShootingSolution;
import frc.robot.libraries.lib9427.BallisticSolver.TrajectoryPoint;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Comprehensive ballistic simulation report generator.
 *
 * <p>Produces CSV data files for visualization by {@code plot_ballistic_report.py}. Covers four
 * simulation scenarios:
 *
 * <ol>
 *   <li><b>Distance Sweep</b> — static shot parameters vs distance
 *   <li><b>Motion Compensation</b> — turret/hood tracking while robot moves
 *   <li><b>Sensitivity Analysis</b> — impact of pitch/velocity perturbation
 *   <li><b>Hit Zone Map</b> — field coverage and hexagon impact distribution
 * </ol>
 *
 * <h2>Hardware Configuration</h2>
 *
 * <pre>
 * Shooter: Single 4" Orange (40A) AndyMark Stealth Wheel + Polycarbonate Backplate
 *   - Wheel OD:      4.0 in  (0.1016 m)
 *   - Wheel Radius:  2.0 in  (0.0508 m)
 *   - Durometer:     40A (soft, high grip on game piece)
 *   - Slip Factor:   ~0.60 (single-wheel + stationary backplate)
 *     → Effective radius for RPM↔velocity = actual_radius × slip_factor
 *   - Motor:         Dual KrakenX60 (leader-follower, inward rotation)
 *   - Max Wheel RPM: 4000
 *
 * Projectile (2026 Game Piece):
 *   - Mass:     0.215 kg
 *   - Diameter: 0.150 m
 *   - Cd:       0.485  (drag coefficient for smooth sphere ~Re 10^4)
 *   - Cm:       0.50   (Magnus coefficient)
 * </pre>
 *
 * <h2>Usage</h2>
 *
 * <pre>{@code
 * # Build the native library first, then run:
 * ./gradlew simulateJava -PmainClass=frc.robot.simulation.BallisticReport
 * # Or run from IDE with JNI library on java.library.path
 * }</pre>
 *
 * @author Team 9427
 */
public class BallisticReport {

  // =========================================================================
  // Hardware Constants — 4" Orange Stealth Wheel + Backplate
  // =========================================================================

  /**
   * Wheel radius [m]. 4-inch outer diameter → 2-inch radius.
   *
   * <p>Source: <a href="https://andymark.com/products/stealth-and-sushi-wheels">AndyMark Stealth
   * Wheel</a>
   */
  private static final double WHEEL_RADIUS_M = 0.0508; // 2 inches

  /**
   * Slip factor for single-wheel + hard-backplate configuration.
   *
   * <p>In a single-wheel shooter with a stationary backplate, the ball's exit velocity is less than
   * the wheel surface speed because only one side imparts energy. Typical range: 0.50–0.70.
   *
   * <p>This value should be tuned on the real robot by measuring actual exit velocity vs. wheel
   * RPM. Orbit 1690 reported their factor was "always really close to 1" for a dual-wheel setup;
   * for single-wheel + backplate, expect ~0.6.
   */
  private static final double SLIP_FACTOR = 0.60;

  /**
   * Effective wheel radius passed to the solver. Accounts for the slip factor so that the solver's
   * internal RPM ↔ velocity conversion produces correct exit velocities.
   *
   * <p>Velocity chain:
   *
   * <pre>
   *   actual_surface_speed = RPM × (2π/60) × WHEEL_RADIUS_M
   *   v_exit               = actual_surface_speed × SLIP_FACTOR
   *                        = RPM × (2π/60) × (WHEEL_RADIUS_M × SLIP_FACTOR)
   *                        = RPM × (2π/60) × EFFECTIVE_WHEEL_RADIUS_M
   *
   *   Example @ 4000 RPM:
   *     surface_speed = 4000 × 0.10472 × 0.0508 = 21.28 m/s
   *     v_exit        = 21.28 × 0.60             = 12.77 m/s
   *
   *   The solver reports RPM as the ACTUAL WHEEL RPM the motor must spin.
   * </pre>
   */
  private static final double EFFECTIVE_WHEEL_RADIUS_M = WHEEL_RADIUS_M * SLIP_FACTOR;

  private static final double MIN_RPM = 0.0;
  private static final double MAX_RPM = 4000.0;
  private static final double PITCH_MIN_RAD = 0.10; // ~5.7 deg
  private static final double PITCH_MAX_RAD = 1.30; // ~74.5 deg
  private static final double TURRET_MAX_RATE_RADPS = 6.0; // ~344 deg/s

  // =========================================================================
  // Target & Robot Constants
  // =========================================================================

  /** Target height: 72 inches = 1.8288 m. */
  private static final double TARGET_HEIGHT_M = 72.0 * 0.0254;

  /** Turret pivot height above ground [m]. */
  private static final double TURRET_HEIGHT_M = 0.50;

  /** RK4 integration time step [s]. 2ms → 500 Hz for high accuracy. */
  private static final double DT_S = 0.002;

  /** Robot control loop period [s]. */
  private static final double LOOP_DT = 0.02;

  /** Output directory for CSV files. */
  private static final String OUTPUT_DIR = "src/main/python/ballistic_report";

  // =========================================================================
  // Main
  // =========================================================================

  public static void main(String[] args) {
    System.out.println(banner());

    // Create output directory
    new File(OUTPUT_DIR).mkdirs();

    try (BallisticSolver solver = new BallisticSolver()) {
      configureSolver(solver);

      System.out.println("\n[1/4] Distance Sweep...");
      runDistanceSweep(solver);

      System.out.println("[2/4] Motion Compensation...");
      runMotionCompensation(solver);

      System.out.println("[3/4] Sensitivity Analysis...");
      runSensitivityAnalysis(solver);

      System.out.println("[4/4] Hit Zone Map...");
      runHitZoneMap(solver);

      System.out.println("\n" + "=".repeat(70));
      System.out.println("ALL CSV FILES WRITTEN TO: " + OUTPUT_DIR + "/");
      System.out.println("Run plot_ballistic_report.py to generate figures.");
      System.out.println("=".repeat(70));

    } catch (Exception e) {
      System.err.println("FATAL: " + e.getMessage());
      e.printStackTrace();
    }
  }

  // =========================================================================
  // Solver Configuration
  // =========================================================================

  private static void configureSolver(BallisticSolver solver) {
    // Projectile: 2026 game piece defaults
    solver.configureProjectile(0.215, 0.150, 0.485, 0.50, 0.0024, 0.001);

    // Shooter constraints with effective wheel radius
    solver.setShooterConstraints(
        MIN_RPM,
        MAX_RPM,
        EFFECTIVE_WHEEL_RADIUS_M,
        PITCH_MIN_RAD,
        PITCH_MAX_RAD,
        TURRET_MAX_RATE_RADPS);

    // Target at field origin
    solver.setTarget(0.0, 0.0, TARGET_HEIGHT_M);

    // High-precision RK4
    solver.setTimeStep(DT_S);

    System.out.println("Solver configured:");
    System.out.printf("  Wheel:   4\" Orange Stealth (40A) + backplate%n");
    System.out.printf(
        "  Radius:  %.4f m (actual) × %.2f (slip) = %.4f m (effective)%n",
        WHEEL_RADIUS_M, SLIP_FACTOR, EFFECTIVE_WHEEL_RADIUS_M);
    System.out.printf("  RPM:     %.0f – %.0f%n", MIN_RPM, MAX_RPM);
    System.out.printf(
        "  Pitch:   %.1f° – %.1f°%n", Math.toDegrees(PITCH_MIN_RAD), Math.toDegrees(PITCH_MAX_RAD));
    System.out.printf("  Target:  (0, 0, %.4f m)%n", TARGET_HEIGHT_M);
    System.out.printf("  RK4 dt:  %.1f ms%n", DT_S * 1000);
  }

  // =========================================================================
  // Scenario 1: Distance Sweep
  // =========================================================================

  /**
   * Sweeps distance from 2m to 10m at 0.25m intervals. Robot is stationary, positioned along the +X
   * axis from the target.
   */
  private static void runDistanceSweep(BallisticSolver solver) {
    String path = OUTPUT_DIR + "/distance_sweep.csv";
    try (PrintWriter pw = new PrintWriter(new FileWriter(path))) {
      pw.println(
          "distance_m,pitch_deg,velocity_mps,rpm,tof_s,apex_height_m,impact_z_m,"
              + "impact_x_m,impact_y_m,valid");

      for (double dist = 2.0; dist <= 10.01; dist += 0.25) {
        ShootingSolution sol = solver.solve(dist, 0.0, 0, 0, 0, 0, TURRET_HEIGHT_M, 0, 0, 0, 0);

        if (sol.valid) {
          // Get trajectory for apex computation
          TrajectoryPoint[] traj =
              solver.simulateTrajectory(
                  sol.yawRad,
                  sol.pitchRad,
                  sol.velocityMps,
                  dist,
                  0,
                  0,
                  0,
                  0,
                  0,
                  TURRET_HEIGHT_M,
                  0,
                  0,
                  500);

          double apexZ = 0;
          if (traj != null) {
            for (TrajectoryPoint pt : traj) {
              if (pt.z > apexZ) apexZ = pt.z;
            }
          }

          // Convert solver RPM (based on effective radius) to actual wheel RPM
          // solver reports RPM as: velocity / (2π/60 * effective_radius)
          // actual wheel RPM = solver_RPM × slip_factor  (lower because effective_r is smaller)
          // Actually: solver RPM IS the actual wheel RPM needed (since effective_radius
          // already encodes the slip). The ball exits at velocity = RPM * ω * eff_radius.
          double actualWheelRpm = sol.rpm;

          pw.printf(
              "%.4f,%.4f,%.4f,%.1f,%.4f,%.4f,%.4f,%.4f,%.4f,%b%n",
              dist,
              Math.toDegrees(sol.pitchRad),
              sol.velocityMps,
              actualWheelRpm,
              sol.timeOfFlightS,
              apexZ,
              sol.impactZM,
              sol.impactXM,
              sol.impactYM,
              sol.valid);
        } else {
          pw.printf("%.4f,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,false%n", dist);
        }
      }
      System.out.println("  → " + path);
    } catch (IOException e) {
      System.err.println("Error writing " + path + ": " + e.getMessage());
    }
  }

  // =========================================================================
  // Scenario 2: Motion Compensation Timeline
  // =========================================================================

  /** Defines a motion scenario for the robot. */
  private record MotionScenario(
      String name,
      double startX,
      double startY,
      double vx,
      double vy,
      double omega,
      double heading) {}

  /**
   * Simulates the robot moving over 5 seconds. Each 20ms cycle calls solve() and records turret
   * yaw, hood pitch, RPM, and yaw-lead compensation.
   */
  private static void runMotionCompensation(BallisticSolver solver) {
    MotionScenario[] scenarios = {
      new MotionScenario("approaching", 8.0, 0.0, -2.0, 0.0, 0.0, Math.PI),
      new MotionScenario("retreating", 4.0, 0.0, 2.0, 0.0, 0.0, 0.0),
      new MotionScenario("strafing", 5.0, -4.0, 0.0, 2.0, 0.0, 0.0),
      new MotionScenario("rotating", 5.0, 2.0, 0.0, 0.0, Math.toRadians(90), 0.0),
      new MotionScenario("combined", 7.0, -3.0, -1.5, 1.0, 0.0, Math.PI),
    };

    for (MotionScenario sc : scenarios) {
      String path = OUTPUT_DIR + "/motion_" + sc.name + ".csv";
      try (PrintWriter pw = new PrintWriter(new FileWriter(path))) {
        pw.println(
            "t,robot_x,robot_y,robot_vx,robot_vy,heading_deg,"
                + "distance_m,turret_yaw_deg,direct_yaw_deg,yaw_lead_deg,"
                + "hood_pitch_deg,shooter_rpm,velocity_mps,tof_s,valid");

        double duration = 5.0;
        for (double t = 0; t <= duration + 1e-6; t += LOOP_DT) {
          double rx = sc.startX + sc.vx * t;
          double ry = sc.startY + sc.vy * t;
          double heading = sc.heading + sc.omega * t;
          double dist = Math.sqrt(rx * rx + ry * ry);

          // Direct yaw: angle from robot to target (no compensation)
          double directYaw = Math.atan2(-ry, -rx);

          ShootingSolution sol =
              solver.solve(rx, ry, sc.vx, sc.vy, sc.omega, heading, TURRET_HEIGHT_M, 0, 0, 0, 0);

          if (sol.valid) {
            pw.printf(
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,"
                    + "%.4f,%.4f,%.4f,%.4f,"
                    + "%.4f,%.1f,%.4f,%.4f,%b%n",
                t,
                rx,
                ry,
                sc.vx,
                sc.vy,
                Math.toDegrees(heading),
                dist,
                Math.toDegrees(sol.yawRad),
                Math.toDegrees(directYaw),
                Math.toDegrees(sol.yawLeadRad),
                Math.toDegrees(sol.pitchRad),
                sol.rpm,
                sol.velocityMps,
                sol.timeOfFlightS,
                true);
          } else {
            pw.printf(
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,"
                    + "%.4f,%.4f,%.4f,NaN,"
                    + "NaN,NaN,NaN,NaN,false%n",
                t,
                rx,
                ry,
                sc.vx,
                sc.vy,
                Math.toDegrees(heading),
                dist,
                Double.NaN,
                Math.toDegrees(directYaw));
          }
        }
        System.out.println("  → " + path);
      } catch (IOException e) {
        System.err.println("Error writing " + path + ": " + e.getMessage());
      }
    }

    // Also generate representative trajectory CSVs for the trajectory figure
    generateTrajectoryCSVs(solver);
  }

  /**
   * Generates full trajectory point data for representative static and moving shots. Used by the
   * Python script for Figure 3 (Trajectory Views).
   */
  private static void generateTrajectoryCSVs(BallisticSolver solver) {
    // Static shot from 5m
    writeTrajectoryCSV(solver, "trajectory_static_5m", 5.0, 0.0, 0, 0, 0, 0);
    // Static shot from 8m
    writeTrajectoryCSV(solver, "trajectory_static_8m", 8.0, 0.0, 0, 0, 0, 0);
    // Moving: approaching at 2 m/s from 6m
    writeTrajectoryCSV(solver, "trajectory_approaching", 6.0, 0.0, -2.0, 0.0, 0, Math.PI);
    // Moving: strafing at 2 m/s from 5m, offset Y=2
    writeTrajectoryCSV(solver, "trajectory_strafing", 5.0, 2.0, 0.0, -2.0, 0, 0);
  }

  private static void writeTrajectoryCSV(
      BallisticSolver solver,
      String name,
      double rx,
      double ry,
      double vx,
      double vy,
      double omega,
      double heading) {

    ShootingSolution sol =
        solver.solve(rx, ry, vx, vy, omega, heading, TURRET_HEIGHT_M, 0, 0, 0, 0);

    if (!sol.valid) {
      System.out.println("  [SKIP] No solution for " + name);
      return;
    }

    TrajectoryPoint[] traj =
        solver.simulateTrajectory(
            sol.yawRad,
            sol.pitchRad,
            sol.velocityMps,
            rx,
            ry,
            vx,
            vy,
            omega,
            heading,
            TURRET_HEIGHT_M,
            0,
            0,
            500);

    if (traj == null || traj.length == 0) return;

    String path = OUTPUT_DIR + "/" + name + ".csv";
    try (PrintWriter pw = new PrintWriter(new FileWriter(path))) {
      pw.println("t,x,y,z,vx,vy,vz,horiz_dist,speed");

      double launchX = traj[0].x;
      double launchY = traj[0].y;

      for (TrajectoryPoint pt : traj) {
        double hdist =
            Math.sqrt((pt.x - launchX) * (pt.x - launchX) + (pt.y - launchY) * (pt.y - launchY));
        double speed = Math.sqrt(pt.vx * pt.vx + pt.vy * pt.vy + pt.vz * pt.vz);
        pw.printf(
            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
            pt.t, pt.x, pt.y, pt.z, pt.vx, pt.vy, pt.vz, hdist, speed);
      }

      // Write metadata as a separate one-line file
      String metaPath = OUTPUT_DIR + "/" + name + "_meta.csv";
      try (PrintWriter meta = new PrintWriter(new FileWriter(metaPath))) {
        meta.println(
            "robot_x,robot_y,robot_vx,robot_vy,yaw_deg,pitch_deg,velocity_mps,rpm,tof_s,"
                + "yaw_lead_deg,target_z");
        meta.printf(
            "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.4f,%.4f,%.4f%n",
            rx,
            ry,
            vx,
            vy,
            Math.toDegrees(sol.yawRad),
            Math.toDegrees(sol.pitchRad),
            sol.velocityMps,
            sol.rpm,
            sol.timeOfFlightS,
            Math.toDegrees(sol.yawLeadRad),
            TARGET_HEIGHT_M);
      }

      System.out.println("  → " + path);
    } catch (IOException e) {
      System.err.println("Error writing " + path + ": " + e.getMessage());
    }
  }

  // =========================================================================
  // Scenario 3: Sensitivity Analysis
  // =========================================================================

  /**
   * At a fixed distance (5m), perturbs pitch and velocity around the optimal solution and records
   * where the ball lands. Shows how tolerant the solution is to small errors.
   */
  private static void runSensitivityAnalysis(BallisticSolver solver) {
    double robotX = 5.0;
    double robotY = 0.0;

    // Get optimal solution
    ShootingSolution optimal =
        solver.solve(robotX, robotY, 0, 0, 0, 0, TURRET_HEIGHT_M, 0, 0, 0, 0);

    if (!optimal.valid) {
      System.out.println("  [SKIP] No optimal solution at 5m");
      return;
    }

    double optPitch = optimal.pitchRad;
    double optVel = optimal.velocityMps;
    double optYaw = optimal.yawRad;

    String path = OUTPUT_DIR + "/sensitivity.csv";
    try (PrintWriter pw = new PrintWriter(new FileWriter(path))) {
      pw.println(
          "pitch_offset_deg,velocity_offset_pct,pitch_deg,velocity_mps,"
              + "impact_x_m,impact_y_m,impact_z_m,horiz_miss_m,vert_miss_m,"
              + "opt_pitch_deg,opt_velocity_mps,distance_m");

      // Write optimal reference on first line
      pw.printf(
          "0.0000,0.0000,%.4f,%.4f,%.4f,%.4f,%.4f,0.0000,0.0000,%.4f,%.4f,%.4f%n",
          Math.toDegrees(optPitch),
          optVel,
          optimal.impactXM,
          optimal.impactYM,
          optimal.impactZM,
          Math.toDegrees(optPitch),
          optVel,
          robotX);

      // Sweep pitch: -3° to +3° in 0.25° steps
      // Sweep velocity: -10% to +10% in 1% steps
      for (double pitchOffDeg = -3.0; pitchOffDeg <= 3.01; pitchOffDeg += 0.25) {
        for (double velOffPct = -10.0; velOffPct <= 10.01; velOffPct += 1.0) {
          if (Math.abs(pitchOffDeg) < 0.01 && Math.abs(velOffPct) < 0.01) continue; // skip optimal

          double testPitch = optPitch + Math.toRadians(pitchOffDeg);
          double testVel = optVel * (1.0 + velOffPct / 100.0);

          // Simulate perturbed trajectory
          TrajectoryPoint[] traj =
              solver.simulateTrajectory(
                  optYaw,
                  testPitch,
                  testVel,
                  robotX,
                  robotY,
                  0,
                  0,
                  0,
                  0,
                  TURRET_HEIGHT_M,
                  0,
                  0,
                  500);

          if (traj == null || traj.length < 2) continue;

          // Find where ball crosses target distance
          double launchX = traj[0].x;
          double launchY = traj[0].y;
          double targetDist = robotX; // distance to origin

          double impX = Double.NaN, impY = Double.NaN, impZ = Double.NaN;

          for (int i = 1; i < traj.length; i++) {
            double d =
                Math.sqrt(
                    (traj[i].x - launchX) * (traj[i].x - launchX)
                        + (traj[i].y - launchY) * (traj[i].y - launchY));
            double prevD =
                Math.sqrt(
                    (traj[i - 1].x - launchX) * (traj[i - 1].x - launchX)
                        + (traj[i - 1].y - launchY) * (traj[i - 1].y - launchY));

            if (d >= targetDist && prevD < targetDist) {
              double alpha = (targetDist - prevD) / (d - prevD);
              impX = traj[i - 1].x + alpha * (traj[i].x - traj[i - 1].x);
              impY = traj[i - 1].y + alpha * (traj[i].y - traj[i - 1].y);
              impZ = traj[i - 1].z + alpha * (traj[i].z - traj[i - 1].z);
              break;
            }
          }

          if (Double.isNaN(impZ)) continue;

          // Miss distance relative to target center (0, 0, TARGET_HEIGHT)
          double horizMiss = Math.sqrt(impX * impX + impY * impY); // from origin
          double vertMiss = impZ - TARGET_HEIGHT_M;

          pw.printf(
              "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n",
              pitchOffDeg,
              velOffPct,
              Math.toDegrees(testPitch),
              testVel,
              impX,
              impY,
              impZ,
              horizMiss,
              vertMiss,
              Math.toDegrees(optPitch),
              optVel,
              robotX);
        }
      }

      System.out.println("  → " + path);
    } catch (IOException e) {
      System.err.println("Error writing " + path + ": " + e.getMessage());
    }
  }

  // =========================================================================
  // Scenario 4: Hit Zone Map
  // =========================================================================

  /**
   * Solves from a grid of field positions (X: 2–9m, Y: -5–5m) and records the solution and impact
   * point. Shows which positions on the field have valid shots and where they land on the hexagon.
   */
  private static void runHitZoneMap(BallisticSolver solver) {
    String path = OUTPUT_DIR + "/hitzone.csv";
    try (PrintWriter pw = new PrintWriter(new FileWriter(path))) {
      pw.println(
          "robot_x,robot_y,distance_m,valid,pitch_deg,velocity_mps,rpm,tof_s,"
              + "impact_x_m,impact_y_m,impact_z_m,yaw_deg");

      for (double rx = 2.0; rx <= 9.01; rx += 0.5) {
        for (double ry = -5.0; ry <= 5.01; ry += 0.5) {
          double dist = Math.sqrt(rx * rx + ry * ry);

          ShootingSolution sol = solver.solve(rx, ry, 0, 0, 0, 0, TURRET_HEIGHT_M, 0, 0, 0, 0);

          if (sol.valid) {
            pw.printf(
                "%.4f,%.4f,%.4f,true,%.4f,%.4f,%.1f,%.4f,%.6f,%.6f,%.6f,%.4f%n",
                rx,
                ry,
                dist,
                Math.toDegrees(sol.pitchRad),
                sol.velocityMps,
                sol.rpm,
                sol.timeOfFlightS,
                sol.impactXM,
                sol.impactYM,
                sol.impactZM,
                Math.toDegrees(sol.yawRad));
          } else {
            pw.printf("%.4f,%.4f,%.4f,false,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN%n", rx, ry, dist);
          }
        }
      }

      System.out.println("  → " + path);
    } catch (IOException e) {
      System.err.println("Error writing " + path + ": " + e.getMessage());
    }
  }

  // =========================================================================
  // Banner
  // =========================================================================

  private static String banner() {
    return """
        ╔══════════════════════════════════════════════════════════════════╗
        ║          BALLISTIC SOLVER — FULL SIMULATION REPORT             ║
        ║                    Team 9427 · 2026                            ║
        ╠══════════════════════════════════════════════════════════════════╣
        ║  Shooter:  4" Orange (40A) Stealth Wheel + PC Backplate       ║
        ║  Physics:  Gravity + Drag + Magnus · RK4 @ 500 Hz             ║
        ║  Solver:   C++ native via JNI                                 ║
        ╚══════════════════════════════════════════════════════════════════╝""";
  }
}
