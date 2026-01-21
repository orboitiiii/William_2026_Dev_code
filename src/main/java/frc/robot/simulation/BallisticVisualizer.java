package frc.robot.simulation;

import frc.robot.libraries.lib9427.BallisticSolver;
import frc.robot.libraries.lib9427.BallisticSolver.ShootingSolution;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Generates visualization data for 3D trajectory plots.
 *
 * <p>Outputs:
 *
 * <ul>
 *   <li>CSV files for external plotting (Python/MATLAB/Excel)
 *   <li>Console frame-by-frame animation showing robot + turret + projectile
 * </ul>
 *
 * @author Team 9427 William
 */
public class BallisticVisualizer {

  // Simulation parameters
  private static final double DT = 0.02; // 50 Hz frame rate

  /**
   * Generates a "Rapid Fire" simulation where projectiles are launched periodically. Outputs two
   * CSV files: 1. [name]_robot.csv: t, robot_x, robot_y, heading, turret_yaw 2.
   * [name]_projectiles.csv: t, id, x, y, z
   */
  private static void runRapidFireSimulation(
      BallisticSolver solver, SimulationScenario scenario, double duration, double fireInterval) {
    System.out.println("Running Rapid Fire Simulation: " + scenario.name);

    List<ProjectileState> activeProjectiles = new ArrayList<>();
    int projectileCounter = 0;
    double lastFireTime = -fireInterval; // Fire immediately at t=0

    // Buffers for CSV data
    StringBuilder robotCsv =
        new StringBuilder("t,x,y,heading,turret_yaw,pitch,target_x,target_y,target_z\n");
    StringBuilder projCsv = new StringBuilder("t,id,x,y,z\n");

    for (double t = 0; t <= duration; t += DT) {
      // 1. Update Robot State
      double robotX = scenario.robotX + scenario.robotVx * t;
      double robotY = scenario.robotY + scenario.robotVy * t;
      double heading = scenario.robotHeading + scenario.robotOmega * t;

      // 2. Solve for Target
      ShootingSolution sol =
          solver.solve(
              robotX,
              robotY,
              scenario.robotVx,
              scenario.robotVy,
              scenario.robotOmega,
              heading,
              0.5,
              0.0,
              0.0,
              0.0,
              0.0);

      double turretYaw = sol.valid ? sol.yawRad : Math.atan2(-robotY, -robotX);

      // 3. Fire Gun?
      if (sol.valid && (t - lastFireTime >= fireInterval)) {
        // v0 is launch speed (relative to shooter muzzle)
        double v0 = sol.velocityMps;
        double pitch = sol.pitchRad;
        // turretYaw is already defined in outer scope

        // 1. Calculate Muzzle Velocity Vector (Relative to Field, IGNORING robot motion for a
        // moment)
        // This vector represents the direction the barrel is pointing.
        // Vx_muzzle = v0 * cos(pitch) * cos(yaw)
        // Vy_muzzle = v0 * cos(pitch) * sin(yaw)
        // Vz_muzzle = v0 * sin(pitch)
        double mvx = v0 * Math.cos(pitch) * Math.cos(turretYaw);
        double mvy = v0 * Math.cos(pitch) * Math.sin(turretYaw);
        double mvz = v0 * Math.sin(pitch);

        // 2. Add Robot Velocity (The "Inertia")
        // The ball inherits the robot's velocity vector.
        // V_ball_world = V_muzzle_world + V_robot_world
        double ballVx = mvx + scenario.robotVx;
        double ballVy = mvy + scenario.robotVy;
        double ballVz = mvz;

        // Launch position (approximate at robot center + 0.5m height)
        activeProjectiles.add(
            new ProjectileState(projectileCounter++, robotX, robotY, 0.5, ballVx, ballVy, ballVz));
        lastFireTime = t;
      }

      // 4. Update Projectiles (Physics Step)
      List<ProjectileState> keptProjectiles = new ArrayList<>();
      for (ProjectileState p : activeProjectiles) {
        // Symplectic Euler
        // Forces: Gravity + Drag
        double v = Math.sqrt(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz);
        double dragForce = 0.5 * 1.225 * 0.47 * (Math.PI * 0.075 * 0.075) * v * v;
        double dragAcc = dragForce / 0.21; // mass ~0.21 kg

        double ax = -(dragAcc * (p.vx / v));
        double ay = -(dragAcc * (p.vy / v));
        double az = -9.81 - (dragAcc * (p.vz / v));

        p.vx += ax * DT;
        p.vy += ay * DT;
        p.vz += az * DT;

        p.x += p.vx * DT;
        p.y += p.vy * DT;
        p.z += p.vz * DT;

        // Ground constraint
        if (p.z > 0.0) {
          keptProjectiles.add(p);
          // Log to CSV
          projCsv.append(String.format("%.4f,%d,%.4f,%.4f,%.4f\n", t, p.id, p.x, p.y, p.z));
        }
      }
      activeProjectiles = keptProjectiles;

      // 5. Log Robot State
      // Added turret pitch (defaults to 0 if invalid, but we only log valid shots usually?
      // Actually let's use the last known or 0).
      // Wait, we need pitch from the solution.

      double logPitch = sol.valid ? sol.pitchRad : 0.0;

      robotCsv.append(
          String.format(
              "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n",
              t, robotX, robotY, heading, turretYaw, logPitch, 0.0, 0.0, 1.83));
    }

    // Write Files
    writeToFile(scenario.name + "_robot.csv", robotCsv.toString());
    writeToFile(scenario.name + "_projectiles.csv", projCsv.toString());

    System.out.println("Exported rapid fire data for " + scenario.name);
  }

  private static void writeToFile(String filename, String content) {
    try (PrintWriter pw = new PrintWriter(new FileWriter(filename))) {
      pw.print(content);
    } catch (IOException e) {
      System.err.println("Error writing " + filename + ": " + e.getMessage());
    }
  }

  // Updated Main to run these scenarios
  public static void main(String[] args) {
    try (BallisticSolver solver = new BallisticSolver()) {
      solver.setShooterConstraints(0, 4500, 0.05, 0.15, 1.3, 6.0);
      solver.setTarget(0.0, 0.0, 1.8288);
      solver.setTimeStep(0.005); // High fidelity

      // 1. Strafing Right (Top View candidate)
      // High speed to show lead angle clearly
      SimulationScenario strafe =
          new SimulationScenario(
              "strafing_fire",
              5.0,
              4.0,
              0.0,
              -3.0, // Vx=0, Vy=-3.0 (FAST strafe)
              Math.PI,
              0.0);
      runRapidFireSimulation(
          solver, strafe, 6.0, 0.4); // Shorter duration is fine for demo, but kept 6s?
      // User liked long, let's keep 10s but adjust start Y so we don't go off map.
      // Start Y=10, move -3 m/s for 8s = -24m. End Y=-14. Fits in -15 limits.
      strafe.robotY = 10.0;
      runRapidFireSimulation(solver, strafe, 8.0, 0.4);

      // 2. Retreating (Side View candidate)
      // Start near, move back.
      SimulationScenario retreat =
          new SimulationScenario(
              "retreating_fire",
              2.0,
              0.0,
              1.5,
              0.0, // Vx=1.5 (Retreating)
              Math.PI, // Facing target (backing away)
              0.0);
      runRapidFireSimulation(solver, retreat, 10.0, 0.4);

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  static class ProjectileState {
    int id;
    double x, y, z;
    double vx, vy, vz;

    ProjectileState(int id, double x, double y, double z, double vx, double vy, double vz) {
      this.id = id;
      this.x = x;
      this.y = y;
      this.z = z;
      this.vx = vx;
      this.vy = vy;
      this.vz = vz;
    }
  }

  // (Keeping existing helper classes if needed, or redefining them)
  static class SimulationScenario {
    String name;
    double robotX, robotY, robotVx, robotVy, robotHeading, robotOmega;

    SimulationScenario(String name, double x, double y, double vx, double vy, double h, double o) {
      this.name = name;
      this.robotX = x;
      this.robotY = y;
      this.robotVx = vx;
      this.robotVy = vy;
      this.robotHeading = h;
      this.robotOmega = o;
    }
  }
}
