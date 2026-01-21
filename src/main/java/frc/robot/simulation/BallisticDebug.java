package frc.robot.simulation;

import frc.robot.libraries.lib9427.BallisticSolver;
import frc.robot.libraries.lib9427.BallisticSolver.ShootingSolution;
import frc.robot.libraries.lib9427.BallisticSolver.TrajectoryPoint;

/** Debug test for ballistic solver - tests the solve() function with calculated velocity. */
public class BallisticDebug {

  public static void main(String[] args) {
    System.out.println("=== Ballistic Solver Debug Test ===\n");
    System.out.println("Testing solve() function with calculated velocity\n");

    try (BallisticSolver solver = new BallisticSolver()) {
      solver.setTarget(0.0, 0.0, 1.8288);
      solver.setTimeStep(0.002);
      solver.setShooterConstraints(0, 4000, 0.0508, 0.15, 1.3, 6.0);

      // Test various distances
      double[][] scenarios = {
        {5.0, 0.0},
        {3.0, 0.0},
        {8.0, 0.0},
        {5.0, 2.0},
        {6.0, 3.0},
      };

      System.out.println("Testing solve() function:");
      System.out.println("-".repeat(80));

      for (double[] scen : scenarios) {
        double robotX = scen[0];
        double robotY = scen[1];
        double dist = Math.sqrt(robotX * robotX + robotY * robotY);

        System.out.printf("\nRobot @ (%.1f, %.1f) m, distance = %.2f m%n", robotX, robotY, dist);

        ShootingSolution sol = solver.solve(robotX, robotY, 0, 0, 0, 0, 0.5, 0, 0, 0, 0);

        if (sol.valid) {
          System.out.println("*** SOLUTION FOUND ***");
          System.out.printf("  Pitch:      %.1f deg%n", Math.toDegrees(sol.pitchRad));
          System.out.printf("  Yaw:        %.1f deg%n", Math.toDegrees(sol.yawRad));
          System.out.printf("  Velocity:   %.2f m/s%n", sol.velocityMps);
          System.out.printf("  RPM:        %.0f%n", sol.rpm);
          System.out.printf("  ToF:        %.3f s%n", sol.timeOfFlightS);
          System.out.printf("  Impact Z:   %.3f m (target: 1.8288 m)%n", sol.impactZM);

          // Verify trajectory
          TrajectoryPoint[] traj =
              solver.simulateTrajectory(
                  sol.yawRad,
                  sol.pitchRad,
                  sol.velocityMps,
                  robotX,
                  robotY,
                  0,
                  0,
                  0,
                  0,
                  0.5,
                  0,
                  0,
                  200);

          if (traj != null && traj.length > 0) {
            double launchX = traj[0].x;
            double launchY = traj[0].y;
            double apexZ = 0, apexDist = 0;
            double zAtTarget = -1, vzAtTarget = 0;

            for (int i = 1; i < traj.length; i++) {
              TrajectoryPoint pt = traj[i];
              TrajectoryPoint prev = traj[i - 1];
              double d =
                  Math.sqrt(
                      (pt.x - launchX) * (pt.x - launchX) + (pt.y - launchY) * (pt.y - launchY));
              double prevD =
                  Math.sqrt(
                      (prev.x - launchX) * (prev.x - launchX)
                          + (prev.y - launchY) * (prev.y - launchY));

              if (pt.z > apexZ) {
                apexZ = pt.z;
                apexDist = d;
              }

              if (zAtTarget < 0 && d >= dist && prevD < dist) {
                double alpha = (dist - prevD) / (d - prevD);
                zAtTarget = prev.z + alpha * (pt.z - prev.z);
                vzAtTarget = prev.vz + alpha * (pt.vz - prev.vz);
              }
            }

            System.out.printf("  Apex:       %.2f m high at %.2f m distance%n", apexZ, apexDist);
            System.out.printf("  Z @ target: %.2f m, vz = %+.2f m/s%n", zAtTarget, vzAtTarget);
            System.out.printf(
                "  Descent:    %s%n", vzAtTarget < 0 ? "YES (vz < 0)" : "NO (ascending)");
          }
        } else {
          System.out.println("NO SOLUTION FOUND");
        }
      }

      System.out.println("\n" + "=".repeat(80));
      System.out.println("DEBUG COMPLETE");

    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
      e.printStackTrace();
    }
  }
}
