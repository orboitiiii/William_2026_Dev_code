package frc.robot.simulation;

import frc.robot.libraries.lib9427.BallisticSolver;
import frc.robot.libraries.lib9427.BallisticSolver.ShootingSolution;
import frc.robot.libraries.lib9427.BallisticSolver.TrajectoryPoint;

/**
 * Advanced ballistic trajectory simulation with robot motion scenarios.
 *
 * <p>Simulates various robot motion states and shows turret compensation behavior:
 *
 * <ul>
 *   <li>Approaching: Robot moving toward target
 *   <li>Retreating: Robot moving away from target
 *   <li>Strafing: Robot moving laterally
 *   <li>Rotating: Robot spinning in place
 *   <li>Stationary: Robot at fixed position (baseline)
 * </ul>
 *
 * <p>For each scenario, displays:
 *
 * <ul>
 *   <li>Trajectory (XZ side view with descent through target)
 *   <li>Robot + turret animation (top view)
 *   <li>Turret yaw compensation over time
 * </ul>
 *
 * @author Team 9427 William
 */
public class BallisticSimulation {

  // === Visualization Settings ===
  private static final int PLOT_WIDTH = 100;
  private static final int PLOT_HEIGHT = 30;
  private static final int TOP_VIEW_SIZE = 40;

  // === Physical Constants ===
  private static final double TARGET_HEIGHT_M = 1.8288; // 72 inches
  private static final double TURRET_HEIGHT_M = 0.5;

  // private static final double ROBOT_SIZE_M = 0.6; // Square robot for
  // visualization

  public static void main(String[] args) {
    printBanner();

    try (BallisticSolver solver = new BallisticSolver()) {
      // Configure solver
      configureStandardShooter(solver);

      // Run motion scenarios
      System.out.println("\n" + "=".repeat(80));
      System.out.println("MOTION SCENARIO SIMULATIONS");
      System.out.println("=".repeat(80));

      // 1. Stationary baseline at various distances
      runScenario(solver, "STATIONARY @ 5m", 5.0, 0.0, 0.0, 0.0, 0.0);
      runScenario(solver, "STATIONARY @ 8m", 8.0, 0.0, 0.0, 0.0, 0.0);
      runScenario(solver, "STATIONARY @ 3m", 3.0, 2.0, 0.0, 0.0, 0.0);

      // 2. Approaching target (moving toward origin)
      runScenario(solver, "APPROACHING @ 2 m/s", 6.0, 0.0, -2.0, 0.0, 0.0);
      runScenario(solver, "APPROACHING @ 3 m/s", 6.0, 0.0, -3.0, 0.0, 0.0);

      // 3. Retreating from target (moving away)
      runScenario(solver, "RETREATING @ 2 m/s", 5.0, 0.0, 2.0, 0.0, 0.0);
      runScenario(solver, "RETREATING @ 3 m/s", 5.0, 0.0, 3.0, 0.0, 0.0);

      // 4. Strafing (lateral motion)
      runScenario(solver, "STRAFING LEFT @ 2 m/s", 5.0, 1.0, 0.0, 2.0, 0.0);
      runScenario(solver, "STRAFING RIGHT @ 2 m/s", 5.0, 1.0, 0.0, -2.0, 0.0);

      // 5. Rotating in place
      runScenario(solver, "ROTATING CCW @ 90 deg/s", 5.0, 2.0, 0.0, 0.0, Math.toRadians(90));
      runScenario(solver, "ROTATING CW @ 90 deg/s", 5.0, 2.0, 0.0, 0.0, Math.toRadians(-90));

      // 6. Combined motion
      runScenario(solver, "APPROACH + STRAFE", 6.0, 2.0, -1.5, 1.0, 0.0);
      runScenario(solver, "RETREAT + ROTATE", 5.0, 0.0, 2.0, 0.0, Math.toRadians(45));

      System.out.println("\n" + "=".repeat(80));
      System.out.println("SIMULATION COMPLETE");
      System.out.println("=".repeat(80));

    } catch (Exception e) {
      System.err.println("Simulation failed: " + e.getMessage());
      e.printStackTrace();
    }
  }

  private static void configureStandardShooter(BallisticSolver solver) {
    solver.setShooterConstraints(
        0, // min RPM
        4000, // max RPM
        0.0508, // wheel radius (2 inch)
        0.15, // min pitch (rad) ~8.6 deg
        1.3, // max pitch (rad) ~74.5 deg
        6.0 // max turret rate (rad/s)
        );
    solver.setTarget(0.0, 0.0, TARGET_HEIGHT_M);
    solver.setTimeStep(0.002); // 2ms for accuracy
  }

  private static void runScenario(
      BallisticSolver solver,
      String scenarioName,
      double robotX,
      double robotY,
      double robotVx,
      double robotVy,
      double robotOmega) {

    System.out.println("\n" + "-".repeat(80));
    System.out.println("SCENARIO: " + scenarioName);
    System.out.println("-".repeat(80));

    // Calculate direction to target for context
    double distToTarget = Math.sqrt(robotX * robotX + robotY * robotY);
    // double angleToTarget = Math.toDegrees(Math.atan2(-robotY, -robotX));

    System.out.printf(
        "Robot Position:   (%.2f, %.2f) m  [Distance: %.2f m]%n", robotX, robotY, distToTarget);
    System.out.printf("Robot Velocity:   (%.2f, %.2f) m/s%n", robotVx, robotVy);
    if (Math.abs(robotOmega) > 0.01) {
      System.out.printf("Robot Rotation:   %.1f deg/s%n", Math.toDegrees(robotOmega));
    }

    // Solve with current turret at 0 (facing forward)
    double turretYaw = 0.0;
    double turretYawRate = 0.0;

    ShootingSolution sol =
        solver.solve(
            robotX,
            robotY,
            robotVx,
            robotVy,
            robotOmega,
            0.0, // heading (robot facing +X)
            TURRET_HEIGHT_M,
            turretYaw,
            turretYawRate,
            0.0, // offset X
            0.0 // offset Y
            );

    if (!sol.valid) {
      System.out.println("\n*** NO VALID SOLUTION FOUND ***");
      System.out.println("The target cannot be reached with current parameters.");
      return;
    }

    // Print solution
    printSolution(sol, robotVx, robotVy, robotOmega);

    // Get trajectory for visualization
    TrajectoryPoint[] trajectory =
        solver.simulateTrajectory(
            sol.yawRad,
            sol.pitchRad,
            sol.velocityMps,
            robotX,
            robotY,
            robotVx,
            robotVy,
            robotOmega,
            0.0,
            TURRET_HEIGHT_M,
            0.0,
            0.0,
            500);

    if (trajectory.length > 0) {
      // Plot XZ side view with descent verification
      plotTrajectoryXZ(trajectory, distToTarget, sol);

      // Plot XY top view with robot and turret
      plotTopView(robotX, robotY, robotVx, robotVy, sol.yawRad, turretYaw, distToTarget);

      // Show turret compensation animation frames
      showTurretCompensation(robotX, robotY, robotVx, robotVy, robotOmega, sol);
    }
  }

  private static void printSolution(
      ShootingSolution sol, double robotVx, double robotVy, double robotOmega) {
    System.out.println("\n--- SHOOTING SOLUTION ---");
    System.out.printf(
        "  Turret Yaw:      %7.2f deg (absolute field angle)%n", Math.toDegrees(sol.yawRad));
    System.out.printf(
        "  Yaw Lead:        %7.2f deg (motion compensation)%n", Math.toDegrees(sol.yawLeadRad));
    System.out.printf("  Pitch:           %7.2f deg%n", Math.toDegrees(sol.pitchRad));
    System.out.printf("  Exit Velocity:   %7.2f m/s%n", sol.velocityMps);
    System.out.printf("  Shooter RPM:     %7.0f%n", sol.rpm);
    System.out.printf("  Time of Flight:  %7.3f s%n", sol.timeOfFlightS);
    System.out.printf("  Turret Travel:   %7.3f s%n", sol.turretTravelTimeS);
    System.out.printf(
        "  Impact Point:    (%.3f, %.3f, %.3f) m%n", sol.impactXM, sol.impactYM, sol.impactZM);

    // Verify descent condition
    System.out.println("\n--- TRAJECTORY VERIFICATION ---");
    double heightAboveTarget = sol.impactZM - TARGET_HEIGHT_M;
    System.out.printf("  Impact Height vs Target: %.3f m (should be ~0)%n", heightAboveTarget);

    // Analyze lead angle compensation
    double robotSpeed = Math.sqrt(robotVx * robotVx + robotVy * robotVy);
    if (robotSpeed > 0.1) {
      double expectedLeadDeg =
          Math.toDegrees(Math.atan(robotSpeed * sol.timeOfFlightS / 5.0)); // rough estimate
      System.out.printf(
          "  Robot Speed: %.2f m/s -> Expected lead: ~%.1f deg%n", robotSpeed, expectedLeadDeg);
    }
  }

  private static void plotTrajectoryXZ(
      TrajectoryPoint[] trajectory, double targetDistance, ShootingSolution sol) {

    System.out.println("\n=== TRAJECTORY (XZ Side View) ===");
    System.out.println("    Shows projectile path from launch to target");
    System.out.println("    *** Trajectory must DESCEND through target ***\n");

    char[][] grid = new char[PLOT_HEIGHT][PLOT_WIDTH];
    for (int z = 0; z < PLOT_HEIGHT; z++) {
      for (int x = 0; x < PLOT_WIDTH; x++) {
        grid[z][x] = ' ';
      }
    }

    // Find bounds
    double xMax = 0, zMax = 0;
    double apexZ = 0;
    int apexIndex = 0;

    for (int i = 0; i < trajectory.length; i++) {
      TrajectoryPoint pt = trajectory[i];
      double dist = Math.sqrt(pt.x * pt.x + pt.y * pt.y);
      xMax = Math.max(xMax, dist);
      zMax = Math.max(zMax, pt.z);
      if (pt.z > apexZ) {
        apexZ = pt.z;
        apexIndex = i;
      }
    }

    xMax = Math.max(xMax, targetDistance * 1.15);
    zMax = Math.max(zMax, TARGET_HEIGHT_M * 1.8);

    // Draw ground
    for (int x = 0; x < PLOT_WIDTH; x++) {
      grid[PLOT_HEIGHT - 1][x] = '=';
    }

    // Draw target height line
    int targetGz = (int) (((zMax - TARGET_HEIGHT_M) / zMax) * (PLOT_HEIGHT - 2));
    targetGz = Math.max(0, Math.min(PLOT_HEIGHT - 2, targetGz));
    int targetGx = (int) ((targetDistance / xMax) * (PLOT_WIDTH - 1));
    targetGx = Math.max(0, Math.min(PLOT_WIDTH - 1, targetGx));

    // Draw target zone (hexagon opening)
    for (int dx = -2; dx <= 2; dx++) {
      int tx = targetGx + dx;
      if (tx >= 0 && tx < PLOT_WIDTH) {
        grid[targetGz][tx] = (dx == -2 || dx == 2) ? '[' : (dx == 0 ? 'O' : '-');
      }
    }

    // Draw trajectory - color code ascending vs descending
    for (int i = 0; i < trajectory.length; i++) {
      TrajectoryPoint pt = trajectory[i];
      double dist = Math.sqrt(pt.x * pt.x + pt.y * pt.y);

      int gx = (int) ((dist / xMax) * (PLOT_WIDTH - 1));
      int gz = (int) (((zMax - pt.z) / zMax) * (PLOT_HEIGHT - 2));

      gx = Math.max(0, Math.min(PLOT_WIDTH - 1, gx));
      gz = Math.max(0, Math.min(PLOT_HEIGHT - 2, gz));

      // Use different characters for ascending vs descending
      char c = (i < apexIndex) ? '^' : 'v'; // ^ ascending, v descending
      if (i == apexIndex) c = 'A'; // Apex
      grid[gz][gx] = c;
    }

    // Mark launch point
    int launchGz = (int) (((zMax - trajectory[0].z) / zMax) * (PLOT_HEIGHT - 2));
    launchGz = Math.max(0, Math.min(PLOT_HEIGHT - 2, launchGz));
    grid[launchGz][0] = '@';

    // Print grid with axis
    System.out.printf("Z(m)%n");
    for (int z = 0; z < PLOT_HEIGHT; z++) {
      double zVal = zMax * (1.0 - (double) z / (PLOT_HEIGHT - 1));
      System.out.printf("%4.1f |", zVal);
      System.out.println(new String(grid[z]));
    }
    System.out.println("     +" + "-".repeat(PLOT_WIDTH));
    System.out.printf("      0%s%.1f m (horizontal distance)%n", " ".repeat(PLOT_WIDTH - 15), xMax);

    // Legend
    System.out.println(
        "\nLegend: @ = Launch, ^ = Ascending, A = Apex, v = Descending, [-O-] = Target");
    System.out.printf(
        "Apex Height: %.2f m at distance %.2f m%n",
        apexZ,
        Math.sqrt(
            trajectory[apexIndex].x * trajectory[apexIndex].x
                + trajectory[apexIndex].y * trajectory[apexIndex].y));
  }

  private static void plotTopView(
      double robotX,
      double robotY,
      double robotVx,
      double robotVy,
      double turretYaw,
      double chassisYaw,
      double distToTarget) {

    System.out.println("\n=== TOP VIEW (XY Plane) ===");
    System.out.println("    Shows robot position, velocity, and turret aim\n");

    char[][] grid = new char[TOP_VIEW_SIZE][TOP_VIEW_SIZE];
    for (int y = 0; y < TOP_VIEW_SIZE; y++) {
      for (int x = 0; x < TOP_VIEW_SIZE; x++) {
        grid[y][x] = '.';
      }
    }

    // Scale: fit robot and target in view
    double scale = (TOP_VIEW_SIZE - 4) / Math.max(distToTarget * 1.2, 10.0);

    // Draw target at origin (center of grid)
    int targetGx = TOP_VIEW_SIZE / 2;
    int targetGy = TOP_VIEW_SIZE / 2;
    grid[targetGy][targetGx] = 'X';
    // Draw hexagon outline
    for (int d = 0; d < 6; d++) {
      double angle = d * Math.PI / 3;
      int hx = targetGx + (int) (2 * Math.cos(angle));
      int hy = targetGy - (int) (2 * Math.sin(angle));
      if (hx >= 0 && hx < TOP_VIEW_SIZE && hy >= 0 && hy < TOP_VIEW_SIZE) {
        grid[hy][hx] = '*';
      }
    }

    // Draw robot
    int robotGx = targetGx + (int) (robotX * scale);
    int robotGy = targetGy - (int) (robotY * scale);
    if (robotGx >= 0 && robotGx < TOP_VIEW_SIZE && robotGy >= 0 && robotGy < TOP_VIEW_SIZE) {
      grid[robotGy][robotGx] = 'R';
    }

    // Draw velocity vector
    double velMag = Math.sqrt(robotVx * robotVx + robotVy * robotVy);
    if (velMag > 0.1) {
      for (int i = 1; i <= 3; i++) {
        int vx = robotGx + (int) (i * (robotVx / velMag) * 1.5);
        int vy = robotGy - (int) (i * (robotVy / velMag) * 1.5);
        if (vx >= 0 && vx < TOP_VIEW_SIZE && vy >= 0 && vy < TOP_VIEW_SIZE) {
          grid[vy][vx] = (i == 3) ? '>' : '-';
        }
      }
    }

    // Draw turret aim line
    for (int i = 1; i <= (int) (distToTarget * scale); i++) {
      int tx = robotGx + (int) (i * Math.cos(turretYaw));
      int ty = robotGy - (int) (i * Math.sin(turretYaw));
      if (tx >= 0 && tx < TOP_VIEW_SIZE && ty >= 0 && ty < TOP_VIEW_SIZE) {
        grid[ty][tx] = (grid[ty][tx] == '.') ? '+' : grid[ty][tx];
      }
    }

    // Print grid
    System.out.println("  Y");
    for (int y = 0; y < TOP_VIEW_SIZE; y++) {
      System.out.print("  |");
      System.out.println(new String(grid[y]));
    }
    System.out.println("  +" + "-".repeat(TOP_VIEW_SIZE) + " X");
    System.out.println("\nLegend: R = Robot, X = Target, + = Turret aim, - -> = Velocity");
  }

  private static void showTurretCompensation(
      double robotX,
      double robotY,
      double robotVx,
      double robotVy,
      double robotOmega,
      ShootingSolution sol) {

    System.out.println("\n=== TURRET COMPENSATION OVER TIME ===");
    System.out.println("    Shows how turret yaw changes as robot moves\n");

    double totalTime = sol.turretTravelTimeS + sol.timeOfFlightS + 0.2;
    int steps = 10;
    double dt = totalTime / steps;

    System.out.println("Time(s)  Robot Pos         Direct Yaw    Compensated Yaw    Delta");
    System.out.println("-".repeat(75));

    for (int i = 0; i <= steps; i++) {
      double t = i * dt;

      // Predict robot position at time t
      double futureX = robotX + robotVx * t;
      double futureY = robotY + robotVy * t;
      // double futureHeading = robotOmega * t;

      // Direct angle to target (no compensation)
      double directYaw = Math.atan2(-futureY, -futureX);

      // Required turret yaw (with compensation, would need to resolve again)
      // For display, we use the solution's yaw which already includes compensation
      double compensatedYaw = sol.yawRad;

      // Difference shows the lead
      double delta = compensatedYaw - directYaw;
      while (delta > Math.PI) delta -= 2 * Math.PI;
      while (delta < -Math.PI) delta += 2 * Math.PI;

      String phase =
          (t < sol.turretTravelTimeS)
              ? "[AIMING]"
              : ((t < sol.turretTravelTimeS + sol.timeOfFlightS) ? "[FLYING]" : "[SCORED]");

      System.out.printf(
          "%5.3f    (%5.2f, %5.2f)    %7.2f deg     %7.2f deg       %+6.2f deg   %s%n",
          t,
          futureX,
          futureY,
          Math.toDegrees(directYaw),
          Math.toDegrees(compensatedYaw),
          Math.toDegrees(delta),
          phase);
    }
  }

  private static void printBanner() {
    System.out.println();
    System.out.println(
        "╔══════════════════════════════════════════════════════════════════════════════╗");
    System.out.println(
        "║                     FRC BALLISTIC SOLVER SIMULATION                          ║");
    System.out.println(
        "║                         Team 9427 - First Principles                         ║");
    System.out.println(
        "╠══════════════════════════════════════════════════════════════════════════════╣");
    System.out.println(
        "║  Physics Model:                                                              ║");
    System.out.println(
        "║    • RK4 Integration (dt=2ms)                                                ║");
    System.out.println(
        "║    • Gravity + Air Drag + Magnus Effect                                      ║");
    System.out.println(
        "║    • Robot motion compensation (translation + rotation)                      ║");
    System.out.println(
        "║    • Turret delay prediction                                                 ║");
    System.out.println(
        "║                                                                              ║");
    System.out.println(
        "║  Trajectory Constraint:                                                      ║");
    System.out.println(
        "║    • MUST descend through target (vz < 0 at impact)                          ║");
    System.out.println(
        "║    • Impact point MUST be inside hexagon boundary                            ║");
    System.out.println(
        "╚══════════════════════════════════════════════════════════════════════════════╝");
    System.out.println();
  }
}
