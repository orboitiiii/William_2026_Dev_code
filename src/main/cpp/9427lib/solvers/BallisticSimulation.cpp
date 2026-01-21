/**
 * @file BallisticSimulation.cpp
 * @brief Standalone C++ trajectory simulation and visualization tool.
 *
 * This file provides a console-based trajectory visualization and CSV export
 * for analyzing ballistic solver behavior. It can be compiled separately as
 * a development/debugging tool.
 *
 * ## Output Formats
 * 1. Console ASCII art visualization (XZ plane)
 * 2. CSV file export for plotting in external tools
 *
 * ## Usage
 * Compile: g++ -std=c++20 -O2 BallisticSimulation.cpp BallisticSolver.cpp -o sim
 * Run: ./sim [distance_m] [target_height_m]
 *
 * @author Team 9427 Control Systems
 * @date 2026-01
 */

#include "BallisticSolver.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <cmath>

using namespace lib9427::solvers;

// ============================================================================
// ASCII Art Trajectory Visualization
// ============================================================================

/**
 * Simple ASCII art trajectory plot in XZ plane.
 *
 * Renders the trajectory as ASCII characters in a fixed-size grid.
 * X-axis = horizontal distance, Z-axis = height.
 */
class AsciiPlotter {
public:
    static constexpr int WIDTH = 80;   // Characters wide
    static constexpr int HEIGHT = 25;  // Characters tall

    void plot(const TrajectoryBuffer& trajectory, double target_distance, double target_height) {
        // Initialize grid with spaces
        std::array<std::array<char, WIDTH>, HEIGHT> grid;
        for (auto& row : grid) {
            row.fill(' ');
        }

        // Find bounds
        double x_min = 0.0, x_max = 0.0;
        double z_min = 0.0, z_max = 0.0;

        for (size_t i = 0; i < trajectory.count; ++i) {
            const auto& pt = trajectory.points[i];
            // Use XY planar distance as "x" for visualization
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            x_max = std::max(x_max, dist);
            z_max = std::max(z_max, pt.z);
        }

        x_max = std::max(x_max, target_distance * 1.1);
        z_max = std::max(z_max, target_height * 1.5);

        // Draw ground line
        for (int x = 0; x < WIDTH; ++x) {
            grid[HEIGHT - 1][x] = '-';
        }

        // Draw trajectory
        for (size_t i = 0; i < trajectory.count; ++i) {
            const auto& pt = trajectory.points[i];
            double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);

            int gx = static_cast<int>((dist / x_max) * (WIDTH - 1));
            int gz = static_cast<int>(((z_max - pt.z) / z_max) * (HEIGHT - 2));

            gx = std::clamp(gx, 0, WIDTH - 1);
            gz = std::clamp(gz, 0, HEIGHT - 2);

            grid[gz][gx] = '*';
        }

        // Draw target
        int target_gx = static_cast<int>((target_distance / x_max) * (WIDTH - 1));
        int target_gz = static_cast<int>(((z_max - target_height) / z_max) * (HEIGHT - 2));
        target_gx = std::clamp(target_gx, 0, WIDTH - 1);
        target_gz = std::clamp(target_gz, 0, HEIGHT - 2);
        grid[target_gz][target_gx] = 'X';

        // Draw launcher position
        int launch_gz = static_cast<int>(((z_max - trajectory.points[0].z) / z_max) * (HEIGHT - 2));
        launch_gz = std::clamp(launch_gz, 0, HEIGHT - 2);
        grid[launch_gz][0] = 'O';

        // Print grid
        std::cout << "\n=== Trajectory Visualization (XZ Plane) ===\n";
        std::cout << "Legend: O=Launch, X=Target, *=Trajectory\n\n";

        for (int z = 0; z < HEIGHT; ++z) {
            for (int x = 0; x < WIDTH; ++x) {
                std::cout << grid[z][x];
            }
            std::cout << '\n';
        }

        // Print axes labels
        std::cout << "X: 0 ";
        for (int i = 0; i < WIDTH - 15; ++i) std::cout << ' ';
        std::cout << std::fixed << std::setprecision(1) << x_max << " m\n";
        std::cout << "Z: 0 - " << z_max << " m\n";
    }
};

// ============================================================================
// CSV Export
// ============================================================================

/**
 * Exports trajectory to CSV for external visualization.
 */
void exportToCsv(const TrajectoryBuffer& trajectory, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing.\n";
        return;
    }

    file << "time,x,y,z,vx,vy,vz,speed,distance\n";

    double launch_x = trajectory.points[0].x;
    double launch_y = trajectory.points[0].y;

    for (size_t i = 0; i < trajectory.count; ++i) {
        const auto& pt = trajectory.points[i];
        double speed = std::sqrt(pt.vx*pt.vx + pt.vy*pt.vy + pt.vz*pt.vz);
        double dist = std::sqrt((pt.x - launch_x)*(pt.x - launch_x) +
                                (pt.y - launch_y)*(pt.y - launch_y));

        file << std::fixed << std::setprecision(6)
             << pt.t << ","
             << pt.x << ","
             << pt.y << ","
             << pt.z << ","
             << pt.vx << ","
             << pt.vy << ","
             << pt.vz << ","
             << speed << ","
             << dist << "\n";
    }

    file.close();
    std::cout << "Trajectory exported to: " << filename << "\n";
}

// ============================================================================
// Moving Robot Simulation
// ============================================================================

/**
 * Simulates shooting while robot is moving.
 */
void simulateMovingRobot(BallisticSolver& solver, double robot_vx, double robot_vy) {
    std::cout << "\n=== Moving Robot Simulation ===\n";
    std::cout << "Robot velocity: (" << robot_vx << ", " << robot_vy << ") m/s\n";

    RobotState robot;
    robot.x_m = 5.0;
    robot.y_m = 2.0;
    robot.vx_mps = robot_vx;
    robot.vy_mps = robot_vy;
    robot.omega_radps = 0.0;
    robot.heading_rad = 0.0;
    robot.turret_height_m = 0.5;
    robot.turret_yaw_rad = 0.0;
    robot.turret_yaw_rate_radps = 0.0;
    robot.turret_offset_x_m = 0.0;
    robot.turret_offset_y_m = 0.0;

    auto solution = solver.solve(robot);

    if (solution.valid) {
        std::cout << "\n--- Solution Found ---\n";
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Yaw:        " << (solution.yaw_rad * 180.0 / PI) << " degrees\n";
        std::cout << "Pitch:      " << (solution.pitch_rad * 180.0 / PI) << " degrees\n";
        std::cout << "Velocity:   " << solution.velocity_mps << " m/s\n";
        std::cout << "RPM:        " << solution.rpm << "\n";
        std::cout << "ToF:        " << solution.time_of_flight_s << " s\n";
        std::cout << "Yaw Lead:   " << (solution.yaw_lead_rad * 180.0 / PI) << " degrees\n";
        std::cout << "Turret Travel: " << solution.turret_travel_time_s << " s\n";
        std::cout << "Impact:     (" << solution.impact_x_m << ", "
                  << solution.impact_y_m << ", " << solution.impact_z_m << ") m\n";

        // Get and visualize trajectory
        const auto& traj = solver.getLastTrajectory();
        AsciiPlotter plotter;
        double distance = std::sqrt(robot.x_m * robot.x_m + robot.y_m * robot.y_m);
        plotter.plot(traj, distance, 1.8288);

        // Export to CSV
        std::string filename = "trajectory_vx" + std::to_string(static_cast<int>(robot_vx * 10))
                             + "_vy" + std::to_string(static_cast<int>(robot_vy * 10)) + ".csv";
        exportToCsv(traj, filename);
    } else {
        std::cout << "No valid solution found!\n";
    }
}

// ============================================================================
// Hexagon Boundary Test
// ============================================================================

void testHexagonBoundary() {
    std::cout << "\n=== Hexagon Boundary Tests ===\n";

    struct TestCase {
        double x, y;
        bool expected;
    };

    std::array<TestCase, 8> tests = {{
        {0.0, 0.0, true},        // Center
        {0.0, 20.0, true},       // Near top edge
        {0.0, 21.0, false},      // Above top edge
        {20.0, 0.0, true},       // Near right corner
        {25.0, 0.0, false},      // Past right corner
        {10.0, 30.0, true},      // Inside upper-right
        {10.0, 35.0, false},     // Outside upper-right diagonal
        {-15.0, -15.0, true}     // Inside lower-left
    }};

    for (const auto& test : tests) {
        bool result = BallisticSolver::isInsideHexagon(test.x, test.y);
        std::cout << "(" << std::setw(6) << test.x << ", "
                  << std::setw(6) << test.y << "): "
                  << (result ? "INSIDE" : "OUTSIDE")
                  << (result == test.expected ? " [OK]" : " [FAIL]") << "\n";
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << " FRC Ballistic Solver Simulation Tool\n";
    std::cout << " Team 9427 - First Principles Physics\n";
    std::cout << "========================================\n";

    // Create and configure solver
    BallisticSolver solver;
    solver.configure(ProjectileConfig::defaultConfig());

    // Set shooter constraints (0-4000 RPM as specified by user)
    ShooterConstraints constraints;
    constraints.min_rpm = 0.0;
    constraints.max_rpm = 4000.0;
    constraints.wheel_radius_m = 0.0508;  // 2 inch wheel
    constraints.pitch_min_rad = 0.1;
    constraints.pitch_max_rad = 1.2;
    constraints.turret_max_rate_radps = 6.0;
    solver.setShooterConstraints(constraints);

    // Set target at field origin, 72 inches high
    solver.setTarget(TargetSpec::defaultSpec());

    // Run tests
    testHexagonBoundary();

    // Static robot test
    std::cout << "\n=== Static Robot Test ===\n";
    simulateMovingRobot(solver, 0.0, 0.0);

    // Moving robot tests
    simulateMovingRobot(solver, 1.0, 0.0);   // Forward motion
    simulateMovingRobot(solver, 0.0, 1.0);   // Lateral motion
    simulateMovingRobot(solver, 1.5, 0.5);   // Diagonal motion

    std::cout << "\n=== Simulation Complete ===\n";
    return 0;
}
