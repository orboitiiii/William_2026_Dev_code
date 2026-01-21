/**
 * @file BallisticSolver.h
 * @brief First-Principles Ballistic Solver for FRC Shooting Mechanisms
 *
 * This solver computes optimal turret yaw, pitch, and launch velocity for a
 * moving Swerve robot to score projectiles into a hexagonal target. All
 * calculations are derived from fundamental physics equations—no empirical
 * curve fits or lookup tables.
 *
 * ## Physical Model
 * The projectile trajectory is governed by the 3D equation of motion:
 *   m * dv/dt = F_gravity + F_drag + F_magnus
 *
 * Where:
 *   - F_gravity = m * g (constant, downward)
 *   - F_drag = -0.5 * rho * Cd * A * |v| * v (opposes velocity)
 *   - F_magnus = S0 * (omega x v) (perpendicular to spin and velocity)
 *
 * ## Algorithm
 * 1. Geometric yaw lead angle calculation (iterative refinement)
 * 2. Binary search for pitch angle with RK4 trajectory simulation
 * 3. Turret rotation delay compensation via prediction
 * 4. Select minimum Time-of-Flight solution (direct shot preferred)
 *
 * ## Real-Time Guarantees
 * - Zero dynamic memory allocation in solve() hot path
 * - All arrays use std::array with compile-time sizes
 * - Typical solve time: < 500 microseconds on roboRIO
 *
 * @author Team 9427 Control Systems
 * @date 2026-01
 *
 * @see "Fundamentals of Aerodynamics" - J.D. Anderson (drag/Magnus derivations)
 * @see "Classical Mechanics" - Goldstein (rigid body dynamics)
 */

#pragma once

#include <array>
#include <cmath>
#include <limits>

namespace lib9427 {
namespace solvers {

// ============================================================================
// Physical Constants
// ============================================================================

/**
 * Mathematical constant Pi.
 * Defined directly for MSVC compatibility (M_PI is not standard C++).
 */
constexpr double PI = 3.14159265358979323846;

/**
 * Standard gravitational acceleration at Earth's surface.
 * Source: CODATA 2018 recommended value.
 * Units: m/s^2
 */
constexpr double GRAVITY_ACCEL = 9.80665;

/**
 * Air density at sea level, 15C, 101.325 kPa (ISA standard atmosphere).
 * Source: International Standard Atmosphere model.
 * Units: kg/m^3
 */
constexpr double AIR_DENSITY = 1.225;

/**
 * Conversion factor: Revolutions Per Minute to Radians Per Second.
 * omega_rad = omega_rpm * RPM_TO_RAD_PER_SEC
 */
constexpr double RPM_TO_RAD_PER_SEC = 2.0 * PI / 60.0;

/**
 * Numerical tolerance for convergence checks.
 */
constexpr double SOLVER_TOLERANCE = 1e-6;

// ============================================================================
// Data Structures
// ============================================================================

/**
 * Projectile physical configuration.
 *
 * All parameters represent the physical properties of the game piece being
 * launched. These should be measured or obtained from game manual specs.
 */
struct ProjectileConfig {
    double mass_kg;              ///< Mass [kg]. Nominal: 0.215 kg
    double diameter_m;           ///< Diameter [m]. Nominal: 0.150 m
    double drag_coefficient;     ///< Drag coefficient Cd. Nominal: 0.485
    double magnus_coefficient;   ///< Magnus coefficient Cm. Nominal: 0.5
    double moment_of_inertia;    ///< Rotational inertia I [kg*m^2]. Nominal: 0.0024
    double spin_decay_coeff;     ///< Rotational drag coefficient. Nominal: 0.001

    /**
     * Computes cross-sectional area from diameter.
     * Formula: A = pi * (d/2)^2
     * @return Cross-sectional area [m^2]
     */
    [[nodiscard]] double crossSectionalArea() const {
        double r = diameter_m / 2.0;
        return PI * r * r;
    }

    /**
     * Default configuration for 2026 game piece.
     */
    static ProjectileConfig defaultConfig() {
        return {
            .mass_kg = 0.215,
            .diameter_m = 0.150,
            .drag_coefficient = 0.485,
            .magnus_coefficient = 0.50,
            .moment_of_inertia = 0.0024,
            .spin_decay_coeff = 0.001
        };
    }
};

/**
 * Robot kinematic state at the moment of shooting.
 *
 * All values are in the field coordinate frame (origin at field center,
 * X toward opponent alliance wall, Y toward left, Z up).
 */
struct RobotState {
    // --- Position ---
    double x_m;                  ///< Robot X position [m]
    double y_m;                  ///< Robot Y position [m]

    // --- Translational Velocity ---
    double vx_mps;               ///< X velocity [m/s]
    double vy_mps;               ///< Y velocity [m/s]

    // --- Rotational State ---
    double omega_radps;          ///< Chassis angular velocity, CCW positive [rad/s]
    double heading_rad;          ///< Chassis heading (yaw) [rad]

    // --- Turret Configuration ---
    double turret_height_m;      ///< Turret pivot height above ground [m]
    double turret_yaw_rad;       ///< Current turret yaw relative to chassis [rad]
    double turret_yaw_rate_radps;///< Turret yaw angular velocity [rad/s]
    double turret_offset_x_m;    ///< Turret X offset from robot center [m]
    double turret_offset_y_m;    ///< Turret Y offset from robot center [m]
};

/**
 * Target specification.
 *
 * The hexagonal target is defined by its center position and the constraint
 * equations that bound its interior.
 */
struct TargetSpec {
    double center_x_m;           ///< Target center X [m]
    double center_y_m;           ///< Target center Y [m]
    double center_z_m;           ///< Target center height [m]. Nominal: 1.8288 m (72 in)

    // Hexagon boundary constants (in inches, relative to center)
    // See implementation for constraint equations.
    static constexpr double HEXAGON_Y_BOUND = 20.966;    ///< |y| <= 20.966 in
    static constexpr double HEXAGON_DIAG_BOUND = 41.932; ///< diagonal constraints
    static constexpr double SQRT3 = 1.7320508075688772;  ///< sqrt(3)
    static constexpr double INCH_TO_METER = 0.0254;

    /**
     * Default target at field origin, 72 inches high.
     */
    static TargetSpec defaultSpec() {
        return {
            .center_x_m = 0.0,
            .center_y_m = 0.0,
            .center_z_m = 72.0 * INCH_TO_METER  // 1.8288 m
        };
    }
};

/**
 * Shooter mechanism constraints.
 */
struct ShooterConstraints {
    double min_rpm;              ///< Minimum shooter RPM. Nominal: 0
    double max_rpm;              ///< Maximum shooter RPM. Nominal: 4000
    double wheel_radius_m;       ///< Shooter wheel radius [m]
    double pitch_min_rad;        ///< Minimum pitch angle [rad]
    double pitch_max_rad;        ///< Maximum pitch angle [rad]
    double turret_max_rate_radps;///< Max turret slew rate [rad/s]

    /**
     * Converts RPM to exit velocity using wheel radius.
     * Formula: v = omega * r = (RPM * 2*pi/60) * r
     *
     * Note: This assumes 1:1 contact ratio. Real shooter may have slip.
     * @param rpm Wheel speed in RPM
     * @return Exit velocity [m/s]
     */
    [[nodiscard]] double rpmToVelocity(double rpm) const {
        return rpm * RPM_TO_RAD_PER_SEC * wheel_radius_m;
    }

    /**
     * Converts exit velocity to RPM.
     * @param velocity Exit velocity [m/s]
     * @return Wheel speed [RPM]
     */
    [[nodiscard]] double velocityToRpm(double velocity) const {
        return velocity / (RPM_TO_RAD_PER_SEC * wheel_radius_m);
    }

    /**
     * Default constraints.
     */
    static ShooterConstraints defaultConstraints() {
        return {
            .min_rpm = 0.0,
            .max_rpm = 4000.0,
            .wheel_radius_m = 0.0508,  // 2 inch wheel
            .pitch_min_rad = 0.1,      // ~5.7 degrees
            .pitch_max_rad = 1.2,      // ~68.8 degrees
            .turret_max_rate_radps = 6.0  // ~344 deg/s
        };
    }
};

/**
 * Computed shooting solution.
 *
 * All angles are in the turret-relative frame.
 */
struct ShootingSolution {
    bool valid;                  ///< True if solution was found

    double yaw_rad;              ///< Turret yaw angle (field-absolute) [rad]
    double pitch_rad;            ///< Launch pitch angle [rad]
    double velocity_mps;         ///< Launch exit velocity [m/s]
    double rpm;                  ///< Corresponding shooter RPM

    double time_of_flight_s;     ///< Predicted flight time [s]
    double impact_x_m;           ///< Predicted impact X [m]
    double impact_y_m;           ///< Predicted impact Y [m]
    double impact_z_m;           ///< Predicted impact Z [m]

    double yaw_lead_rad;         ///< Lead angle added for motion compensation [rad]
    double turret_travel_time_s; ///< Time for turret to reach target yaw [s]

    /**
     * Invalid/no-solution marker.
     */
    static ShootingSolution invalid() {
        ShootingSolution sol;
        sol.valid = false;
        sol.yaw_rad = 0.0;
        sol.pitch_rad = 0.0;
        sol.velocity_mps = 0.0;
        sol.rpm = 0.0;
        sol.time_of_flight_s = 0.0;
        sol.impact_x_m = 0.0;
        sol.impact_y_m = 0.0;
        sol.impact_z_m = 0.0;
        sol.yaw_lead_rad = 0.0;
        sol.turret_travel_time_s = 0.0;
        return sol;
    }
};

/**
 * 3D trajectory point for visualization.
 */
struct TrajectoryPoint {
    double t;    ///< Time since launch [s]
    double x;    ///< X position [m]
    double y;    ///< Y position [m]
    double z;    ///< Z position [m]
    double vx;   ///< X velocity [m/s]
    double vy;   ///< Y velocity [m/s]
    double vz;   ///< Z velocity [m/s]
};

// Maximum trajectory points to avoid dynamic allocation
constexpr size_t MAX_TRAJECTORY_POINTS = 500;

/**
 * Full trajectory buffer (pre-allocated, no heap).
 */
struct TrajectoryBuffer {
    std::array<TrajectoryPoint, MAX_TRAJECTORY_POINTS> points;
    size_t count;  ///< Number of valid points
};

// ============================================================================
// Ballistic Solver Class
// ============================================================================

/**
 * First-principles ballistic solver for FRC shooting.
 *
 * This class implements a complete 3D ballistic trajectory solver that
 * accounts for:
 *   - Gravitational acceleration
 *   - Aerodynamic drag (quadratic velocity dependence)
 *   - Magnus effect (spin-induced lift)
 *   - Robot translational and rotational motion
 *   - Turret slew delay compensation
 *
 * ## Usage Example
 * ```cpp
 * BallisticSolver solver;
 * solver.configure(ProjectileConfig::defaultConfig());
 * solver.setShooterConstraints(ShooterConstraints::defaultConstraints());
 * solver.setTarget(TargetSpec::defaultSpec());
 *
 * RobotState robot = {...};
 * ShootingSolution sol = solver.solve(robot);
 * if (sol.valid) {
 *     // Apply sol.yaw_rad, sol.pitch_rad, sol.rpm to mechanism
 * }
 * ```
 *
 * ## Thread Safety
 * This class is NOT thread-safe. Each thread should use its own instance.
 */
class BallisticSolver {
public:
    BallisticSolver();

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    /**
     * Sets projectile physical parameters.
     * @param config Projectile configuration
     */
    void configure(const ProjectileConfig& config);

    /**
     * Sets shooter mechanism constraints.
     * @param constraints Shooter constraints
     */
    void setShooterConstraints(const ShooterConstraints& constraints);

    /**
     * Sets target specification.
     * @param target Target spec
     */
    void setTarget(const TargetSpec& target);

    /**
     * Sets integration time step.
     * @param dt_s Time step in seconds. Must be <= 0.01s.
     * @throws std::invalid_argument if dt > 0.01s
     */
    void setTimeStep(double dt_s);

    /**
     * Sets projectile spin rate (backspin RPM).
     * @param spin_rpm Spin rate in RPM. Positive = backspin.
     */
    void setSpinRate(double spin_rpm);

    // -------------------------------------------------------------------------
    // Core Solving
    // -------------------------------------------------------------------------

    /**
     * Computes optimal shooting solution for current robot state.
     *
     * This is the main entry point. Given the robot's position, velocity,
     * and turret state, it computes yaw, pitch, and velocity to hit the target.
     *
     * @param robot Current robot kinematic state
     * @return Shooting solution (check .valid before using)
     *
     * ## Algorithm
     * 1. Compute initial yaw guess (direct line to target)
     * 2. Iteratively refine yaw with motion lead compensation
     * 3. Binary search pitch for trajectory that hits target height
     * 4. Validate hit lands inside hexagon boundary
     * 5. Compute turret travel time and adjust for delay
     */
    ShootingSolution solve(const RobotState& robot);

    /**
     * Simulates trajectory with given shooting parameters.
     *
     * @param yaw Turret yaw (absolute field angle) [rad]
     * @param pitch Launch pitch [rad]
     * @param velocity Exit velocity [m/s]
     * @param robot Robot state at launch
     * @param[out] buffer Trajectory buffer to fill
     * @return True if trajectory was computed successfully
     */
    bool simulateTrajectory(
        double yaw, double pitch, double velocity,
        const RobotState& robot,
        TrajectoryBuffer& buffer);

    // -------------------------------------------------------------------------
    // Trajectory Queries
    // -------------------------------------------------------------------------

    /**
     * Gets the last computed trajectory for visualization.
     * @return Const reference to trajectory buffer
     */
    [[nodiscard]] const TrajectoryBuffer& getLastTrajectory() const;

    /**
     * Checks if a 2D point (x, y in inches) is inside the hexagon.
     *
     * The hexagon is defined by 5 linear constraints:
     *   1. -20.966 <= y <= 20.966
     *   2. -sqrt(3)*x + y <= 41.932
     *   3.  sqrt(3)*x + y <= 41.932
     *   4. -sqrt(3)*x - y <= 41.932
     *   5.  sqrt(3)*x - y <= 41.932
     *
     * @param x_inch X coordinate relative to hexagon center [inches]
     * @param y_inch Y coordinate relative to hexagon center [inches]
     * @return True if point is inside hexagon
     */
    [[nodiscard]] static bool isInsideHexagon(double x_inch, double y_inch);

private:
    // -------------------------------------------------------------------------
    // Internal Helpers
    // -------------------------------------------------------------------------

    /**
     * Computes turret launch position in field coordinates.
     *
     * Accounts for turret offset from robot center and current heading.
     *
     * @param robot Robot state
     * @param yaw Turret yaw (absolute) [rad]
     * @param[out] launch_x Launch X position [m]
     * @param[out] launch_y Launch Y position [m]
     * @param[out] launch_z Launch Z position [m]
     */
    void computeLaunchPosition(
        const RobotState& robot, double yaw,
        double& launch_x, double& launch_y, double& launch_z) const;

    /**
     * Computes initial projectile velocity in field frame.
     *
     * Combines:
     *   - Turret exit velocity vector
     *   - Robot chassis velocity
     *   - Cross-product of robot angular velocity and turret position
     *
     * @param robot Robot state
     * @param yaw Launch yaw [rad]
     * @param pitch Launch pitch [rad]
     * @param exit_velocity Exit speed [m/s]
     * @param[out] vx, vy, vz Velocity components [m/s]
     */
    void computeInitialVelocity(
        const RobotState& robot,
        double yaw, double pitch, double exit_velocity,
        double& vx, double& vy, double& vz) const;

    /**
     * Computes forces on projectile at current state.
     *
     * Forces computed:
     *   1. Gravity: F_g = [0, 0, -m*g]
     *   2. Drag: F_d = -0.5 * rho * Cd * A * |v| * v
     *   3. Magnus: F_m = S0 * (omega x v)
     *
     * @param vx, vy, vz Current velocity [m/s]
     * @param omega_x, omega_y, omega_z Current spin [rad/s]
     * @param[out] ax, ay, az Acceleration components [m/s^2]
     */
    void computeAcceleration(
        double vx, double vy, double vz,
        double omega_x, double omega_y, double omega_z,
        double& ax, double& ay, double& az) const;

    /**
     * Runge-Kutta 4th order integration step.
     *
     * Advances the state by one time step using RK4:
     *   k1 = f(x_n)
     *   k2 = f(x_n + dt/2 * k1)
     *   k3 = f(x_n + dt/2 * k2)
     *   k4 = f(x_n + dt * k3)
     *   x_{n+1} = x_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
     *
     * @param[in,out] state Current state [x,y,z,vx,vy,vz,ox,oy,oz]
     */
    void rk4Step(std::array<double, 9>& state) const;

    /**
     * Computes yaw lead angle for moving target/robot.
     *
     * Uses iterative refinement:
     *   1. Estimate time of flight
     *   2. Predict where robot will be at impact time
     *   3. Compute yaw to that predicted position
     *   4. Repeat until convergence
     *
     * @param robot Robot state
     * @param base_distance Horizontal distance to target [m]
     * @param pitch Estimated pitch [rad]
     * @param velocity Exit velocity [m/s]
     * @return Yaw angle (field-absolute) [rad]
     */
    double computeYawWithLead(
        const RobotState& robot,
        double base_distance, double pitch, double velocity);

    /**
     * Binary search for pitch angle that hits target height.
     *
     * @param robot Robot state
     * @param yaw Launch yaw [rad]
     * @param velocity Exit velocity [m/s]
     * @param target_distance Horizontal distance to target [m]
     * @param target_height Target Z height [m]
     * @param[out] tof Time of flight to target [s]
     * @return Pitch angle [rad], or NaN if no solution
     */
    double searchPitch(
        const RobotState& robot,
        double yaw, double velocity,
        double target_distance, double target_height,
        double& tof);

    /**
     * Computes turret rotation time to reach target yaw.
     *
     * @param current_yaw Current turret yaw [rad]
     * @param target_yaw Target turret yaw [rad]
     * @param max_rate Max slew rate [rad/s]
     * @return Time to reach target [s]
     */
    double computeTurretTravelTime(
        double current_yaw, double target_yaw, double max_rate) const;

    // -------------------------------------------------------------------------
    // Member Variables
    // -------------------------------------------------------------------------

    ProjectileConfig m_projectile;
    ShooterConstraints m_constraints;
    TargetSpec m_target;

    double m_dt;                 // Integration time step [s]
    double m_spin_rpm;           // Backspin rate [RPM]

    TrajectoryBuffer m_trajectory;  // Last computed trajectory
    ShootingSolution m_last_solution; // Last valid solution for warm-starting
};

}  // namespace solvers
}  // namespace lib9427
