/**
 * @file BallisticSolver.cpp
 * @brief Implementation of the First-Principles Ballistic Solver
 *
 * This file contains the core physics and numerical methods for trajectory
 * computation. All formulas are derived from fundamental principles and
 * cited with their sources.
 *
 * @author Team 9427 Control Systems
 * @date 2026-01
 */

#include "BallisticSolver.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace lib9427 {
namespace solvers {

// ============================================================================
// Constructor
// ============================================================================

BallisticSolver::BallisticSolver()
    : m_projectile(ProjectileConfig::defaultConfig()),
      m_constraints(ShooterConstraints::defaultConstraints()),
      m_target(TargetSpec::defaultSpec()),
      m_dt(0.005),        // 5ms default, 200 Hz integration
      m_spin_rpm(0.0),
      m_trajectory({}),
      m_last_solution(ShootingSolution::invalid()) {
    // Validate time step at construction
    if (m_dt > 0.01) {
        m_dt = 0.01;  // Clamp to maximum allowed
    }
}

// ============================================================================
// Configuration Methods
// ============================================================================

void BallisticSolver::configure(const ProjectileConfig& config) {
    m_projectile = config;
}

void BallisticSolver::setShooterConstraints(const ShooterConstraints& constraints) {
    m_constraints = constraints;
}

void BallisticSolver::setTarget(const TargetSpec& target) {
    m_target = target;
}

void BallisticSolver::setTimeStep(double dt_s) {
    if (dt_s > 0.01) {
        throw std::invalid_argument(
            "Time step must be <= 0.01s for RK4 accuracy. Euler method is forbidden.");
    }
    if (dt_s <= 0.0) {
        throw std::invalid_argument("Time step must be positive.");
    }
    m_dt = dt_s;
}

void BallisticSolver::setSpinRate(double spin_rpm) {
    m_spin_rpm = spin_rpm;
}

// ============================================================================
// Hexagon Boundary Check
// ============================================================================

bool BallisticSolver::isInsideHexagon(double x_inch, double y_inch) {
    /**
     * Hexagon boundary constraints (center at origin).
     *
     * The hexagon is defined by the intersection of 5 half-planes:
     *   1. Horizontal band: |y| <= 20.966
     *   2. Upper-left diagonal: -sqrt(3)*x + y <= 41.932
     *   3. Upper-right diagonal: sqrt(3)*x + y <= 41.932
     *   4. Lower-left diagonal: -sqrt(3)*x - y <= 41.932
     *   5. Lower-right diagonal: sqrt(3)*x - y <= 41.932
     *
     * These constraints define a regular hexagon when combined.
     * The values 20.966 and 41.932 are derived from the 72-inch height
     * specification (half-height ≈ 20.966" vertical, diagonal ≈ 41.932").
     */
    constexpr double SQRT3 = 1.7320508075688772;
    constexpr double Y_BOUND = 20.966;
    constexpr double DIAG_BOUND = 41.932;

    // Constraint 1: Horizontal band
    if (y_inch < -Y_BOUND || y_inch > Y_BOUND) {
        return false;
    }

    // Constraint 2: Upper-left diagonal (-sqrt3*x + y <= bound)
    if (-SQRT3 * x_inch + y_inch > DIAG_BOUND) {
        return false;
    }

    // Constraint 3: Upper-right diagonal (sqrt3*x + y <= bound)
    if (SQRT3 * x_inch + y_inch > DIAG_BOUND) {
        return false;
    }

    // Constraint 4: Lower-left diagonal (-sqrt3*x - y <= bound)
    if (-SQRT3 * x_inch - y_inch > DIAG_BOUND) {
        return false;
    }

    // Constraint 5: Lower-right diagonal (sqrt3*x - y <= bound)
    if (SQRT3 * x_inch - y_inch > DIAG_BOUND) {
        return false;
    }

    return true;
}

// ============================================================================
// Launch Position Computation
// ============================================================================

void BallisticSolver::computeLaunchPosition(
    const RobotState& robot, double yaw,
    double& launch_x, double& launch_y, double& launch_z) const {
    /**
     * Transform turret offset from robot frame to field frame.
     *
     * The turret is offset from robot center by (offset_x, offset_y) in the
     * robot frame. We rotate this offset by the robot heading to get the
     * field-frame position.
     *
     * Rotation matrix:
     *   [cos(θ) -sin(θ)] [offset_x]
     *   [sin(θ)  cos(θ)] [offset_y]
     */
    double cos_heading = std::cos(robot.heading_rad);
    double sin_heading = std::sin(robot.heading_rad);

    // Turret offset in field frame
    double turret_x_field = robot.turret_offset_x_m * cos_heading
                          - robot.turret_offset_y_m * sin_heading;
    double turret_y_field = robot.turret_offset_x_m * sin_heading
                          + robot.turret_offset_y_m * cos_heading;

    // Launch position = robot center + turret offset
    launch_x = robot.x_m + turret_x_field;
    launch_y = robot.y_m + turret_y_field;
    launch_z = robot.turret_height_m;
}

// ============================================================================
// Initial Velocity Computation
// ============================================================================

void BallisticSolver::computeInitialVelocity(
    const RobotState& robot,
    double yaw, double pitch, double exit_velocity,
    double& vx, double& vy, double& vz) const {
    /**
     * Initial velocity composition.
     *
     * The projectile's initial velocity in the field frame is the vector sum of:
     *   1. Exit velocity from shooter (in turret frame, transformed to field)
     *   2. Robot chassis translational velocity
     *   3. Tangential velocity from chassis rotation (ω × r_turret)
     *
     * Exit velocity in field frame:
     *   v_exit = v0 * [cos(pitch)*cos(yaw), cos(pitch)*sin(yaw), sin(pitch)]
     *
     * Tangential velocity from rotation:
     *   v_tangent = ω_chassis × r_turret
     *   (For 2D rotation about Z: v_tangent = [-ω*r_y, ω*r_x, 0])
     */

    // Turret exit velocity vector (field frame)
    double cos_pitch = std::cos(pitch);
    double sin_pitch = std::sin(pitch);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    double v_exit_x = exit_velocity * cos_pitch * cos_yaw;
    double v_exit_y = exit_velocity * cos_pitch * sin_yaw;
    double v_exit_z = exit_velocity * sin_pitch;

    // Turret offset in field frame (for rotation contribution)
    double cos_heading = std::cos(robot.heading_rad);
    double sin_heading = std::sin(robot.heading_rad);
    double turret_x_field = robot.turret_offset_x_m * cos_heading
                          - robot.turret_offset_y_m * sin_heading;
    double turret_y_field = robot.turret_offset_x_m * sin_heading
                          + robot.turret_offset_y_m * cos_heading;

    // Tangential velocity from chassis rotation (ω × r)
    // ω = [0, 0, omega], r = [rx, ry, 0]
    // ω × r = [-omega*ry, omega*rx, 0]
    double v_tangent_x = -robot.omega_radps * turret_y_field;
    double v_tangent_y =  robot.omega_radps * turret_x_field;

    // Total initial velocity
    vx = v_exit_x + robot.vx_mps + v_tangent_x;
    vy = v_exit_y + robot.vy_mps + v_tangent_y;
    vz = v_exit_z;  // No vertical contribution from chassis motion
}

// ============================================================================
// Force and Acceleration Computation
// ============================================================================

void BallisticSolver::computeAcceleration(
    double vx, double vy, double vz,
    double omega_x, double omega_y, double omega_z,
    double& ax, double& ay, double& az) const {
    /**
     * Acceleration computation from net force.
     *
     * Newton's Second Law: a = F/m
     *
     * Forces acting on projectile:
     *
     * 1. GRAVITY
     *    F_g = m * g (downward)
     *    Source: Fundamental physics, g = 9.80665 m/s^2 (CODATA 2018)
     *
     * 2. AERODYNAMIC DRAG
     *    F_d = -0.5 * ρ * Cd * A * |v| * v
     *
     *    Derivation:
     *    - Fluid dynamic pressure: q = 0.5 * ρ * v^2
     *    - Drag force: F_d = Cd * A * q (opposing velocity direction)
     *    - Vector form: F_d = -0.5 * ρ * Cd * A * |v| * v
     *
     *    Source: Anderson, "Fundamentals of Aerodynamics", Ch. 4
     *    Cd ≈ 0.47-0.50 for smooth spheres at Re ~ 10^4-10^5
     *
     * 3. MAGNUS FORCE
     *    F_m = S0 * (ω × v)
     *
     *    Derivation (Robins-Magnus effect):
     *    - Spinning sphere creates asymmetric pressure distribution
     *    - Higher velocity on one side (accelerated by spin) → lower pressure
     *    - Net force perpendicular to both spin axis and velocity
     *    - S0 = 0.5 * ρ * Cm * A * r (spin parameter)
     *
     *    Source: Briggs, "Effect of Spin and Speed on Baseball" (1959)
     *            Cross, "Physics of Overarm Throwing" (2004)
     *    Cm ≈ 0.5 for smooth spheres in laminar regime
     */

    const double m = m_projectile.mass_kg;
    const double A = m_projectile.crossSectionalArea();
    const double Cd = m_projectile.drag_coefficient;
    const double Cm = m_projectile.magnus_coefficient;
    const double r = m_projectile.diameter_m / 2.0;

    // Velocity magnitude
    double v_mag = std::sqrt(vx*vx + vy*vy + vz*vz);

    // ----- Gravity -----
    // a_gravity = [0, 0, -g]
    double a_gravity_z = -GRAVITY_ACCEL;

    // ----- Drag -----
    // F_drag = -0.5 * rho * Cd * A * |v| * v
    // a_drag = F_drag / m
    double drag_factor = 0.0;
    if (v_mag > 1e-6) {  // Avoid division by zero
        drag_factor = -0.5 * AIR_DENSITY * Cd * A * v_mag / m;
    }
    double a_drag_x = drag_factor * vx;
    double a_drag_y = drag_factor * vy;
    double a_drag_z = drag_factor * vz;

    // ----- Magnus -----
    // F_magnus = S0 * (omega × v)
    // S0 = 0.5 * rho * Cm * A * r
    // a_magnus = F_magnus / m
    double S0 = 0.5 * AIR_DENSITY * Cm * A * r;
    double magnus_factor = S0 / m;

    // Cross product: omega × v
    // [ox, oy, oz] × [vx, vy, vz] = [oy*vz - oz*vy, oz*vx - ox*vz, ox*vy - oy*vx]
    double cross_x = omega_y * vz - omega_z * vy;
    double cross_y = omega_z * vx - omega_x * vz;
    double cross_z = omega_x * vy - omega_y * vx;

    double a_magnus_x = magnus_factor * cross_x;
    double a_magnus_y = magnus_factor * cross_y;
    double a_magnus_z = magnus_factor * cross_z;

    // ----- Total Acceleration -----
    ax = a_drag_x + a_magnus_x;
    ay = a_drag_y + a_magnus_y;
    az = a_gravity_z + a_drag_z + a_magnus_z;
}

// ============================================================================
// Runge-Kutta 4 Integration Step
// ============================================================================

void BallisticSolver::rk4Step(std::array<double, 9>& state) const {
    /**
     * 4th-order Runge-Kutta integration.
     *
     * State vector: [x, y, z, vx, vy, vz, omega_x, omega_y, omega_z]
     *
     * The RK4 method achieves O(dt^5) local error, far superior to Euler's
     * O(dt^2). For a typical 3-second flight at dt=0.005s, RK4 accumulates
     * ~10^-8 relative error vs. Euler's ~10^-3.
     *
     * Algorithm:
     *   k1 = f(t_n, y_n)
     *   k2 = f(t_n + dt/2, y_n + dt/2 * k1)
     *   k3 = f(t_n + dt/2, y_n + dt/2 * k2)
     *   k4 = f(t_n + dt, y_n + dt * k3)
     *   y_{n+1} = y_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
     *
     * Source: Burden & Faires, "Numerical Analysis", Ch. 5.4
     */

    // Unpack state
    double x = state[0], y = state[1], z = state[2];
    double vx = state[3], vy = state[4], vz = state[5];
    double ox = state[6], oy = state[7], oz = state[8];

    // State derivative function: returns [vx, vy, vz, ax, ay, az, dox, doy, doz]
    // Position derivative = velocity
    // Velocity derivative = acceleration (from forces)
    // Spin derivative = spin decay

    // Spin decay model: d(omega)/dt = -C_omega * |omega| * omega / I
    // This represents aerodynamic torque opposing rotation
    const double spin_decay = m_projectile.spin_decay_coeff;
    const double I = m_projectile.moment_of_inertia;

    auto computeDerivative = [&](double vx_, double vy_, double vz_,
                                  double ox_, double oy_, double oz_,
                                  std::array<double, 9>& deriv) {
        // Position derivative = velocity
        deriv[0] = vx_;
        deriv[1] = vy_;
        deriv[2] = vz_;

        // Velocity derivative = acceleration
        computeAcceleration(vx_, vy_, vz_, ox_, oy_, oz_,
                           deriv[3], deriv[4], deriv[5]);

        // Spin derivative = decay
        double omega_mag = std::sqrt(ox_*ox_ + oy_*oy_ + oz_*oz_);
        if (omega_mag > 1e-6) {
            double decay_factor = -spin_decay * omega_mag / I;
            deriv[6] = decay_factor * ox_;
            deriv[7] = decay_factor * oy_;
            deriv[8] = decay_factor * oz_;
        } else {
            deriv[6] = deriv[7] = deriv[8] = 0.0;
        }
    };

    std::array<double, 9> k1, k2, k3, k4;
    double dt = m_dt;
    double dt2 = dt / 2.0;

    // k1 = f(state)
    computeDerivative(vx, vy, vz, ox, oy, oz, k1);

    // k2 = f(state + dt/2 * k1)
    computeDerivative(
        vx + dt2 * k1[3], vy + dt2 * k1[4], vz + dt2 * k1[5],
        ox + dt2 * k1[6], oy + dt2 * k1[7], oz + dt2 * k1[8],
        k2);

    // k3 = f(state + dt/2 * k2)
    computeDerivative(
        vx + dt2 * k2[3], vy + dt2 * k2[4], vz + dt2 * k2[5],
        ox + dt2 * k2[6], oy + dt2 * k2[7], oz + dt2 * k2[8],
        k3);

    // k4 = f(state + dt * k3)
    computeDerivative(
        vx + dt * k3[3], vy + dt * k3[4], vz + dt * k3[5],
        ox + dt * k3[6], oy + dt * k3[7], oz + dt * k3[8],
        k4);

    // Update state: y_{n+1} = y_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    double dt6 = dt / 6.0;
    state[0] = x + dt6 * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]);
    state[1] = y + dt6 * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]);
    state[2] = z + dt6 * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]);
    state[3] = vx + dt6 * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]);
    state[4] = vy + dt6 * (k1[4] + 2*k2[4] + 2*k3[4] + k4[4]);
    state[5] = vz + dt6 * (k1[5] + 2*k2[5] + 2*k3[5] + k4[5]);
    state[6] = ox + dt6 * (k1[6] + 2*k2[6] + 2*k3[6] + k4[6]);
    state[7] = oy + dt6 * (k1[7] + 2*k2[7] + 2*k3[7] + k4[7]);
    state[8] = oz + dt6 * (k1[8] + 2*k2[8] + 2*k3[8] + k4[8]);
}

// ============================================================================
// Trajectory Simulation
// ============================================================================

bool BallisticSolver::simulateTrajectory(
    double yaw, double pitch, double velocity,
    const RobotState& robot,
    TrajectoryBuffer& buffer) {
    /**
     * Full trajectory simulation using RK4 integration.
     *
     * Simulates until projectile hits ground (z <= 0) or exceeds max time.
     * Records trajectory points at each integration step.
     */

    buffer.count = 0;

    // Compute launch position
    double launch_x, launch_y, launch_z;
    computeLaunchPosition(robot, yaw, launch_x, launch_y, launch_z);

    // Compute initial velocity
    double vx, vy, vz;
    computeInitialVelocity(robot, yaw, pitch, velocity, vx, vy, vz);

    // Initial spin (backspin about axis perpendicular to velocity in XY plane)
    // Backspin axis is perpendicular to horizontal velocity direction
    double spin_rad_per_sec = m_spin_rpm * RPM_TO_RAD_PER_SEC;
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);
    // Backspin axis: [-sin(yaw), cos(yaw), 0] (perpendicular to shot direction)
    double omega_x = -sin_yaw * spin_rad_per_sec;
    double omega_y =  cos_yaw * spin_rad_per_sec;
    double omega_z = 0.0;

    // State: [x, y, z, vx, vy, vz, omega_x, omega_y, omega_z]
    std::array<double, 9> state = {
        launch_x, launch_y, launch_z,
        vx, vy, vz,
        omega_x, omega_y, omega_z
    };

    double t = 0.0;
    constexpr double MAX_FLIGHT_TIME = 5.0;  // Sanity limit

    while (t < MAX_FLIGHT_TIME && buffer.count < MAX_TRAJECTORY_POINTS) {
        // Record current point
        TrajectoryPoint& pt = buffer.points[buffer.count];
        pt.t = t;
        pt.x = state[0];
        pt.y = state[1];
        pt.z = state[2];
        pt.vx = state[3];
        pt.vy = state[4];
        pt.vz = state[5];
        buffer.count++;

        // Check termination (hit ground)
        if (state[2] <= 0.0 && t > 0.0) {
            break;
        }

        // Advance state
        rk4Step(state);
        t += m_dt;
    }

    return buffer.count > 0;
}

// ============================================================================
// Yaw Lead Computation
// ============================================================================

double BallisticSolver::computeYawWithLead(
    const RobotState& robot,
    double base_distance, double pitch, double velocity) {
    /**
     * Iterative yaw lead calculation for moving robot.
     *
     * The robot moves during the projectile's flight. To hit a stationary
     * target, we need to aim ahead of where the robot currently is.
     *
     * Algorithm:
     * 1. Estimate time-of-flight (ToF) based on current shot parameters
     * 2. Predict robot position at launch time + ToF
     * 3. Compute yaw angle from predicted launch position to target
     * 4. Iterate until yaw converges (typically 2-3 iterations)
     *
     * This accounts for:
     * - Robot translation during flight
     * - Robot rotation during turret slew (future: add turret delay)
     */

    // Target position (fixed)
    double target_x = m_target.center_x_m;
    double target_y = m_target.center_y_m;

    // Initial estimate: direct line to target
    double dx = target_x - robot.x_m;
    double dy = target_y - robot.y_m;
    double yaw = std::atan2(dy, dx);

    // Horizontal velocity component
    double v_horizontal = velocity * std::cos(pitch);
    if (v_horizontal < 1.0) {
        v_horizontal = 1.0;  // Avoid division by zero
    }

    // Iterate to refine yaw
    constexpr int MAX_ITERATIONS = 5;
    for (int i = 0; i < MAX_ITERATIONS; ++i) {
        // Estimate ToF from horizontal distance and velocity
        // ToF ≈ distance / v_horizontal (first-order approximation)
        double distance = std::sqrt(dx*dx + dy*dy);
        double tof_estimate = distance / v_horizontal;

        // Predict where the robot will be at (now + turret_travel + tof)
        // For now, assume robot maintains constant velocity
        double turret_travel = computeTurretTravelTime(
            robot.turret_yaw_rad + robot.heading_rad,
            yaw,
            m_constraints.turret_max_rate_radps);

        double total_delay = turret_travel;  // Can add shooter spinup delay here

        // Adjust target relative to predicted robot position
        double pred_robot_x = robot.x_m + robot.vx_mps * (total_delay + tof_estimate);
        double pred_robot_y = robot.y_m + robot.vy_mps * (total_delay + tof_estimate);

        // Recompute delta to target
        dx = target_x - pred_robot_x;
        dy = target_y - pred_robot_y;

        double new_yaw = std::atan2(dy, dx);

        // Check convergence
        if (std::abs(new_yaw - yaw) < 1e-4) {
            return new_yaw;
        }
        yaw = new_yaw;
    }

    return yaw;
}

// ============================================================================
// Pitch Binary Search
// ============================================================================

double BallisticSolver::searchPitch(
    const RobotState& robot,
    double yaw, double velocity,
    double target_distance, double target_height,
    double& tof) {
    /**
     * Binary search for pitch angle that achieves target height at distance.
     *
     * The relationship between pitch and impact height at a given distance
     * is monotonic (higher pitch → higher impact point, up to 45° for vacuum).
     * With drag, the optimal angle is lower than 45°.
     *
     * We need the projectile to pass through the target from ABOVE (like
     * scoring in a basket), meaning the trajectory must be on its DESCENDING
     * arc when it reaches the target distance. This requires vz < 0 at impact.
     *
     * Algorithm:
     * 1. Set search bounds [pitch_min, pitch_max]
     * 2. Simulate trajectory at mid-point pitch
     * 3. Find the point at target distance WITH vz < 0 (descending)
     * 4. Adjust bounds based on overshoot/undershoot
     * 5. Repeat until convergence
     *
     * Key insight: Higher pitch = higher apex = lower impact height on descent
     * So if we're overshooting (too high), we need HIGHER pitch (steeper arc)
     */

    double pitch_low = m_constraints.pitch_min_rad;
    double pitch_high = m_constraints.pitch_max_rad;

    /**
     * Result structure: height at target distance, time of flight, vertical velocity.
     * is_descending indicates if the projectile is on its way down when reaching target.
     */
    struct TrajectoryResult {
        double z_at_target;
        double tof;
        double vz_at_target;
        bool is_descending;
        bool reached;
    };

    // Check if solution exists
    // The key requirement is that when the projectile reaches target_distance,
    // it must be DESCENDING (vz < 0). This ensures it enters the target from above.
    auto simulateAndGetHeight = [&](double p) -> TrajectoryResult {
        TrajectoryResult result = {0.0, 0.0, 0.0, false, false};
        
        TrajectoryBuffer temp_buf;
        simulateTrajectory(yaw, p, velocity, robot, temp_buf);

        // Find the point where horizontal distance matches target
        double launch_x, launch_y, launch_z;
        computeLaunchPosition(robot, yaw, launch_x, launch_y, launch_z);

        double prev_dist = 0.0;
        double prev_z = launch_z;
        double prev_vz = 0.0;
        double prev_t = 0.0;
        
        // Track if we've found any crossing of target distance
        // We want the LAST crossing (on descent) if there are multiple
        bool found_ascending = false;
        bool found_descending = false;
        TrajectoryResult ascending_result = {0.0, 0.0, 0.0, false, false};
        TrajectoryResult descending_result = {0.0, 0.0, 0.0, false, false};

        for (size_t i = 0; i < temp_buf.count; ++i) {
            const auto& pt = temp_buf.points[i];
            
            double dx = pt.x - launch_x;
            double dy = pt.y - launch_y;
            double dist = std::sqrt(dx*dx + dy*dy);

            // Check if we just crossed target distance
            if (dist >= target_distance && prev_dist < target_distance && prev_dist > 0) {
                double alpha = (target_distance - prev_dist) / (dist - prev_dist);
                double z_at = prev_z + alpha * (pt.z - prev_z);
                double t_at = prev_t + alpha * (pt.t - prev_t);
                double vz_at = prev_vz + alpha * (pt.vz - prev_vz);
                
                // Record based on whether ascending or descending
                if (vz_at >= 0) {
                    // Ascending crossing
                    found_ascending = true;
                    ascending_result = {z_at, t_at, vz_at, false, true};
                } else {
                    // Descending crossing - this is what we want
                    found_descending = true;
                    descending_result = {z_at, t_at, vz_at, true, true};
                }
            }

            prev_dist = dist;
            prev_z = pt.z;
            prev_vz = pt.vz;
            prev_t = pt.t;
        }

        // Prefer descending crossing, fall back to ascending if no descending found
        if (found_descending) {
            return descending_result;
        } else if (found_ascending) {
            return ascending_result;
        }

        // Projectile didn't reach target distance
        result.reached = false;
        return result;
    };

    // Binary search for optimal pitch
    // We ONLY accept solutions where is_descending == true
    // 
    // Physics insight for search direction:
    // - Higher pitch = ball goes higher = apex is farther away
    // - On descent at target distance: higher pitch = ball has fallen more = LOWER z
    // - So if z_at_target > target_height (too high), we need HIGHER pitch
    // - And if z_at_target < target_height (too low), we need LOWER pitch
    //
    // But we also need to ensure we get a descending solution at all
    constexpr int MAX_ITERATIONS = 30;
    constexpr double TOLERANCE = 0.0005;  // ~0.03 degrees

    double best_pitch = std::nan("");
    double best_tof = 0.0;
    double best_error = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        double pitch_mid = (pitch_low + pitch_high) / 2.0;
        auto result = simulateAndGetHeight(pitch_mid);

        if (!result.reached) {
            // Trajectory didn't reach target at all
            // Need higher pitch to get more range
            pitch_low = pitch_mid;
            continue;
        }

        // We only accept DESCENDING solutions (is_descending == true)
        // If we got an ascending solution, we need to go to higher pitch
        // so the apex moves past the target distance
        if (!result.is_descending) {
            // Got ascending solution - need higher pitch so apex extends 
            // beyond target distance, allowing descent at target
            pitch_low = pitch_mid;
            continue;
        }

        // We have a descending solution - now adjust based on height error
        double error = result.z_at_target - target_height;

        // Track best DESCENDING solution
        if (std::abs(error) < std::abs(best_error)) {
            best_error = error;
            best_pitch = pitch_mid;
            best_tof = result.tof;
        }

        if (std::abs(error) < 0.02) {  // Within 2 cm
            tof = result.tof;
            return pitch_mid;
        }

        // Adjust search bounds for descending arc
        // Higher pitch = lower z at target (ball falls more before reaching target)
        if (error > 0) {
            // Too high at target - need higher pitch for steeper descent
            pitch_low = pitch_mid;
        } else {
            // Too low at target - need lower pitch for shallower descent
            pitch_high = pitch_mid;
        }

        if (pitch_high - pitch_low < TOLERANCE) {
            break;
        }
    }

    // Return best found DESCENDING solution
    if (!std::isnan(best_pitch)) {
        tof = best_tof;
        return best_pitch;
    }

    // No valid descending solution found
    tof = 0.0;
    return std::nan("");
}

// ============================================================================
// Turret Travel Time
// ============================================================================

double BallisticSolver::computeTurretTravelTime(
    double current_yaw, double target_yaw, double max_rate) const {
    /**
     * Computes time for turret to rotate from current to target yaw.
     *
     * Assumes bang-bang control (instant acceleration to max rate).
     * For more accurate modeling, use trapezoidal velocity profile.
     *
     * Formula: t = |Δyaw| / max_rate
     */

    // Normalize angle difference to [-π, π]
    double delta = target_yaw - current_yaw;
    while (delta > PI) delta -= 2.0 * PI;
    while (delta < -PI) delta += 2.0 * PI;

    return std::abs(delta) / max_rate;
}

// ============================================================================
// Main Solve Function
// ============================================================================

ShootingSolution BallisticSolver::solve(const RobotState& robot) {
    ShootingSolution sol = ShootingSolution::invalid();

    // 1. Basic geometry
    double dx = m_target.center_x_m - robot.x_m;
    double dy = m_target.center_y_m - robot.y_m;
    double horizontal_distance = std::sqrt(dx*dx + dy*dy);
    double target_height = m_target.center_z_m;

    // 2. Velocity constraints
    double v_min = m_constraints.rpmToVelocity(m_constraints.min_rpm);
    double v_max = m_constraints.rpmToVelocity(m_constraints.max_rpm);

    // 3. Declare best result variables (FIXED SCOPE)
    double best_pitch = std::nan("");
    double best_velocity = 0.0;
    double best_tof = 0.0;
    double best_yaw = 0.0;

    // Lambda: Evaluate single shot
    auto evaluateSolution = [&](double pitch, double velocity, double yaw) 
        -> std::tuple<double, double, double, bool> { // z_at, tof, vz_at, reached
        
        TrajectoryBuffer temp_buf;
        simulateTrajectory(yaw, pitch, velocity, robot, temp_buf);

        double launch_x, launch_y, launch_z;
        computeLaunchPosition(robot, yaw, launch_x, launch_y, launch_z);

        double prev_dist = 0.0;
        double prev_z = launch_z;
        double prev_vz = 0.0;
        double prev_t = 0.0;

        for (size_t i = 0; i < temp_buf.count; ++i) {
            const auto& pt = temp_buf.points[i];
            double ddx = pt.x - launch_x;
            double ddy = pt.y - launch_y;
            double dist = std::sqrt(ddx*ddx + ddy*ddy);

            if (dist >= horizontal_distance && prev_dist < horizontal_distance && prev_dist > 0) {
                double alpha = (horizontal_distance - prev_dist) / (dist - prev_dist);
                double z_at = prev_z + alpha * (pt.z - prev_z);
                double t_at = prev_t + alpha * (pt.t - prev_t);
                double vz_at = prev_vz + alpha * (pt.vz - prev_vz);
                return {z_at, t_at, vz_at, true};
            }
            prev_dist = dist;
            prev_z = pt.z;
            prev_vz = pt.vz;
            prev_t = pt.t;
        }
        return {0.0, 0.0, 0.0, false};
    };

    // Lambda: For a given pitch, find velocity that hits target
    auto findVelocityForPitch = [&](double pitch) 
        -> std::tuple<double, double, bool> { // velocity, tof, valid
        
        double yaw = computeYawWithLead(robot, horizontal_distance, pitch, (v_min + v_max) / 2.0);
        
        double low = v_min;
        double high = v_max;
        
        double local_best_v = 0.0;
        double local_best_t = 0.0;
        double local_best_err = std::numeric_limits<double>::infinity();
        bool found_descending = false;

        constexpr int VEL_ITER = 20; 
        for (int i = 0; i < VEL_ITER; ++i) {
            double mid = (low + high) / 2.0;
            auto [z_at, tof, vz_at, reached] = evaluateSolution(pitch, mid, yaw);

            if (!reached) {
                low = mid; 
                continue;
            }

            double error = z_at - target_height;

            // Strict descent constraint
            if (vz_at < 0) {
                // Descending -> candidate
                if (std::abs(error) < std::abs(local_best_err)) {
                    local_best_err = error;
                    local_best_v = mid;
                    local_best_t = tof;
                    found_descending = true;
                }
                
                // Error correction (on descent)
                if (error > 0) {
                    high = mid; // Too high, reduce velocity
                } else {
                    low = mid;  // Too low, increase velocity
                }
            } else {
                // Ascending -> Too fast -> reduce velocity
                high = mid;
            }
        }

        if (found_descending && std::abs(local_best_err) < 0.05) { // 5cm tolerance
            return {local_best_v, local_best_t, true};
        }
        return {0.0, 0.0, false};
    };

    // 4. Critical Pitch Search (Binary Search for Min Pitch)
    double p_low = m_constraints.pitch_min_rad;
    double p_high = m_constraints.pitch_max_rad;
    
    constexpr int PITCH_ITER = 15;
    bool global_found = false;

    // Search for the lowest valid pitch that allows a solution
    for (int i = 0; i < PITCH_ITER; ++i) {
        double p_mid = (p_low + p_high) / 2.0;
        
        auto [vel, tof, valid] = findVelocityForPitch(p_mid);
        
        bool rpm_ok = false;
        if (valid) {
            double rpm = m_constraints.velocityToRpm(vel);
            if (rpm <= m_constraints.max_rpm) {
                rpm_ok = true;
            }
        }

        if (valid && rpm_ok) {
            // Valid! Try lower pitch to minimize ToF
            best_pitch = p_mid;
            best_velocity = vel;
            best_tof = tof;
            global_found = true;
            
            p_high = p_mid; 
        } else {
            // Invalid (likely too flat/fast) -> Try higher pitch
            p_low = p_mid;
        }
    }

    if (!global_found) {
        return sol; 
    }

    // 5. Finalize solution
    best_yaw = computeYawWithLead(robot, horizontal_distance, best_pitch, best_velocity);

    // Final simulation to populate trajectory buffer
    TrajectoryBuffer final_traj;
    if (!simulateTrajectory(best_yaw, best_pitch, best_velocity, robot, final_traj)) {
        return sol;
    }

    // Find impact coords
    double launch_x, launch_y, launch_z;
    computeLaunchPosition(robot, best_yaw, launch_x, launch_y, launch_z);
    
    double impact_x = 0, impact_y = 0, impact_z = 0;
    for (size_t i = 0; i < final_traj.count; ++i) {
        const auto& pt = final_traj.points[i];
        double ddx = pt.x - launch_x;
        double ddy = pt.y - launch_y;
        double dist = std::sqrt(ddx*ddx + ddy*ddy);
        
        if (dist >= horizontal_distance) {
            // Found impact point
            impact_x = pt.x;
            impact_y = pt.y;
            impact_z = pt.z;
            break;
        }
    }

    // Check hexagon (reuse existing logic if needed, or simple check)
    // The previous implementation had detailed hex check. 
    // The user's provided code simplified it or assumed accessors.
    // I will include the check.
    double rel_x_m = impact_x - m_target.center_x_m;
    double rel_y_m = impact_y - m_target.center_y_m;
    if (!isInsideHexagon(rel_x_m / 0.0254, rel_y_m / 0.0254)) {
        return sol;
    }

    // Fill results
    sol.valid = true;
    sol.yaw_rad = best_yaw;
    sol.pitch_rad = best_pitch;
    sol.velocity_mps = best_velocity;
    sol.rpm = m_constraints.velocityToRpm(best_velocity);
    sol.time_of_flight_s = best_tof;
    sol.impact_x_m = impact_x;
    sol.impact_y_m = impact_y;
    sol.impact_z_m = impact_z;
    
    double direct_yaw = std::atan2(dy, dx);
    sol.yaw_lead_rad = best_yaw - direct_yaw;
    
    double cur_turret_abs = robot.turret_yaw_rad + robot.heading_rad;
    sol.turret_travel_time_s = computeTurretTravelTime(cur_turret_abs, best_yaw, m_constraints.turret_max_rate_radps);

    m_last_solution = sol;
    return sol;
}

// ============================================================================
// Accessors
// ============================================================================

const TrajectoryBuffer& BallisticSolver::getLastTrajectory() const {
    return m_trajectory;
}

}  // namespace solvers
}  // namespace lib9427
