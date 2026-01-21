#pragma once

#include <vector>
#include <array>
#include <Eigen/Dense>
#include "osqp/include/osqp.h"

namespace lib9427 {
namespace solvers {

/**
 * @brief Zero-Allocation Model Predictive Control Solver using OSQP.
 * 
 * Formulates the Trajectory Tracking MPC as a Quadratic Program (QP):
 * 
 * min  sum_{k=0}^{N-1} (x_k^T Q x_k + u_k^T R u_k) + x_N^T Q_N x_N
 * s.t. x_{k+1} = A_k x_k + B_k u_k
 *      u_min <= u_k <= u_max
 *      x_min <= u_k <= x_max
 * 
 * The solver pre-allocates all necessary OSQP structures. 
 * During execution, it updates the linear constraints matrix (A_sparse) and bounds (l, u)
 * directly to avoid memory allocation.
 */
class MPCSolver {
public:
    struct Config {
        int horizon;
        double dt;
        Eigen::Vector3d Q;     // State weights [x, y, theta]
        Eigen::Vector3d R;     // Input weights [vx, vy, omega]
        Eigen::Vector3d Q_final; 
        
        // Limits
        Eigen::Vector3d input_min;
        Eigen::Vector3d input_max;
        Eigen::Vector3d state_min;
        Eigen::Vector3d state_max;
    };

    MPCSolver(const Config& config);
    ~MPCSolver();

    /**
     * @brief Solves the MPC problem for the given reference and dynamics.
     * 
     * @param current_state Robot current state [x, y, theta] (3 doubles)
     * @param linearized_matrices Flattened array of A/B matrices from Java (18 * N doubles)
     * @param ref_trajectory Flattened reference trajectory [x, y, theta, vx, vy, omega...] (Usually just state ref needed for QP q vector)
     *                       Wait, for Error-State MPC (dx), the reference is the origin. 
     *                       Standard formulation: min (x-xref)^T Q (x-xref) 
     *                       = min x^T Q x - 2 xref^T Q x ...
     *                       We need reference states for the linear cost term q.
     * @param output_u Buffer to store the first control action [vx, vy, omega]
     * @return int 0 if successful, OSQP error code otherwise.
     */
    int Solve(const double* current_state, 
              const double* linearized_matrices, 
              const double* ref_states, // [x0, y0, theta0, x1, y1...]
              double* output_u);

private:
    Config config_;
    
    // OSQP Workspace
    OSQPSettings* settings_;
    OSQPData* data_;
    OSQPWorkspace* work_;

    // Problem Dimensions
    int nx_; // Number of states (3)
    int nu_; // Number of inputs (3)
    int n_vars_; // Total variables (N * (nx + nu) + nx) or similar formulation
    int n_constraints_; // Total constraints

    // Internal Buffers for CSC Matrix construction (Zero Allocation)
    // We store the values (Ax) separate from indices (Ai, Ap) which are constant.
    std::vector<c_float> P_x_;
    std::vector<c_int>   P_i_;
    std::vector<c_int>   P_p_;

    std::vector<c_float> A_x_;
    std::vector<c_int>   A_i_;
    std::vector<c_int>   A_p_;

    std::vector<c_float> q_;
    std::vector<c_float> l_;
    std::vector<c_float> u_;

    bool is_initialized_ = false;

    // Helper to build sparsity pattern once
    void BuildSparsityPattern();
    void UpdateProblem(const double* x0, const double* lin_mats, const double* ref_states);
};

} // namespace solvers
} // namespace lib9427
