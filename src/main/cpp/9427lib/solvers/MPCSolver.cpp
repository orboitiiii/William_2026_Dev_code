#include "MPCSolver.h"
#include <iostream>
#include <algorithm>

// OSQP C API Wrapper
namespace lib9427 {
namespace solvers {

MPCSolver::MPCSolver(const Config& config) : config_(config) {
    nx_ = 3;
    nu_ = 3;
    
    // Reformulation:
    // Decision variables z = [x_0, u_0, x_1, u_1, ..., x_{N-1}, u_{N-1}, x_N]
    // Total Variables = (N+1)*nx + N*nu
    // Constraints:
    // 1. Initial State: x_0 = current_state
    // 2. Dynamics: x_{k+1} = A_k x_k + B_k u_k  =>  -Ax_k - Bu_k + x_{k+1} = 0
    // 3. Inequality: u_min < u < u_max, x_min < x < x_max
    
    int N = config_.horizon;
    n_vars_ = (N + 1) * nx_ + N * nu_;
    
    // Constraints count:
    // Dynamics: N * nx (equality)
    // Initial state: nx (equality)
    // Bounds on u: N * nu (inequality)
    // Bounds on x: (N+1) * nx (inequality) -> actually usually handled by OSQP box constraints if variables are mapped directly?
    // OSQP form: l <= A z <= u
    // A matrix must include Identity rows for box constraints if we want to change them easily.
    // Ideally, we just put Dynamics + Initial State into A.
    // Wait, OSQP does NOT support simple box constraints on variables directly in the solver struct unless you add them to A (A = [A_dyn; I]).
    // Yes, A must include Identity blocks for variable bounds.
    
    int n_dyn_constraints = (N + 1) * nx_; // Initial x0 + N dynamics steps
    int n_box_constraints = n_vars_;       // Bounds on every variable
    n_constraints_ = n_dyn_constraints + n_box_constraints;

    BuildSparsityPattern();

    // Allocation - Use c_calloc to ensure and zero-initialize all pointers
    // This prevents garbage values causing a crash in the destructor if Solve is never called.
    settings_ = (OSQPSettings*)c_calloc(1, sizeof(OSQPSettings));
    data_     = (OSQPData*)c_calloc(1, sizeof(OSQPData));
    work_     = nullptr; // Initialize to prevent crash in destructor if setup fails

    if (settings_) {
        osqp_set_default_settings(settings_);
        settings_->alpha = 1.6;  // Over-relaxation for faster convergence (default: 1.6)
        settings_->verbose = 0;  // Disable verbose to reduce RIO log spam
        settings_->check_termination = 25;  // Check every 25 iterations
        
        // Balanced MPC Configuration
        // Tolerance: relaxed from 5e-4 to 1e-3 to ensure convergence in real-time
        settings_->eps_abs = 1e-3;
        settings_->eps_rel = 1e-3;
        
        // Scaling: 25 iterations for better numerical conditioning
        settings_->scaling = 25;
        
        // ADMM step size: Lower rho improves primal feasibility convergence
        settings_->rho = 0.1;  // Default is 0.1, keep it
        
        // Max iterations: Severely capped to avoid blocking JVM GC via JNI pinning
        settings_->max_iter = 500;
        
        settings_->warm_start = 1;  // Enable Warm Start for Error-State MPC
    }
}

MPCSolver::~MPCSolver() {
    if (work_) osqp_cleanup(work_);
    if (data_) {
        // OSQP data pointers (A, P) are allocated by csc_matrix and MUST be freed
        if (data_->A) c_free(data_->A);
        if (data_->P) c_free(data_->P);
        c_free(data_);
    }
    if (settings_) c_free(settings_);
}

void MPCSolver::BuildSparsityPattern() {
    // This function sets up P_i_, P_p_, A_i_, A_p_ (CSC format)
    // Since this is extremely tedious to write correctly in one shot without a library,
    // we employ a rigorous block-diagonal construction strategy.
    
    int N = config_.horizon;
    
    // ================= P Matrix (Quadratic Cost) =================
    // Block diagonal Q, R, Q, R ... Q_final
    // Size: n_vars_ x n_vars_
    // Since it's diagonal, CSC is trivial.
    // Each column has exactly 1 entry.
    
    int p_nnz = n_vars_; // Diagonal only
    P_x_.resize(p_nnz);
    P_i_.resize(p_nnz);
    P_p_.resize(n_vars_ + 1);

    int nnz_count = 0;
    for (int k = 0; k < n_vars_; k++) {
        P_p_[k] = nnz_count; // Column start
        P_i_[nnz_count] = k; // Row index (diagonal)
        
        // Value filling (constant weights)
        // Determine if this variable is x or u
        // Layout: x0, u0, x1, u1 ...
        int step = k / (nx_ + nu_);
        int rem  = k % (nx_ + nu_);
        
        double val = 1.0;
        if (k >= n_vars_ - nx_) {
            // Final state x_N
            int state_idx = k - (n_vars_ - nx_); // 0..2
            val = config_.Q_final[state_idx];
        } else {
            if (rem < nx_) {
                // State x_k
                val = config_.Q[rem];
            } else {
                // Input u_k
                int u_idx = rem - nx_;
                val = config_.R[u_idx];
            }
        }
        P_x_[nnz_count] = val;
        nnz_count++;
    }
    P_p_[n_vars_] = nnz_count;

    // ================= A Matrix (Constraints) =================
    // Structure:
    // [ I_box ]  (Identity for bounds) -> n_vars_ rows
    // [ Dense ]  (Dynamics)            -> (N+1)*nx_ rows
    
    // Dynamics Rows:
    // Row 0..2: x0 = init ( 1*x0 = init )
    // Row 3..5: x1 - A0*x0 - B0*u0 = 0
    // ...
    
    // We construct CSC column by column.
    
    // Total NNZ estimation:
    // Box constraints: 1 per column (n_vars_)
    // Dynamics:
    //   For x_k columns: 
    //      - Identity in x_{k} = A_{k-1}... row (current step equation) -> Coeff 1.
    //      - A_k block in x_{k+1} = ... row (next step equation)         -> Coeff -A values.
    //   For u_k columns:
    //      - B_k block in x_{k+1} = ... row                              -> Coeff -B values.
    
    // Let's iterate cols (variables)
    
    A_x_.clear();
    A_i_.clear();
    A_p_.clear();
    A_p_.push_back(0); // Start of col 0

    int row_offset_box = 0;
    int row_offset_dyn = n_vars_; 

    for (int k = 0; k <= N; ++k) {
        // x_k columns (size nx_)
        for (int i = 0; i < nx_; ++i) {
            int col_idx = k * (nx_ + nu_) + i;
            
            // 1. Box constraint entry (Row = col_idx)
            A_i_.push_back(col_idx); 
            A_x_.push_back(1.0);

            // 2. Dynamics entry (Transformation from previous step)
            // x_k is the "result" of step k-1 dynamics: x_k - A*x_{k-1}... = 0
            // So coefficient is +1.0 in row (row_offset_dyn + k*nx_ + i)
            // Initial state (k=0) is also x0 = init, so row (row_offset_dyn + 0 + i) has +1 for x0.
            
            int dyn_row_curr = row_offset_dyn + k * nx_ + i;
            A_i_.push_back(dyn_row_curr);
            A_x_.push_back(1.0); // +1 coeff

            // 3. Dynamics entry (Source for next step)
            // If k < N, x_k appears in x_{k+1} = A_k x_k + ...
            // Equation: x_{k+1} - A_k x_k - B_k u_k = 0
            // So coefficient is -A_k(row, i) in row (row_offset_dyn + (k+1)*nx_ + row)
            if (k < N) {
                // A_k is 3x3. This x_k variable (component i) affects all 3 rows of next dynamics.
                // But wait, A_k might be sparse? LinearizedTrajectoryGenerator outputs full 3x3.
                // So we add 3 entries.
                int dyn_row_next_base = row_offset_dyn + (k + 1) * nx_;
                for (int r = 0; r < nx_; ++r) {
                    A_i_.push_back(dyn_row_next_base + r);
                    A_x_.push_back(0.0); // Placeholder for -A_k(r, i). Updated at runtime.
                }
            }

            A_p_.push_back(A_i_.size());
        }

        // u_k columns (size nu_), only if k < N
        if (k < N) {
            for (int i = 0; i < nu_; ++i) {
                int col_idx = k * (nx_ + nu_) + nx_ + i;

                // 1. Box constraint
                A_i_.push_back(col_idx);
                A_x_.push_back(1.0);

                // 2. Dynamics entry (Source for next step)
                // Equation: x_{k+1} - A_k x_k - B_k u_k = 0
                // Coefficient: -B_k(row, i) in row (row_offset_dyn + (k+1)*nx_ + row)
                int dyn_row_next_base = row_offset_dyn + (k + 1) * nx_;
                for (int r = 0; r < nx_; ++r) {
                    A_i_.push_back(dyn_row_next_base + r);
                    A_x_.push_back(0.0); // Placeholder for -B_k(r, i). Updated at runtime.
                } 
                
                A_p_.push_back(A_i_.size());
            }
        }
    }

    // Prepare q, l, u
    q_.resize(n_vars_, 0.0);
    l_.resize(n_constraints_, -OSQP_INFTY);
    u_.resize(n_constraints_, OSQP_INFTY);
}

void MPCSolver::UpdateProblem(const double* current_state, const double* lin_mats, const double* ref_states, const double* ref_inputs) {
    int N = config_.horizon;
    int row_offset_dyn = n_vars_; 

    // 1. Update Box Constraints (Shifted by ref_inputs for delta_u)
    // Box bounds rows are 0 to n_vars_-1
    for (int k = 0; k <= N; ++k) {
        int base = k * (nx_ + nu_);
        
        // x_k bounds (usually +/- INF)
        for (int i = 0; i < nx_; ++i) {
            l_[base + i] = config_.state_min[i];
            u_[base + i] = config_.state_max[i];
        }

        if (k < N) {
            // delta_u bounds: (u_min - u_ref) <= delta_u <= (u_max - u_ref)
            for (int i = 0; i < nu_; ++i) {
                double u_ref = ref_inputs[k * nu_ + i];
                l_[base + nx_ + i] = config_.input_min[i] - u_ref;
                u_[base + nx_ + i] = config_.input_max[i] - u_ref;
            }
        }
    }

    // 2. Dynamics and Initial State Constraints (Equality l = u)
    // Row 0..nx-1: x0 = current_state (the error)
    for (int i = 0; i < nx_; ++i) {
        l_[row_offset_dyn + i] = current_state[i];
        u_[row_offset_dyn + i] = current_state[i];
    }
    
    // Rows subsequent: x_{k+1} - A_k x_k - B_k u_k = 0
    // Note: Since z contains delta_u, and x is error, the dynamics are simply
    // x_{k+1} = A x_k + B delta_u.
    for (int k = 0; k < N; ++k) {
        int dyn_row_base = row_offset_dyn + (k+1) * nx_;
        for (int i = 0; i < nx_; ++i) {
            l_[dyn_row_base + i] = 0.0;
            u_[dyn_row_base + i] = 0.0;
        }
    }

    // 3. Update A Matrix Values (The linearized dynamics)
    // Same sparsity, just update -A and -B entries.
    int A_x_idx = 0;
    int A_mat_size = nx_ * nx_; // Size of A matrix (e.g., 3x3 = 9)
    int B_mat_size = nx_ * nu_; // Size of B matrix (e.g., 3x3 = 9)
    int mat_block_size = A_mat_size + B_mat_size; // Total size for A and B for one step

    for (int k = 0; k <= N; ++k) {
        // x_k columns
        for (int i = 0; i < nx_; ++i) {
            A_x_idx++; // Box constraint (value 1.0, skip)
            
            A_x_idx++; // Dynamics Identity/Init (value 1.0, skip)

            if (k < N) {
                // Next step A dynamics: -A_k(r, i)
                // A_k is row-major in dense array: Ak[row*nx_ + col]
                // We are in col `i`, iterating rows `r`.
                const double* Ak = &lin_mats[k * mat_block_size + 0]; // A is first A_mat_size elements
                for (int r = 0; r < nx_; ++r) {
                    A_x_[A_x_idx++] = -Ak[r * nx_ + i];
                }
            }
        }

        // u_k (delta_u) columns
        if (k < N) {
           const double* Bk = &lin_mats[k * mat_block_size + A_mat_size]; // B is after A
           for (int i = 0; i < nu_; ++i) {
               A_x_idx++; // Box constraint (value 1.0) for delta_u limits

               // Next step B dynamics: -B_k(r, i)
               for (int r = 0; r < nx_; ++r) {
                   A_x_[A_x_idx++] = -Bk[r * nu_ + i];
               }
           }
        }
    }

    // 4. Update Linear Cost q (Gradient)
    // Minimize x'Qx + delta_u'R delta_u.
    // q = [Q*x_ref_offset, R*u_ref_offset...]
    // Since x is error, we want x to be 0 -> q_x = 0.
    // Since u is delta, we want delta_u to be 0 -> q_u = 0.
    for (int k = 0; k < n_vars_; ++k) {
        q_[k] = 0.0;
    }
}

int MPCSolver::Solve(const double* current_state, const double* linearized_matrices, const double* ref_states, const double* ref_inputs, double* output_u) {
    if (!data_ || !settings_) return -1;

    // 1. Update vectors and A matrix values
    UpdateProblem(current_state, linearized_matrices, ref_states, ref_inputs);

    // 2. Setup or Update OSQP
    if (!is_initialized_) {
        data_->n = n_vars_;
        data_->m = n_constraints_;
        
        // Convert P to CSC
        data_->P = csc_matrix(data_->n, data_->n, P_x_.size(), P_x_.data(), P_i_.data(), P_p_.data());
        
        // Convert A to CSC
        data_->A = csc_matrix(data_->m, data_->n, A_x_.size(), A_x_.data(), A_i_.data(), A_p_.data());
        
        data_->q = q_.data();
        data_->l = l_.data();
        data_->u = u_.data();
        
        c_int setup_res = osqp_setup(&work_, data_, settings_);
        if (setup_res == 0) {
            is_initialized_ = true;
        } else {
            std::cerr << "[MPC] CRITICAL: osqp_setup failed with error code: " << (int)setup_res << std::endl;
            // Memory Cleanup: csc_matrix allocated heap memory that must be freed if setup fails
            if (data_->A) { c_free(data_->A); data_->A = nullptr; }
            if (data_->P) { c_free(data_->P); data_->P = nullptr; }
            return (int)setup_res;
        }
    } else {
        // Hot Update
        c_int res;
        res = osqp_update_lin_cost(work_, q_.data());
        if (res != 0) std::cerr << "[MPC] Error: osqp_update_lin_cost failed: " << (int)res << std::endl;
        
        res = osqp_update_bounds(work_, l_.data(), u_.data());
        if (res != 0) std::cerr << "[MPC] Error: osqp_update_bounds failed: " << (int)res << std::endl;
        
        res = osqp_update_A(work_, A_x_.data(), NULL, (c_int)A_x_.size());
        if (res != 0) std::cerr << "[MPC] Error: osqp_update_A failed: " << (int)res << std::endl;
        // Note: P is constant, no update needed.
    }

    // 3. Solve
    osqp_solve(work_);

    // 4. Extract Result
    if (work_->info->status_val != OSQP_SOLVED && work_->info->status_val != OSQP_SOLVED_INACCURATE) {
        return (int)work_->info->status_val;
    }

    // Safety check: Ensure solution is available before dereferencing
    if (!work_->solution || !work_->solution->x) {
        return -2; // Custom error code for missing solution
    }

    // Result z = [x0, u0, ... ]
    // u0 is at index nx_ (3)
    output_u[0] = work_->solution->x[nx_ + 0];
    output_u[1] = work_->solution->x[nx_ + 1];
    output_u[2] = work_->solution->x[nx_ + 2];

    return 0;
}

} // namespace solvers
} // namespace lib9427
