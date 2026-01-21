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

    // Allocation
    settings_ = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
    data_     = (OSQPData*)c_malloc(sizeof(OSQPData));

    if (settings_) {
        osqp_set_default_settings(settings_);
        settings_->alpha = 1.6;  // Over-relaxation for faster convergence (default: 1.6)
        settings_->verbose = 0;  // Disable verbose to reduce RIO log spam
        settings_->check_termination = 25;  // Check every 25 iterations
        
        // Balanced MPC Configuration
        // Tolerance: ~1mm precision - relaxed from 1e-4 to ensure convergence
        // Reference: OSQP may fail to converge with very tight tolerances
        settings_->eps_abs = 1e-3;
        settings_->eps_rel = 1e-3;
        
        // Scaling: Critical for numerical stability when Q/R weights span
        // multiple orders of magnitude. 15 iterations (reduced from 25).
        settings_->scaling = 15;
        
        // ADMM step size: Lower rho improves primal feasibility convergence
        settings_->rho = 0.1;  // Default is 0.1, keep it
        
        // Max iterations: Increased to ensure convergence with MPC dynamics
        settings_->max_iter = 20000;
        
        settings_->warm_start = 1;  // Enable Warm Start for Error-State MPC
    }
}

MPCSolver::~MPCSolver() {
    if (work_) osqp_cleanup(work_);
    if (data_) {
        // OSQP data pointers (A, P) need free if we allocated them via c_malloc/csc_matrix
        // But here we use our std::vectors and pass pointers.
        // We typically just free the struct.
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

void MPCSolver::UpdateProblem(const double* current_state, const double* lin_mats, const double* ref_states) {
    int N = config_.horizon;
    int row_offset_dyn = n_vars_; 

    // 1. Update Box Constraints (l, u)
    // Rows 0..n_vars_-1
    for (int k = 0; k <= N; ++k) {
        // x_k
        for (int i = 0; i < nx_; ++i) {
            int idx = k * (nx_ + nu_) + i;
            l_[idx] = config_.state_min[i];
            u_[idx] = config_.state_max[i];
        }
        // u_k
        if (k < N) {
            for (int i = 0; i < nu_; ++i) {
                int idx = k * (nx_ + nu_) + nx_ + i;
                l_[idx] = config_.input_min[i];
                u_[idx] = config_.input_max[i];
            }
        }
    }

    // 2. Update Dynamics Constraints & Initial State
    // Row n_vars_ .. end
    
    // Initial State: x0 = current_state
    // Rows: row_offset_dyn + 0..2
    for (int i = 0; i < nx_; ++i) {
        int idx = row_offset_dyn + i;
        l_[idx] = current_state[i];
        u_[idx] = current_state[i];
    }

    // Dynamics Steps: x_{k+1} - A x - B u = 0  => val = 0
    // Rows: row_offset_dyn + (k+1)*nx ..
    for (int k = 0; k < N; ++k) {
        int base = row_offset_dyn + (k + 1) * nx_;
        for (int i = 0; i < nx_; ++i) {
            l_[base + i] = 0.0;
            u_[base + i] = 0.0;
        }
    }

    // 3. Update A Matrix Values (The linearized dynamics)
    // We need to traverse the A_x_ array exactly as we built it in BuildSparsityPattern.
    // This implies a strict coupling. 
    // To ensure safety, we re-run the traversal logic but only write to A_x_.
    
    int A_x_idx = 0;
    int mat_block_size = 18; // 9 for A, 9 for B

    for (int k = 0; k <= N; ++k) {
        const double* Ak = nullptr;
        const double* Bk = nullptr;
        if (k < N) {
            Ak = &lin_mats[k * mat_block_size + 0]; // A is first 9
            Bk = &lin_mats[k * mat_block_size + 9]; // B is next 9
        }

        // x_k columns
        for (int i = 0; i < nx_; ++i) {
            A_x_idx++; // Box constraint (value 1.0, skip)
            
            A_x_idx++; // Dynamics Identity/Init (value 1.0, skip)

            if (k < N) {
                // Next step A dynamics: -A_k(r, i)
                // A_k is row-major in dense array: Ak[row*3 + col]
                // We are in col `i`, iterating rows `r`.
                for (int r = 0; r < nx_; ++r) {
                    double val = -Ak[r * nx_ + i];
                    A_x_[A_x_idx++] = val;
                }
            }
        }

        // u_k columns
        if (k < N) {
           for (int i = 0; i < nu_; ++i) {
               A_x_idx++; // Box constraint (value 1.0, skip)

               // Next step B dynamics: -B_k(r, i)
               for (int r = 0; r < nx_; ++r) {
                   double val = -Bk[r * nu_ + i]; // B is nx * nu (3x3)
                   A_x_[A_x_idx++] = val;
               }
           }
        }
    }

    // 4. Update Linear Cost q (Gradient)
    // J = (x-xref)^T Q (x-xref) = x^T Q x - 2 xref^T Q x + const
    // Linear term q = - Q * xref  (factor of 1/2 is usually in 0.5 xPx, so check solvers definition)
    // OSQP min 1/2 x'Px + q'x
    // So if cost is (x-r)'Q(x-r) = x'Qx - 2r'Qx + ... = 1/2 x'(2Q)x + (-2Q r)'x
    // Our P matrix has 'Q' on diagonal. That effectively means P = Q (if we assume 1/2 factor in solver).
    // Wait, OSQP objective is 1/2 x'Px + q'x.
    // If we want to minimize 1/2 (x - r)' Q (x - r) = 1/2 x'Qx - r'Qx
    // Then P = Q, q = -Q * r.
    
    // HOWEVER, typically we define P = Q directly.
    // So P has Q values.
    // q vector should be -Q * ref for states, -R * ref for inputs (usually 0).
    
    for (int k = 0; k < n_vars_; ++k) {
        q_[k] = 0; // Default
        
        // Find if State or Input
        int step = k / (nx_ + nu_);
        int rem  = k % (nx_ + nu_);
        
        if (rem < nx_) {
            // State
            // Get reference state for this step
            // ref_states layout: [x0, y0, th0, x1...]
            double r_val = ref_states[step * nx_ + rem]; 
            
            double weight = (k >= n_vars_ - nx_) ? config_.Q_final[rem] : config_.Q[rem];
            
            // q = -Q * r
            // Note: If P already contains 'Q', this is consistent with 1/2 xPx + qx 
            // matching 1/2 (x-r)Q(x-r).
            q_[k] = -weight * r_val;
        } // else Input u, ref usually 0 -> q=0
    }
}

int MPCSolver::Solve(const double* current_state, const double* linearized_matrices, const double* ref_states, double* output_u) {
    if (!data_ || !settings_) return -1;

    // 1. Update vectors and A matrix values
    UpdateProblem(current_state, linearized_matrices, ref_states);

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
        
        osqp_setup(&work_, data_, settings_);
        is_initialized_ = true;
    } else {
        // Hot Update
        osqp_update_lin_cost(work_, q_.data());
        osqp_update_bounds(work_, l_.data(), u_.data());
        osqp_update_A(work_, A_x_.data(), OSQP_NULL, 0); // Update all values
        // Note: P is constant, no update needed.
    }

    // 3. Solve
    osqp_solve(work_);

    // 4. Extract Result
    if (work_->info->status_val != OSQP_SOLVED && work_->info->status_val != OSQP_SOLVED_INACCURATE) {
        // Fallback?
        // std::cerr << "OSQP Failed: " << work_->info->status_val << std::endl;
        return work_->info->status_val;
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
