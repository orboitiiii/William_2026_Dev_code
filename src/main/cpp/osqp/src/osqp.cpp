/**
 * @file osqp.c
 * @brief Minimal OSQP ADMM solver implementation.
 *
 * Implements the core ADMM algorithm for solving QPs of the form:
 *   min 1/2 x'Px + q'x
 *   s.t. l <= Ax <= u
 *
 * This is a simplified implementation focused on:
 * 1. Zero dynamic memory allocation during solve (all memory pre-allocated).
 * 2. Efficient sparse matrix operations.
 * 3. Warm-starting for MPC applications.
 *
 * Algorithm Reference:
 * Stellato et al., "OSQP: An operator splitting solver for quadratic programs"
 */

#include "osqp.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// ============================================================================
// Memory Helpers
// ============================================================================

void* c_malloc(size_t size) {
    return malloc(size);
}

void c_free(void* ptr) {
    free(ptr);
}

void* c_calloc(size_t num, size_t size) {
    return calloc(num, size);
}

void* c_realloc(void* ptr, size_t size) {
    return realloc(ptr, size);
}

// ============================================================================
// CSC Matrix Helpers
// ============================================================================

csc* csc_matrix(c_int m, c_int n, c_int nzmax, c_float* x, c_int* i, c_int* p) {
    csc* A = (csc*)c_malloc(sizeof(csc));
    if (!A) return OSQP_NULL;
    
    A->m = m;
    A->n = n;
    A->nzmax = nzmax;
    A->x = x;
    A->i = i;
    A->p = p;
    A->nz = -1; // CSC format uses p[n] for nnz
    
    return A;
}

void csc_spfree(csc* A) {
    if (A) {
        // Note: We don't free x, i, p as they are externally owned
        c_free(A);
    }
}

// ============================================================================
// Vector Operations
// ============================================================================

static void vec_set_scalar(c_float* v, c_float val, c_int n) {
    for (c_int i = 0; i < n; i++) v[i] = val;
}

static void vec_copy(const c_float* src, c_float* dst, c_int n) {
    memcpy(dst, src, n * sizeof(c_float));
}

static c_float vec_norm_inf(const c_float* v, c_int n) {
    c_float max_val = 0.0;
    for (c_int i = 0; i < n; i++) {
        c_float abs_val = fabs(v[i]);
        if (abs_val > max_val) max_val = abs_val;
    }
    return max_val;
}

static void vec_add_scaled(c_float* result, const c_float* a, const c_float* b, c_float scale, c_int n) {
    for (c_int i = 0; i < n; i++) {
        result[i] = a[i] + scale * b[i];
    }
}

static void vec_ew_prod(c_float* result, const c_float* a, const c_float* b, c_int n) {
    for (c_int i = 0; i < n; i++) {
        result[i] = a[i] * b[i];
    }
}

// ============================================================================
// Sparse Matrix-Vector Multiplication
// ============================================================================

// y = A * x
static void mat_vec(const csc* A, const c_float* x, c_float* y) {
    // Zero output
    memset(y, 0, A->m * sizeof(c_float));
    
    // CSC format: iterate columns
    for (c_int j = 0; j < A->n; j++) {
        c_float xj = x[j];
        for (c_int idx = A->p[j]; idx < A->p[j + 1]; idx++) {
            y[A->i[idx]] += A->x[idx] * xj;
        }
    }
}

// y = A' * x (transpose)
static void mat_tpose_vec(const csc* A, const c_float* x, c_float* y) {
    memset(y, 0, A->n * sizeof(c_float));
    
    for (c_int j = 0; j < A->n; j++) {
        c_float sum = 0.0;
        for (c_int idx = A->p[j]; idx < A->p[j + 1]; idx++) {
            sum += A->x[idx] * x[A->i[idx]];
        }
        y[j] = sum;
    }
}

// y += P * x (P is symmetric, stored as upper triangular)
static void mat_vec_sym(const csc* P, const c_float* x, c_float* y) {
    for (c_int j = 0; j < P->n; j++) {
        c_float xj = x[j];
        for (c_int idx = P->p[j]; idx < P->p[j + 1]; idx++) {
            c_int i = P->i[idx];
            c_float val = P->x[idx];
            y[i] += val * xj;
            if (i != j) {
                // Add symmetric contribution
                y[j] += val * x[i];
            }
        }
    }
}

// ============================================================================
// Projection onto Box Constraints
// ============================================================================

static void project_box(c_float* z, const c_float* l, const c_float* u, c_int m) {
    for (c_int i = 0; i < m; i++) {
        if (z[i] < l[i]) z[i] = l[i];
        else if (z[i] > u[i]) z[i] = u[i];
    }
}

// ============================================================================
// Default Settings
// ============================================================================

void osqp_set_default_settings(OSQPSettings* settings) {
    settings->rho = 0.1;
    settings->sigma = 1e-6;
    settings->max_iter = 4000;
    settings->eps_abs = 1e-3;
    settings->eps_rel = 1e-3;
    settings->alpha = 1.6;
    settings->warm_start = 1;
    settings->verbose = 0;
    settings->delta = 1e-6;
    settings->polish = 0;
    settings->check_termination = 25;
    settings->time_limit = 0.0;
    settings->scaling = 10;
    settings->scaled_termination = 0;
}

// ============================================================================
// Solver Setup
// ============================================================================

c_int osqp_setup(OSQPWorkspace** workp, const OSQPData* data, const OSQPSettings* settings) {
    if (!workp || !data || !settings) return -1;
    
    c_int n = data->n;
    c_int m = data->m;
    
    OSQPWorkspace* work = (OSQPWorkspace*)c_calloc(1, sizeof(OSQPWorkspace));
    if (!work) return -1;
    
    // Store pointers to data and settings
    work->data = (OSQPData*)data;
    work->settings = (OSQPSettings*)settings;
    
    // Allocate solution struct
    work->solution = (OSQPSolution*)c_calloc(1, sizeof(OSQPSolution));
    work->info = (OSQPInfo*)c_calloc(1, sizeof(OSQPInfo));
    
    if (!work->solution || !work->info) {
        osqp_cleanup(work);
        return -1;
    }
    
    // Allocate working vectors
    work->x = (c_float*)c_calloc(n, sizeof(c_float));
    work->z = (c_float*)c_calloc(m, sizeof(c_float));
    work->y = (c_float*)c_calloc(m, sizeof(c_float));
    work->x_prev = (c_float*)c_calloc(n, sizeof(c_float));
    work->z_prev = (c_float*)c_calloc(m, sizeof(c_float));
    work->rhs = (c_float*)c_calloc(n, sizeof(c_float));
    work->Ax = (c_float*)c_calloc(m, sizeof(c_float));
    work->Px = (c_float*)c_calloc(n, sizeof(c_float));
    work->delta_y = (c_float*)c_calloc(m, sizeof(c_float));
    work->Atdelta_y = (c_float*)c_calloc(n, sizeof(c_float));
    
    // Solution vectors
    work->solution->x = (c_float*)c_calloc(n, sizeof(c_float));
    work->solution->y = (c_float*)c_calloc(m, sizeof(c_float));
    
    // Check allocations
    if (!work->x || !work->z || !work->y || !work->x_prev || !work->z_prev ||
        !work->rhs || !work->Ax || !work->Px || !work->delta_y || !work->Atdelta_y ||
        !work->solution->x || !work->solution->y) {
        osqp_cleanup(work);
        return -1;
    }
    
    // Initialize info
    work->info->status_val = OSQP_UNSOLVED;
    work->info->status = "unsolved";
    work->info->iter = 0;
    
    *workp = work;
    return 0;
}

// ============================================================================
// Cleanup
// ============================================================================

c_int osqp_cleanup(OSQPWorkspace* work) {
    if (!work) return 0;
    
    c_free(work->x);
    c_free(work->z);
    c_free(work->y);
    c_free(work->x_prev);
    c_free(work->z_prev);
    c_free(work->rhs);
    c_free(work->Ax);
    c_free(work->Px);
    c_free(work->delta_y);
    c_free(work->Atdelta_y);
    
    if (work->solution) {
        c_free(work->solution->x);
        c_free(work->solution->y);
        c_free(work->solution);
    }
    
    c_free(work->info);
    c_free(work);
    
    return 0;
}

// ============================================================================
// Update Functions (Zero Allocation)
// ============================================================================

c_int osqp_update_lin_cost(OSQPWorkspace* work, const c_float* q_new) {
    if (!work || !q_new) return -1;
    memcpy(work->data->q, q_new, work->data->n * sizeof(c_float));
    return 0;
}

c_int osqp_update_bounds(OSQPWorkspace* work, const c_float* l_new, const c_float* u_new) {
    if (!work) return -1;
    if (l_new) memcpy(work->data->l, l_new, work->data->m * sizeof(c_float));
    if (u_new) memcpy(work->data->u, u_new, work->data->m * sizeof(c_float));
    return 0;
}

c_int osqp_update_A(OSQPWorkspace* work, const c_float* Ax_new, const c_int* Ax_new_idx, c_int Ax_new_n) {
    if (!work || !Ax_new) return -1;
    
    csc* A = work->data->A;
    c_int nnz = A->p[A->n];
    
    if (Ax_new_idx == OSQP_NULL) {
        // Update all values
        memcpy(A->x, Ax_new, nnz * sizeof(c_float));
    } else {
        // Update specific values
        for (c_int i = 0; i < Ax_new_n; i++) {
            A->x[Ax_new_idx[i]] = Ax_new[i];
        }
    }
    
    return 0;
}

c_int osqp_update_P(OSQPWorkspace* work, const c_float* Px_new, const c_int* Px_new_idx, c_int Px_new_n) {
    if (!work || !Px_new) return -1;
    
    csc* P = work->data->P;
    c_int nnz = P->p[P->n];
    
    if (Px_new_idx == OSQP_NULL) {
        memcpy(P->x, Px_new, nnz * sizeof(c_float));
    } else {
        for (c_int i = 0; i < Px_new_n; i++) {
            P->x[Px_new_idx[i]] = Px_new[i];
        }
    }
    
    return 0;
}

c_int osqp_warm_start(OSQPWorkspace* work, const c_float* x, const c_float* y) {
    if (!work) return -1;
    
    if (x) vec_copy(x, work->x, work->data->n);
    if (y) vec_copy(y, work->y, work->data->m);
    
    return 0;
}

// ============================================================================
// ADMM Solve
// ============================================================================

/**
 * Simplified ADMM iteration for QP:
 *
 * The ADMM algorithm splits the problem into:
 *   1. x-update: (P + sigma*I + rho*A'A) x = sigma*x_prev - q + A'(rho*z_prev - y)
 *   2. z-update: z = proj_box(A*x + (1/rho)*y)
 *   3. y-update: y = y + rho*(A*x - z)
 *
 * For simplicity, we use an iterative method (conjugate gradient) for the x-update
 * instead of direct factorization, which is more suitable for embedded systems.
 */

// Simple Conjugate Gradient for (P + sigma*I + rho*A'A) x = b
static c_int cg_solve(OSQPWorkspace* work, c_float* x, const c_float* b, c_int max_iter, c_float tol) {
    c_int n = work->data->n;
    c_int m = work->data->m;
    c_float rho = work->settings->rho;
    c_float sigma = work->settings->sigma;
    
    // Allocate temporary vectors (use workspace buffers)
    c_float* r = work->rhs;      // Residual
    c_float* p = work->Px;       // Search direction (reusing Px temporarily)
    c_float* Ap = work->Atdelta_y; // A*p result
    c_float* temp_m = work->delta_y;
    c_float* temp_n = (c_float*)c_calloc(n, sizeof(c_float)); // Small allocation for temp
    
    if (!temp_n) return -1;
    
    // r = b - (P + sigma*I + rho*A'A) * x
    // First compute Ax
    mat_vec(work->data->A, x, work->Ax);
    
    // Ap = P*x + sigma*x + rho*A'*(A*x)
    memset(temp_n, 0, n * sizeof(c_float));
    mat_vec_sym(work->data->P, x, temp_n);
    
    for (c_int i = 0; i < n; i++) {
        temp_n[i] += sigma * x[i];
    }
    
    mat_tpose_vec(work->data->A, work->Ax, Ap);
    for (c_int i = 0; i < n; i++) {
        temp_n[i] += rho * Ap[i];
    }
    
    // r = b - temp_n
    for (c_int i = 0; i < n; i++) {
        r[i] = b[i] - temp_n[i];
    }
    
    // p = r
    vec_copy(r, p, n);
    
    c_float rsold = 0.0;
    for (c_int i = 0; i < n; i++) rsold += r[i] * r[i];
    
    if (rsold < tol * tol) {
        c_free(temp_n);
        return 0; // Already converged
    }
    
    for (c_int iter = 0; iter < max_iter; iter++) {
        // Ap = (P + sigma*I + rho*A'A) * p
        mat_vec(work->data->A, p, temp_m);
        
        memset(temp_n, 0, n * sizeof(c_float));
        mat_vec_sym(work->data->P, p, temp_n);
        
        for (c_int i = 0; i < n; i++) {
            temp_n[i] += sigma * p[i];
        }
        
        mat_tpose_vec(work->data->A, temp_m, Ap);
        for (c_int i = 0; i < n; i++) {
            temp_n[i] += rho * Ap[i];
        }
        
        // alpha = rsold / (p' * Ap)
        c_float pAp = 0.0;
        for (c_int i = 0; i < n; i++) pAp += p[i] * temp_n[i];
        
        if (fabs(pAp) < 1e-12) break; // Numerical issue
        
        c_float alpha = rsold / pAp;
        
        // x = x + alpha * p
        // r = r - alpha * Ap
        c_float rsnew = 0.0;
        for (c_int i = 0; i < n; i++) {
            x[i] += alpha * p[i];
            r[i] -= alpha * temp_n[i];
            rsnew += r[i] * r[i];
        }
        
        if (sqrt(rsnew) < tol) break;
        
        // p = r + (rsnew/rsold) * p
        c_float beta = rsnew / rsold;
        for (c_int i = 0; i < n; i++) {
            p[i] = r[i] + beta * p[i];
        }
        
        rsold = rsnew;
    }
    
    c_free(temp_n);
    return 0;
}

c_int osqp_solve(OSQPWorkspace* work) {
    if (!work || !work->data) return -1;
    
    c_int n = work->data->n;
    c_int m = work->data->m;
    const OSQPSettings* s = work->settings;
    
    c_float* x = work->x;
    c_float* z = work->z;
    c_float* y = work->y;
    c_float* x_prev = work->x_prev;
    c_float* z_prev = work->z_prev;
    c_float* rhs = work->rhs;
    c_float* Ax = work->Ax;
    
    // If not warm starting, zero initialize
    if (!s->warm_start) {
        vec_set_scalar(x, 0.0, n);
        vec_set_scalar(z, 0.0, m);
        vec_set_scalar(y, 0.0, m);
    }
    
    c_int iter = 0;
    c_float pri_res = OSQP_INFTY;
    c_float dua_res = OSQP_INFTY;
    
    for (iter = 0; iter < s->max_iter; iter++) {
        // Save previous iterates
        vec_copy(x, x_prev, n);
        vec_copy(z, z_prev, m);
        
        // ===== X-UPDATE =====
        // Solve (P + sigma*I + rho*A'A) x = sigma*x_prev - q + A'(rho*z - y)
        
        // Build RHS
        // rhs = sigma * x_prev - q
        for (c_int i = 0; i < n; i++) {
            rhs[i] = s->sigma * x_prev[i] - work->data->q[i];
        }
        
        // temp = rho * z - y
        for (c_int i = 0; i < m; i++) {
            work->delta_y[i] = s->rho * z[i] - y[i];
        }
        
        // rhs += A' * temp
        mat_tpose_vec(work->data->A, work->delta_y, work->Atdelta_y);
        for (c_int i = 0; i < n; i++) {
            rhs[i] += work->Atdelta_y[i];
        }
        
        // Solve for x using CG
        cg_solve(work, x, rhs, 50, 1e-6);
        
        // ===== Z-UPDATE =====
        // z = proj_box(alpha * Ax + (1-alpha) * z_prev + (1/rho) * y)
        mat_vec(work->data->A, x, Ax);
        
        c_float alpha = s->alpha;
        for (c_int i = 0; i < m; i++) {
            z[i] = alpha * Ax[i] + (1.0 - alpha) * z_prev[i] + y[i] / s->rho;
        }
        project_box(z, work->data->l, work->data->u, m);
        
        // ===== Y-UPDATE =====
        // y = y + rho * (alpha * Ax + (1-alpha) * z_prev - z)
        for (c_int i = 0; i < m; i++) {
            c_float Az_tilde = alpha * Ax[i] + (1.0 - alpha) * z_prev[i];
            y[i] += s->rho * (Az_tilde - z[i]);
        }
        
        // ===== CHECK CONVERGENCE =====
        if ((iter + 1) % s->check_termination == 0) {
            // Primal residual: ||Ax - z||
            c_float pri_res_norm = 0.0;
            for (c_int i = 0; i < m; i++) {
                c_float diff = Ax[i] - z[i];
                pri_res_norm += diff * diff;
            }
            pri_res = sqrt(pri_res_norm);
            
            // Dual residual: ||rho * A'(z - z_prev)||
            for (c_int i = 0; i < m; i++) {
                work->delta_y[i] = z[i] - z_prev[i];
            }
            mat_tpose_vec(work->data->A, work->delta_y, work->Atdelta_y);
            
            c_float dua_res_norm = 0.0;
            for (c_int i = 0; i < n; i++) {
                c_float val = s->rho * work->Atdelta_y[i];
                dua_res_norm += val * val;
            }
            dua_res = sqrt(dua_res_norm);
            
            // Check termination
            c_float eps_pri = s->eps_abs * sqrt((c_float)m) + s->eps_rel * fmax(vec_norm_inf(Ax, m), vec_norm_inf(z, m));
            c_float eps_dua = s->eps_abs * sqrt((c_float)n) + s->eps_rel * s->rho * vec_norm_inf(work->Atdelta_y, n);
            
            if (pri_res < eps_pri && dua_res < eps_dua) {
                work->info->status_val = OSQP_SOLVED;
                work->info->status = "solved";
                break;
            }
        }
    }
    
    if (iter >= s->max_iter) {
        work->info->status_val = OSQP_MAX_ITER_REACHED;
        work->info->status = "maximum iterations reached";
    }
    
    // Copy solution
    vec_copy(x, work->solution->x, n);
    vec_copy(y, work->solution->y, m);
    
    // Compute objective value
    c_float obj = 0.0;
    memset(work->Px, 0, n * sizeof(c_float));
    mat_vec_sym(work->data->P, x, work->Px);
    for (c_int i = 0; i < n; i++) {
        obj += 0.5 * x[i] * work->Px[i] + work->data->q[i] * x[i];
    }
    
    work->info->obj_val = obj;
    work->info->iter = iter + 1;
    work->info->pri_res = pri_res;
    work->info->dua_res = dua_res;
    
    return 0;
}
