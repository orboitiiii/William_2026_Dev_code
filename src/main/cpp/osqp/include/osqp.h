/**
 * @file osqp.h
 * @brief Minimal OSQP-compatible API for embedded MPC.
 *
 * This is a lightweight implementation of the OSQP interface tailored for
 * FRC robotics. It implements the core ADMM algorithm without external
 * dependencies beyond standard C libraries.
 *
 * Reference: Stellato et al., "OSQP: An operator splitting solver for
 * quadratic programs", Math. Prog. Comp., 2020.
 */

#ifndef OSQP_H
#define OSQP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>

// ============================================================================
// Type Definitions
// ============================================================================

/** Float type for all OSQP computations (double precision for accuracy). */
typedef double c_float;

/** Integer type for indices and sizes. */
typedef int64_t c_int;

/** Infinity constant for unbounded constraints. */
#define OSQP_INFTY ((c_float)1e30)

/** Solver status codes. */
typedef enum {
    OSQP_SOLVED = 1,                // Problem solved
    OSQP_SOLVED_INACCURATE = 2,     // Solved with reduced accuracy
    OSQP_MAX_ITER_REACHED = -2,     // Hit iteration limit
    OSQP_PRIMAL_INFEASIBLE = -3,    // Problem is primal infeasible
    OSQP_DUAL_INFEASIBLE = -4,      // Problem is dual infeasible
    OSQP_UNSOLVED = -10,            // Not solved yet
    OSQP_NON_CVX = -7               // Non-convex problem (P has negative eigenvalues)
} osqp_status;

// ============================================================================
// Sparse Matrix (CSC Format)
// ============================================================================

/**
 * Compressed Sparse Column (CSC) matrix structure.
 * For a matrix with m rows, n cols, nnz non-zeros:
 *   - x: array of nnz values
 *   - i: array of nnz row indices
 *   - p: array of n+1 column pointers (p[j] is start of column j)
 */
typedef struct {
    c_int    nzmax;  ///< Maximum number of entries
    c_int    m;      ///< Number of rows
    c_int    n;      ///< Number of columns
    c_int*   p;      ///< Column pointers (size n+1)
    c_int*   i;      ///< Row indices (size nzmax)
    c_float* x;      ///< Values (size nzmax)
    c_int    nz;     ///< Number of non-zeros (if -1, use p[n])
} csc;

// ============================================================================
// Solver Settings
// ============================================================================

typedef struct {
    c_float rho;              ///< ADMM step size
    c_float sigma;            ///< Regularization parameter
    c_int   max_iter;         ///< Maximum iterations
    c_float eps_abs;          ///< Absolute convergence tolerance
    c_float eps_rel;          ///< Relative convergence tolerance
    c_float alpha;            ///< Over-relaxation parameter (1.0 = no relaxation)
    c_int   warm_start;       ///< Enable warm starting
    c_int   verbose;          ///< Print output
    c_float delta;            ///< Polishing regularization
    c_int   polish;           ///< Enable polishing
    c_int   check_termination;///< Check termination every N iterations
    c_float time_limit;       ///< Time limit (0 = no limit)
    c_int   scaling;          ///< Number of scaling iterations
    c_int   scaled_termination;///< Use scaled termination criteria
} OSQPSettings;

// ============================================================================
// Problem Data
// ============================================================================

typedef struct {
    c_int    n;   ///< Number of variables
    c_int    m;   ///< Number of constraints
    csc*     P;   ///< Quadratic cost matrix (n x n, upper triangular)
    csc*     A;   ///< Constraint matrix (m x n)
    c_float* q;   ///< Linear cost vector (size n)
    c_float* l;   ///< Lower bounds (size m)
    c_float* u;   ///< Upper bounds (size m)
} OSQPData;

// ============================================================================
// Solution
// ============================================================================

typedef struct {
    c_float* x;   ///< Primal variables (size n)
    c_float* y;   ///< Dual variables (size m)
} OSQPSolution;

// ============================================================================
// Solver Info
// ============================================================================

typedef struct {
    c_int    iter;            ///< Number of iterations
    osqp_status status_val;   ///< Status code
    const char* status;       ///< Status string
    c_float  obj_val;         ///< Objective value
    c_float  pri_res;         ///< Primal residual
    c_float  dua_res;         ///< Dual residual
    c_float  run_time;        ///< Run time (seconds)
    c_float  setup_time;      ///< Setup time (seconds)
    c_float  solve_time;      ///< Solve time (seconds)
} OSQPInfo;

// ============================================================================
// Workspace
// ============================================================================

typedef struct OSQPWorkspace {
    OSQPData*     data;       ///< Problem data
    OSQPSettings* settings;   ///< Solver settings
    OSQPSolution* solution;   ///< Primal-dual solution
    OSQPInfo*     info;       ///< Solver information
    
    // Internal working memory
    c_float* x;               ///< Primal iterate
    c_float* z;               ///< Slack variable
    c_float* y;               ///< Dual variable
    c_float* x_prev;          ///< Previous x (for warm start)
    c_float* z_prev;          ///< Previous z
    c_float* rhs;             ///< Right-hand side for KKT
    c_float* Ax;              ///< A * x product
    c_float* Px;              ///< P * x product
    c_float* delta_y;         ///< Dual step
    c_float* Atdelta_y;       ///< A' * delta_y
    
    // Scaling
    c_float* D;               ///< Diagonal scaling for primal
    c_float* E;               ///< Diagonal scaling for dual
    c_float  c;               ///< Cost scaling factor
    
    // Pre-factorized KKT system (for direct methods)
    void*    linsys_solver;   ///< Linear system solver workspace
} OSQPWorkspace;

// ============================================================================
// API Functions
// ============================================================================

/**
 * Set default settings.
 * @param settings Settings struct to initialize.
 */
void osqp_set_default_settings(OSQPSettings* settings);

/**
 * Setup the solver workspace.
 * @param workp Pointer to workspace pointer (output).
 * @param data  Problem data.
 * @param settings Solver settings.
 * @return 0 on success, error code otherwise.
 */
c_int osqp_setup(OSQPWorkspace** workp, const OSQPData* data, const OSQPSettings* settings);

/**
 * Solve the QP.
 * @param work Workspace.
 * @return 0 on success, error code otherwise.
 */
c_int osqp_solve(OSQPWorkspace* work);

/**
 * Update linear cost vector.
 * @param work Workspace.
 * @param q_new New linear cost (size n).
 * @return 0 on success.
 */
c_int osqp_update_lin_cost(OSQPWorkspace* work, const c_float* q_new);

/**
 * Update constraint bounds.
 * @param work Workspace.
 * @param l_new New lower bounds (size m).
 * @param u_new New upper bounds (size m).
 * @return 0 on success.
 */
c_int osqp_update_bounds(OSQPWorkspace* work, const c_float* l_new, const c_float* u_new);

/**
 * Update constraint matrix A values.
 * @param work     Workspace.
 * @param Ax_new   New values (same sparsity pattern).
 * @param Ax_new_idx Indices of values to update (NULL = update all).
 * @param Ax_new_n Number of values to update (0 if Ax_new_idx is NULL).
 * @return 0 on success.
 */
c_int osqp_update_A(OSQPWorkspace* work, const c_float* Ax_new, const c_int* Ax_new_idx, c_int Ax_new_n);

/**
 * Update P matrix values.
 * @param work     Workspace.
 * @param Px_new   New values (same sparsity pattern).
 * @param Px_new_idx Indices of values to update (NULL = update all).
 * @param Px_new_n Number of values to update.
 * @return 0 on success.
 */
c_int osqp_update_P(OSQPWorkspace* work, const c_float* Px_new, const c_int* Px_new_idx, c_int Px_new_n);

/**
 * Warm start primal and/or dual variables.
 * @param work Workspace.
 * @param x    Primal warm start (NULL to skip).
 * @param y    Dual warm start (NULL to skip).
 * @return 0 on success.
 */
c_int osqp_warm_start(OSQPWorkspace* work, const c_float* x, const c_float* y);

/**
 * Cleanup workspace.
 * @param work Workspace to free.
 * @return 0 on success.
 */
c_int osqp_cleanup(OSQPWorkspace* work);

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Create a CSC matrix (does NOT copy data, uses provided pointers directly).
 */
csc* csc_matrix(c_int m, c_int n, c_int nzmax, c_float* x, c_int* i, c_int* p);

/**
 * Free a CSC matrix.
 */
void csc_spfree(csc* A);

/**
 * Allocate memory (wrapper for malloc).
 */
void* c_malloc(size_t size);

/**
 * Free memory (wrapper for free).
 */
void c_free(void* ptr);

/**
 * Calloc wrapper.
 */
void* c_calloc(size_t num, size_t size);

/**
 * Realloc wrapper.
 */
void* c_realloc(void* ptr, size_t size);

// Null pointer constant - use nullptr for C++ compatibility
#ifdef __cplusplus
#define OSQP_NULL nullptr
#else
#define OSQP_NULL ((void*)0)
#endif

#ifdef __cplusplus
}
#endif

#endif // OSQP_H
