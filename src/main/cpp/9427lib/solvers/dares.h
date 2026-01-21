#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>

namespace lib9427 {
namespace solvers {

/**
 * Discrete Algebraic Riccati Equation (DARE) Solver using Schur Decomposition.
 * 
 * Solves: P = A^T P A - (A^T P B)(R + B^T P B)^-1 (B^T P A) + Q
 * 
 * Algorithm:
 * 1. Construct the Symplectic Hamiltonian Matrix (Z).
 * 2. Perform Real Schur Decomposition (Ordered).
 * 3. Extract P using the stable invariant subspace.
 * 
 * Reference: A. J. Laub, "A Schur Method for Solving Algebraic Riccati Equations."
 */
class DARE {
 public:
  template <int States, int Inputs>
  static Eigen::Matrix<double, States, States> Solve(
      const Eigen::Matrix<double, States, States>& A,
      const Eigen::Matrix<double, States, Inputs>& B,
      const Eigen::Matrix<double, States, States>& Q,
      const Eigen::Matrix<double, Inputs, Inputs>& R) {
      
      // Determine sizes at runtime if needed
      int n = A.rows();
      int m = B.cols();
      
      // Optimization: Precompute inverses
      Eigen::Matrix<double, Inputs, Inputs> R_inv = R.inverse();
      Eigen::Matrix<double, States, States> A_inv = A.inverse();
      Eigen::Matrix<double, States, States> A_inv_T = A_inv.transpose();
      
      Eigen::Matrix<double, States, States> BRB = B * R_inv * B.transpose();
      
      // SDA (Structure Preserving Doubling Algorithm)
      Eigen::Matrix<double, States, States> Ak = A;
      Eigen::Matrix<double, States, States> Gk = BRB; // G_0 = B R^-1 B^T
      Eigen::Matrix<double, States, States> Hk = Q;
      
      Eigen::Matrix<double, States, States> I = Eigen::Matrix<double, States, States>::Identity(n, n);
      
      double diff = 1.0;
      int iter = 0;
      while (diff > 1e-10 && iter < 100) {
          Eigen::Matrix<double, States, States> I_plus_GkHk = I + Gk * Hk;
          Eigen::Matrix<double, States, States> Inv = I_plus_GkHk.inverse();
          
          Eigen::Matrix<double, States, States> Ak_next = Ak * Inv * Ak;
          Eigen::Matrix<double, States, States> Gk_next = Gk + Ak * Inv * Gk * Ak.transpose();
          Eigen::Matrix<double, States, States> Hk_next = Hk + Ak.transpose() * Hk * Inv * Ak;
          
          diff = (Hk_next - Hk).norm();
          
          Ak = Ak_next;
          Gk = Gk_next;
          Hk = Hk_next;
          iter++;
      }
      
      return Hk; // H converges to P
  }
};

} // namespace solvers
} // namespace lib9427
