#pragma once

#include <Eigen/Dense>
#include <Eigen/QR>

namespace lib9427 {
namespace controllers {

class Feedforward {
 public:
  /**
   * Computes the feedforward input u_ff that moves the system from r_k to r_next.
   * Solves B * u_ff = r_next - A * r_k using QR Decomposition.
   * 
   * @param A Discrete system matrix A
   * @param B Discrete system matrix B
   * @param r_k Current reference state
   * @param r_next Next reference state
   * @return u_ff Optimal feedforward input
   */
  template <typename DerivedA, typename DerivedB, typename DerivedR>
  static Eigen::VectorXd ComputePlantInversion(
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedB>& B,
      const Eigen::MatrixBase<DerivedR>& r_k,
      const Eigen::MatrixBase<DerivedR>& r_next) {
      
      // Delta = r_next - A * r_k
      Eigen::VectorXd delta = r_next - A * r_k;
      
      // Solve B * u = delta
      // Use ColPivHouseholderQR for robustness against rank deficiency
      Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(B);
      Eigen::VectorXd u_ff = dec.solve(delta);
      
      return u_ff;
  }
};

} // namespace controllers
} // namespace lib9427
