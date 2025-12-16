#pragma once

#include "../solvers/dares.h"

namespace lib9427 {
namespace controllers {

template <int States, int Inputs>
class LQR {
 public:
  /**
   * Computes the optimal gain matrix K for the given system and costs.
   * u = -K * x
    */
  static Eigen::Matrix<double, Inputs, States> ComputeGain(
      const Eigen::Matrix<double, States, States>& A,
      const Eigen::Matrix<double, States, Inputs>& B,
      const Eigen::Matrix<double, States, States>& Q,
      const Eigen::Matrix<double, Inputs, Inputs>& R) {
      
      // 1. Solve DARE to get P
      Eigen::Matrix<double, States, States> P = 
          lib9427::solvers::DARE::Solve<States, Inputs>(A, B, Q, R);
          
      // 2. Compute K = (R + B^T P B)^-1 B^T P A
      Eigen::Matrix<double, Inputs, Inputs> R_plus_BTPB = 
          R + B.transpose() * P * B;
          
      return R_plus_BTPB.inverse() * B.transpose() * P * A;
  }
};

} // namespace controllers
} // namespace lib9427
