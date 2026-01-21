#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace lib9427 {
namespace estimators {

/**
 * High-Performance Kalman Filter.
 * 
 * Uses Eigen for matrix operations.
 * Designed to minimize reallocation by keeping matrices as member variables.
 * 
 * Logic:
 * Predict:
 *   x_hat = A * x_hat + B * u
 *   P = A * P * A^T + Q
 * 
 * Correct:
 *   K = P * H^T * (H * P * H^T + R)^-1
 *   x_hat = x_hat + K * (y - H * x_hat)
 *   P = (I - K * H) * P
 */
class KalmanFilter {
 public:
  // Constructor initiates all matrices with correct sizes to reserve memory once.
  KalmanFilter(int states, int inputs, int outputs)
      : states_(states), inputs_(inputs), outputs_(outputs) {
      
      x_hat_ = Eigen::VectorXd::Zero(states);
      P_ = Eigen::MatrixXd::Identity(states, states); // Initial uncertainty? user can set.
      
      // Temporary/Internal matrices could be pre-allocated if needed,
      // but Eigen handles temporaries quite efficiently.
  }
  
  // Setters for model - usually called once or upon gain scheduling
  void SetModel(const Eigen::Ref<const Eigen::MatrixXd>& A,
                const Eigen::Ref<const Eigen::MatrixXd>& B,
                const Eigen::Ref<const Eigen::MatrixXd>& Q) {
      A_ = A;
      B_ = B;
      Q_ = Q;
  }

  void Predict(const Eigen::Ref<const Eigen::VectorXd>& u) {
      // x_hat = A * x_hat + B * u
      x_hat_ = A_ * x_hat_ + B_ * u;
      
      // P = A * P * A^T + Q
      P_ = A_ * P_ * A_.transpose() + Q_;
  }
  
  void Correct(const Eigen::Ref<const Eigen::VectorXd>& y,
               const Eigen::Ref<const Eigen::MatrixXd>& H,
               const Eigen::Ref<const Eigen::MatrixXd>& R) {
      // Calculate Innovation Covariance S = H * P * H^T + R
      // We can do this efficiently.
      
      Eigen::MatrixXd S = H * P_ * H.transpose() + R;
      
      // Calculate Kalman Gain K = P * H^T * S^-1
      // Use SelfAdjointEigenSolver or simple inverse if small.
      // For FRC sizes, inverse is fine.
      
      Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
      
      // Update Estimate
      // y_hat = H * x_hat
      // x_hat = x_hat + K * (y - y_hat)
      Eigen::VectorXd y_hat = H * x_hat_;
      x_hat_ += K * (y - y_hat);
      
      // Update Covariance
      // P = (I - K * H) * P
      Eigen::MatrixXd I = Eigen::MatrixXd::Identity(states_, states_);
      P_ = (I - K * H) * P_;
  }
  
  // Getters
  Eigen::VectorXd GetXHat() const { return x_hat_; }
  
  // Set initial state
  void SetXHat(const Eigen::Ref<const Eigen::VectorXd>& x0) { x_hat_ = x0; }

 private:
  int states_;
  int inputs_;
  int outputs_;

  // State
  Eigen::VectorXd x_hat_;
  Eigen::MatrixXd P_;
  
  // Dynamics (Constant or Time-Varying)
  // Stored here if constant, or passed in updates. 
  // For 'Predict' simplification, we store A, B, Q here.
  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;
  Eigen::MatrixXd Q_;
};

} // namespace estimators
} // namespace lib9427
