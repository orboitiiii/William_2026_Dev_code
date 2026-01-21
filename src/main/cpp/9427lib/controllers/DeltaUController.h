#pragma once

#include <Eigen/Dense>
#include <iostream>

#include "9427lib/controllers/lqr.h"
#include "9427lib/solvers/dares.h"

namespace lib9427 {
namespace controllers {

/**
 * Delta U Controller (Disturbance Observer Feedback Loop).
 *
 * Implements a control loop that estimates an input disturbance (u_error)
 * to handle integral windup via "Observer Error Compensation" rather than
 * traditional integrator clamping.
 *
 * Theoretical Basis:
 * 1. State Augmentation: The system is augmented with an error state u_error,
 *    where d/dt(u_error) = 0.
 *    x_aug = [x; u_error]
 *    A_aug = [A, B]
 *            [0, I]
 *    B_aug = [B]
 *            [0]
 *
 * 2. Separation Principle:
 *    - LQR gains are calculated for the NOMINAL system (States).
 *    - The Observer is designed for the AUGMENTED system (States + Inputs).
 *
 * 3. Control Law:
 *    u_cmd = K * (r - x_hat) - u_error_hat + u_ff
 *
 * 4. Anti-Windup:
 *    The observer's Predict step must be fed the ACTUAL saturated voltage
 *    applied to the plant. This prevents the "integrator" (u_error estimate)
 *    from winding up when the system is physically saturated.
 *
 * @tparam States Number of system states. (Pass Eigen::Dynamic for runtime size)
 * @tparam Inputs Number of control inputs. (Pass Eigen::Dynamic for runtime size)
 * @tparam Outputs Number of sensor outputs. (Pass Eigen::Dynamic for runtime size)
 */
template <int States, int Inputs, int Outputs>
class DeltaUController {
 private:
  // Helper to handle Dynamic dimension arithmetic
  static constexpr int AugStates = (States == Eigen::Dynamic || Inputs == Eigen::Dynamic) ? Eigen::Dynamic : (States + Inputs);

 public:
  struct Config {
    // LQR Costs for Nominal System
    Eigen::Matrix<double, States, States> Q_nominal;
    Eigen::Matrix<double, Inputs, Inputs> R_nominal;

    // Observer Process Noise
    // Q_state: Reliability of physical state evolution
    Eigen::Matrix<double, States, States> Q_state;

    // Q_disturbance: Input Error Noise (Volatility of the input error).
    Eigen::Matrix<double, Inputs, Inputs> Q_disturbance;

    // Observer Measurement Noise
    Eigen::Matrix<double, Outputs, Outputs> R_measurement;
  };

  /**
   * Constructs the Delta U Controller.
   * Computes LQR gains and initializes the Augmented Kalman Filter.
   */
  DeltaUController(const Eigen::Matrix<double, States, States>& A,
                   const Eigen::Matrix<double, States, Inputs>& B,
                   const Eigen::Matrix<double, Outputs, States>& C,
                   const Config& config)
      : A_(A), B_(B), C_(C) {

    // Determine runtime sizes (crucial if template args are Eigen::Dynamic)
    states_ = A.rows();
    inputs_ = B.cols();
    outputs_ = C.rows();
    int aug_states = states_ + inputs_;

    // 1. Calculate LQR Gain for Nominal System
    // Dispatch LQR calculation based on static or dynamic
    if constexpr (States != Eigen::Dynamic && Inputs != Eigen::Dynamic) {
        K_ = lib9427::controllers::LQR<States, Inputs>::ComputeGain(A, B, config.Q_nominal, config.R_nominal);
    } else {
        K_ = lib9427::controllers::LQR<Eigen::Dynamic, Eigen::Dynamic>::ComputeGain(A, B, config.Q_nominal, config.R_nominal);
    }

    // 2. Construct Augmented System Matrices for Observer
    // x_aug = [x; u_error] (Size: States + Inputs)
    A_aug_.resize(aug_states, aug_states);
    A_aug_.setZero();
    A_aug_.block(0, 0, states_, states_) = A;
    A_aug_.block(0, states_, states_, inputs_) = B;
    A_aug_.block(states_, states_, inputs_, inputs_) = Eigen::Matrix<double, Inputs, Inputs>::Identity(inputs_, inputs_);

    // B_aug = [B]
    //         [0]
    B_aug_.resize(aug_states, inputs_);
    B_aug_.setZero();
    B_aug_.block(0, 0, states_, inputs_) = B;

    // C_aug = [C  0]
    C_aug_.resize(outputs_, aug_states);
    C_aug_.setZero();
    C_aug_.block(0, 0, outputs_, states_) = C;

    // 3. Construct Process Noise Matrix Q_aug
    Q_aug_.resize(aug_states, aug_states);
    Q_aug_.setZero();
    Q_aug_.block(0, 0, states_, states_) = config.Q_state;
    Q_aug_.block(states_, states_, inputs_, inputs_) = config.Q_disturbance;

    // 4. Compute Kalman Gain for Augmented System
    Eigen::Matrix<double, AugStates, AugStates> P_aug;

    if constexpr (AugStates != Eigen::Dynamic) {
         P_aug = lib9427::solvers::DARE::Solve<AugStates, Outputs>(
            A_aug_.transpose(), C_aug_.transpose(),
            Q_aug_, config.R_measurement);
    } else {
         P_aug = lib9427::solvers::DARE::Solve<Eigen::Dynamic, Eigen::Dynamic>(
            A_aug_.transpose(), C_aug_.transpose(),
            Q_aug_, config.R_measurement);
    }

    // K_obs = P C^T (C P C^T + R)^-1
    Eigen::Matrix<double, Outputs, Outputs> S =
        C_aug_ * P_aug * C_aug_.transpose() + config.R_measurement;

    K_obs_ = P_aug * C_aug_.transpose() * S.inverse();

    // Initialize State
    x_hat_aug_.resize(aug_states, 1);
    x_hat_aug_.setZero();
  }

  /**
   * Updates the controller and returns the optimal control input.
   */
  Eigen::Matrix<double, Inputs, 1> Update(
      const Eigen::Matrix<double, Outputs, 1>& y,
      const Eigen::Matrix<double, States, 1>& r,
      const Eigen::Matrix<double, Inputs, 1>& u_ff,
      const Eigen::Matrix<double, Inputs, 1>& u_clamped_prev) {

    // 1. Predict Step (Time Update)
    x_hat_aug_ = A_aug_ * x_hat_aug_ + B_aug_ * u_clamped_prev;

    // 2. Correct Step (Measurement Update)
    Eigen::Matrix<double, Outputs, 1> y_hat = C_aug_ * x_hat_aug_;
    x_hat_aug_ += K_obs_ * (y - y_hat);

    // 3. Extract Estimates
    Eigen::Matrix<double, States, 1> x_hat = x_hat_aug_.head(states_);
    Eigen::Matrix<double, Inputs, 1> u_error_hat = x_hat_aug_.tail(inputs_);

    // 4. Calculate Control Law
    // u = K(r - x) - u_error + u_ff
    // u_error is added to the plant input by physics: u_total = u_cmd + u_error
    // To cancel it, we SUBTRACT the estimate from our command.
    Eigen::Matrix<double, Inputs, 1> u_cmd = K_ * (r - x_hat) - u_error_hat + u_ff;

    return u_cmd;
  }

  /**
   * Resets the observer state (both physical state and estimated disturbance).
   */
  void Reset(const Eigen::Matrix<double, States, 1>& initial_x) {
      x_hat_aug_.setZero();
      x_hat_aug_.head(states_) = initial_x;
  }

  /**
   * Resets the physical state estimate but KEEPS the learned disturbance.
   * Useful when resetting odometry but keeping battery compensation active.
   *
   * @param initial_x The new physical state to reset to.
   */
  void ResetStateOnly(const Eigen::Matrix<double, States, 1>& initial_x) {
      x_hat_aug_.head(states_) = initial_x;
      // u_error part remains unchanged
  }

  // Getters for telemetry
  Eigen::Matrix<double, States, 1> GetEstimatedState() const {
      return x_hat_aug_.head(states_);
  }

  Eigen::Matrix<double, Inputs, 1> GetEstimatedDisturbance() const {
      return x_hat_aug_.tail(inputs_);
  }

  // Note: For dynamic-sized getter, we return copy.
  Eigen::Matrix<double, Inputs, States> GetK() const { return K_; }
  Eigen::Matrix<double, AugStates, Outputs> GetObserverGain() const { return K_obs_; }

 private:
  int states_;
  int inputs_;
  int outputs_;

  // Nominal System Matrices
  Eigen::Matrix<double, States, States> A_;
  Eigen::Matrix<double, States, Inputs> B_;
  Eigen::Matrix<double, Outputs, States> C_;

  // Augmented System Matrices
  Eigen::Matrix<double, AugStates, AugStates> A_aug_;
  Eigen::Matrix<double, AugStates, Inputs> B_aug_;
  Eigen::Matrix<double, Outputs, AugStates> C_aug_;
  Eigen::Matrix<double, AugStates, AugStates> Q_aug_;

  // Gains
  Eigen::Matrix<double, Inputs, States> K_; // LQR Gain
  Eigen::Matrix<double, AugStates, Outputs> K_obs_; // Augmented Kalman Gain

  // State Estimate [x; u_error]
  Eigen::Matrix<double, AugStates, 1> x_hat_aug_;
};

} // namespace controllers
} // namespace lib9427
