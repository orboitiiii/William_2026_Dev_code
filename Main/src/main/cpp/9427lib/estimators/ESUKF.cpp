#include "ESUKF.h"
#include <cmath>

namespace lib9427 {
namespace estimators {

// Gravity constant (assumption: Z is up, so gravity is negative Z in world frame)
// Adjust if needed base on Frame choice. FRC: usually Z up.
static const Eigen::Vector3d GRAVITY(0.0, 0.0, -9.81);

ESUKF::ESUKF() {
    // Initialize Nominal State
    x_.setZero();
    x_(6) = 1.0; // Quaternion w=1

    // Initialize Covariance
    P_.setIdentity();
    P_ *= 0.1; // Initial uncertainty

    // Process Noise Defaults
    Q_.setIdentity();
    Q_.block<3,3>(0,0) *= 1e-4; // Pos
    Q_.block<3,3>(3,3) *= 1e-3; // Vel
    Q_.block<3,3>(6,6) *= 1e-5; // Angle
    Q_.block<3,3>(9,9) *= 1e-6; // Accel Bias
    Q_.block<3,3>(12,12) *= 1e-6;// Gyro Bias

    // UKF Weights
    // Merwe Scaled Unscented Transform parameters
    // alpha = 0.001, beta = 2, kappa = 0
    lambda_ = alpha_ * alpha_ * (ERROR_DIM + kappa_) - ERROR_DIM;
    
    int num_sigma = 2 * ERROR_DIM + 1;
    center_weights_m_.resize(num_sigma);
    center_weights_c_.resize(num_sigma);

    center_weights_m_[0] = lambda_ / (ERROR_DIM + lambda_);
    center_weights_c_[0] = center_weights_m_[0] + (1 - alpha_ * alpha_ + beta_);

    double weight = 0.5 / (ERROR_DIM + lambda_);
    for (int i = 1; i < num_sigma; ++i) {
        center_weights_m_[i] = weight;
        center_weights_c_[i] = weight;
    }

    // Pre-allocation
    error_sigmas_.resize(num_sigma);
    propagated_states_.resize(num_sigma);
    Z_sigmas_.resize(3, num_sigma); // Assuming 3D Position Measurement
    S_.resize(3, 3);
    P_xz_.resize(ERROR_DIM, 3);
}

void ESUKF::Initialize(const Eigen::Vector3d& init_pos, const Eigen::Quaterniond& init_quat) {
    x_.setZero();
    x_.segment<3>(0) = init_pos;
    x_(3) = 0; x_(4) = 0; x_(5) = 0; // Vel = 0
    x_(6) = init_quat.w();
    x_(7) = init_quat.x();
    x_(8) = init_quat.y();
    x_(9) = init_quat.z();
    // Biases 0
}

void ESUKF::Predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt) {
    // 1. Generate Sigma Points
    // Matrix Square Root of P using LLT (Cholesky)
    Eigen::LLT<MatrixE> lltOfP(P_ + 1e-9 * MatrixE::Identity()); // Stabilization
    MatrixE L = lltOfP.matrixL();
    L *= std::sqrt(ERROR_DIM + lambda_);

    // Sigma Points (Error State)
    error_sigmas_[0].setZero();
    for (int i = 0; i < ERROR_DIM; ++i) {
        error_sigmas_[i + 1] = L.col(i);
        error_sigmas_[i + 1 + ERROR_DIM] = -L.col(i);
    }

    // 2. Propagate Sigma Points through Non-Linear Model
    int num_sigma = 2 * ERROR_DIM + 1;

    for (int i = 0; i < num_sigma; ++i) {
        // A. Inject Error: X_i = X \oplus dX_i
        VectorS X_curr = x_;
        const VectorE& dX = error_sigmas_[i];

        // Position & Velocity & Biases: Simple addition
        X_curr.segment<3>(0) += dX.segment<3>(0);
        X_curr.segment<3>(3) += dX.segment<3>(3);
        X_curr.segment<3>(10) += dX.segment<3>(9);
        X_curr.segment<3>(13) += dX.segment<3>(12);

        // Quaternion: Multiply by error rotation
        Eigen::Quaterniond q_curr(X_curr(6), X_curr(7), X_curr(8), X_curr(9));
        Eigen::Vector3d dTheta = dX.segment<3>(6);
        Eigen::Quaterniond q_err = RotVecToQuat(dTheta);
        q_curr = q_curr * q_err; // Local perturbation
        q_curr.normalize();
        
        // Restore to state vector
        X_curr(6) = q_curr.w(); X_curr(7) = q_curr.x(); X_curr(8) = q_curr.y(); X_curr(9) = q_curr.z();

        // B. Prediction Step (Integration)
        Eigen::Vector3d pos = X_curr.segment<3>(0);
        Eigen::Vector3d vel = X_curr.segment<3>(3);
        Eigen::Quaterniond q(X_curr(6), X_curr(7), X_curr(8), X_curr(9));
        Eigen::Vector3d ba = X_curr.segment<3>(10);
        Eigen::Vector3d bg = X_curr.segment<3>(13);

        // Corrected IMU measurements
        Eigen::Vector3d acc_cor = accel - ba;
        Eigen::Vector3d gyro_cor = gyro - bg;

        // Dynamics
        Eigen::Vector3d p_next = pos + vel * dt + 0.5 * (q * acc_cor + GRAVITY) * dt * dt;
        Eigen::Vector3d v_next = vel + (q * acc_cor + GRAVITY) * dt;
        Eigen::Quaterniond q_next = q * RotVecToQuat(gyro_cor * dt);
        q_next.normalize();

        // Store
        VectorS X_next;
        X_next.segment<3>(0) = p_next;
        X_next.segment<3>(3) = v_next;
        X_next(6) = q_next.w(); X_next(7) = q_next.x(); X_next(8) = q_next.y(); X_next(9) = q_next.z();
        X_next.segment<3>(10) = ba; // Random walk process noise added later via Q
        X_next.segment<3>(13) = bg; // Random walk

        propagated_states_[i] = X_next;
    }

    // 3. Compute Mean State
    // Calculate weighted mean of vector parts
    VectorS x_pred_mean = VectorS::Zero();
    
    for (int i = 0; i < num_sigma; ++i) {
        x_pred_mean += center_weights_m_[i] * propagated_states_[i];
    }
    // Normalize quaternion part
    Eigen::Quaterniond q_mean(x_pred_mean(6), x_pred_mean(7), x_pred_mean(8), x_pred_mean(9));
    q_mean.normalize();
    x_pred_mean(6) = q_mean.w(); x_pred_mean(7) = q_mean.x(); x_pred_mean(8) = q_mean.y(); x_pred_mean(9) = q_mean.z();

    // 4. Compute Covariance
    P_.setZero();
    for (int i = 0; i < num_sigma; ++i) {
        // Compute error state delta: dX = X_i \ominus Mean
        VectorE dX;
        
        // Linear parts
        dX.segment<3>(0) = propagated_states_[i].segment<3>(0) - x_pred_mean.segment<3>(0);
        dX.segment<3>(3) = propagated_states_[i].segment<3>(3) - x_pred_mean.segment<3>(3);
        // Biases
        dX.segment<3>(9) = propagated_states_[i].segment<3>(10) - x_pred_mean.segment<3>(10);
        dX.segment<3>(12) = propagated_states_[i].segment<3>(13) - x_pred_mean.segment<3>(13);

        // Angular part
        Eigen::Quaterniond qi(propagated_states_[i](6), propagated_states_[i](7), propagated_states_[i](8), propagated_states_[i](9));
        // Error q_err = q_mean.inverse * qi
        Eigen::Quaterniond q_err = q_mean.conjugate() * qi;
        dX.segment<3>(6) = QuatToRotVec(q_err);

        P_ += center_weights_c_[i] * (dX * dX.transpose());
    }

    P_ += Q_ * dt; // Add process noise

    x_ = x_pred_mean;
}

bool ESUKF::CorrectPosition(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& R, double gate_threshold) {
    // 1. Redraw Sigma Points based on predicted P?
    Eigen::LLT<MatrixE> lltOfP(P_ + 1e-9 * MatrixE::Identity());
    if (lltOfP.info() == Eigen::NumericalIssue) {
        return false;
    }
    MatrixE L = lltOfP.matrixL();
    L *= std::sqrt(ERROR_DIM + lambda_);

    // Assuming we can overwrite error_sigmas_ since Predict needs to regenerate strictly anyway
    error_sigmas_[0].setZero();
    for (int i = 0; i < ERROR_DIM; ++i) {
        error_sigmas_[i + 1] = L.col(i);
        error_sigmas_[i + 1 + ERROR_DIM] = -L.col(i);
    }

    // 2. Predict Measurements
    // Z is just Position (3D)
    int meas_dim = 3;
    // Z_sigmas_ is resized in ctor
    
    // Mean z
    Eigen::Vector3d z_pred_mean = Eigen::Vector3d::Zero();
    int num_sigma = 2 * ERROR_DIM + 1;

    for (int i = 0; i < num_sigma; ++i) {
        // Reconstruct full state X_i
        VectorS X_curr = x_;
        const VectorE& dX = error_sigmas_[i];
        
        // Apply error to pos
        Eigen::Vector3d pos_i = X_curr.segment<3>(0) + dX.segment<3>(0);
        
        Z_sigmas_.col(i) = pos_i;
        
        z_pred_mean += center_weights_m_[i] * pos_i;
    }

    // 3. Compute Innovation Covariance S and Cross Covariance P_xz
    S_.setZero(); // 3x3
    P_xz_.setZero(); // 15x3

    for (int i = 0; i < num_sigma; ++i) {
        Eigen::VectorXd z_diff = Z_sigmas_.col(i) - z_pred_mean;
        const VectorE& x_err = error_sigmas_[i];

        S_ += center_weights_c_[i] * (z_diff * z_diff.transpose());
        P_xz_ += center_weights_c_[i] * (x_err * z_diff.transpose());
    }

    S_ += R;

    // 4. Mahalanobis Gating
    Eigen::VectorXd innovation = measurement - z_pred_mean;
    Eigen::MatrixXd S_inv = S_.inverse();
    double mahalanobis_dist_sq = innovation.transpose() * S_inv * innovation;

    if (mahalanobis_dist_sq > gate_threshold) {
        // REJECT (Gating)
        return false; 
    }

    // 5. Update
    Eigen::MatrixXd K = P_xz_ * S_inv;
    VectorE delta_x = K * innovation;
    
    // Inject error into state
    x_.segment<3>(0) += delta_x.segment<3>(0);
    x_.segment<3>(3) += delta_x.segment<3>(3);
    x_.segment<3>(10) += delta_x.segment<3>(9);
    x_.segment<3>(13) += delta_x.segment<3>(12);

    // Quaternion update
    Eigen::Quaterniond q(x_(6), x_(7), x_(8), x_(9));
    Eigen::Quaterniond q_cor = RotVecToQuat(delta_x.segment<3>(6));
    q = q * q_cor;
    q.normalize();
    x_(6) = q.w(); x_(7) = q.x(); x_(8) = q.y(); x_(9) = q.z();

    // Update Covariance
    P_ = P_ - K * S_ * K.transpose();

    return true;
}

Eigen::Vector3d ESUKF::QuatToRotVec(const Eigen::Quaterniond& q) {
    Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
}

Eigen::Quaterniond ESUKF::RotVecToQuat(const Eigen::Vector3d& v) {
    double angle = v.norm();
    if (angle < 1e-6) {
        return Eigen::Quaterniond::Identity();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, v / angle));
}

} // namespace estimators
} // namespace lib9427
