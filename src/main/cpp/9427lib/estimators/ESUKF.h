#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

namespace lib9427 {
namespace estimators {

/**
 * Error-State Unscented Kalman Filter (ES-UKF)
 *
 * State Definition (16 dimensions):
 * [0-2]   Position (px, py, pz)
 * [3-5]   Velocity (vx, vy, vz)
 * [6-9]   Attitude Quaternion (qw, qx, qy, qz)
 * [10-12] Accel Bias (bax, bay, baz)
 * [13-15] Gyro Bias (bgx, bgy, bgz)
 *
 * Error State Definition (15 dimensions):
 * [0-2]   delta Position
 * [3-5]   delta Velocity
 * [6-8]   delta Theta (Rotation Vector)
 * [9-11]  delta Accel Bias
 * [12-14] delta Gyro Bias
 */
class ESUKF {
public:
    // Fixed size for efficiency
    static constexpr int STATE_DIM = 16;
    static constexpr int ERROR_DIM = 15;

    using VectorS = Eigen::Matrix<double, STATE_DIM, 1>;
    using VectorE = Eigen::Matrix<double, ERROR_DIM, 1>;
    using MatrixE = Eigen::Matrix<double, ERROR_DIM, ERROR_DIM>;

    ESUKF();

    /**
     * Initialize the filter state.
     */
    void Initialize(const Eigen::Vector3d& init_pos, const Eigen::Quaterniond& init_quat);

    /**
     * Prediction step using IMU data.
     * @param accel IMU accelerometer reading (m/s^2)
     * @param gyro IMU gyroscope reading (rad/s)
     * @param dt time step (s)
     */
    void Predict(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, double dt);

    /**
     * Correction step using generic position measurement (e.g., Vision/Odometry).
     * Applies Mahalanobis Gating.
     *
     * @param measurement Measurement vector
     * @param R Measurement noise covariance
     * @param gate_threshold Chi-square threshold for gating (e.g., 3.0 -> ~99%)
     * @return true if accepted, false if rejected (outlier)
     */
    bool CorrectPosition(const Eigen::Vector3d& position, const Eigen::Matrix3d& R, double gate_threshold);

    // Getters
    Eigen::Vector3d GetPosition() const { return x_.segment<3>(0); }
    Eigen::Vector3d GetVelocity() const { return x_.segment<3>(3); }
    Eigen::Quaterniond GetQuaternion() const {
        return Eigen::Quaterniond(x_(6), x_(7), x_(8), x_(9)).normalized();
    }

    VectorS GetStateVector() const { return x_; }
    MatrixE GetCovariance() const { return P_; }

    void SetProcessNoise(const MatrixE& Q) { Q_ = Q; }

private:
    // Nominal State
    VectorS x_;

    // Error Covariance
    MatrixE P_;

    // Process Noise
    MatrixE Q_;

    // UKF Parameters
    double alpha_ = 0.001;
    double kappa_ = 0.0;
    double beta_ = 2.0;
    double lambda_;

    // Weights
    std::vector<double> center_weights_m_;
    std::vector<double> center_weights_c_;

    // Pre-allocated buffers to avoid heap allocation in loops
    std::vector<VectorE> error_sigmas_;
    std::vector<VectorS> propagated_states_;
    Eigen::MatrixXd Z_sigmas_; // Resize in ctor
    Eigen::MatrixXd S_;        // Resize in ctor
    Eigen::MatrixXd P_xz_;     // Resize in ctor

    // Helper: Quat to Rotation Vector (Error state mapping)
    Eigen::Vector3d QuatToRotVec(const Eigen::Quaterniond& q);

    // Helper: Rotation Vector to Quat (Error state mapping)
    // Helper: Rotation Vector to Quat (Error state mapping)
    Eigen::Quaterniond RotVecToQuat(const Eigen::Vector3d& v);

    // Extrinsics: Rotation from IMU frame to Body frame
    Eigen::Quaterniond imu_extrinsics_q_ = Eigen::Quaterniond::Identity();
    // Extrinsics: Position offset from Robot Center to IMU (in Body Frame)
    Eigen::Vector3d imu_extrinsics_p_ = Eigen::Vector3d::Zero();

    // Previous Gyro Measurement for Angular Acceleration Estimation
    Eigen::Vector3d last_gyro_ = Eigen::Vector3d::Zero();
    bool first_gyro_ = true;

public:
    /**
     * Sets the static extrinsics from the IMU frame to the Robot Body frame.
     * @param imu_to_body_q The quaternion representing the rotation.
     * @param imu_to_body_p The vector representing the position offset (Body Frame).
     */
    void SetExtrinsics(const Eigen::Quaterniond& imu_to_body_q, const Eigen::Vector3d& imu_to_body_p);

    /**
     * Performs a Zero Velocity Update (ZUPT).
     * Call this when the chassis indicates it is stopped (e.g. wheel speeds are 0).
     * This acts as a measurement of [0,0,0] velocity with very low covariance.
     * @param measurement Measurement vector (usually 0,0,0)
     * @param R Measurement noise covariance
     * @param gate_threshold Chi-square threshold
     */
    bool CorrectVelocity(const Eigen::Vector3d& measurement, const Eigen::Matrix3d& R, double gate_threshold);
};

} // namespace estimators
} // namespace lib9427
