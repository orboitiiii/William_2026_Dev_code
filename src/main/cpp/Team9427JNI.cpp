#include <jni.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>

#include "9427lib/controllers/lqr.h"
#include "9427lib/controllers/feedforward.h"
#include "9427lib/estimators/KalmanFilter.h"
#include "9427lib/estimators/ESUKF.h"
#include "9427lib/math/discretization.h"
#include "9427lib/modeling/models.h"
#include "9427lib/controllers/DeltaUController.h"
#include "9427lib/solvers/BallisticSolver.h"
#include "9427lib/solvers/MPCSolver.h"

extern "C" {

// ----------------------------------------------------------------------------
// LQR
// ----------------------------------------------------------------------------
JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_computeLQR(
    JNIEnv* env, jclass,
    jdoubleArray A_arr, jdoubleArray B_arr,
    jdoubleArray Q_arr, jdoubleArray R_arr,
    jint states, jint inputs) {

    jdouble* A_ptr = env->GetDoubleArrayElements(A_arr, nullptr);
    jdouble* B_ptr = env->GetDoubleArrayElements(B_arr, nullptr);
    jdouble* Q_ptr = env->GetDoubleArrayElements(Q_arr, nullptr);
    jdouble* R_ptr = env->GetDoubleArrayElements(R_arr, nullptr);

    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    Eigen::Map<MatrixRM> A(A_ptr, states, states);
    Eigen::Map<MatrixRM> B(B_ptr, states, inputs);
    Eigen::Map<MatrixRM> Q(Q_ptr, states, states);
    Eigen::Map<MatrixRM> R(R_ptr, inputs, inputs);

    MatrixRM K_res;
    if (states == 2 && inputs == 1) {
         auto K = lib9427::controllers::LQR<2, 1>::ComputeGain(A, B, Q, R);
         K_res = K;
    } else {
         auto K = lib9427::controllers::LQR<Eigen::Dynamic, Eigen::Dynamic>::ComputeGain(A, B, Q, R);
         K_res = K;
    }

    env->ReleaseDoubleArrayElements(A_arr, A_ptr, 0);
    env->ReleaseDoubleArrayElements(B_arr, B_ptr, 0);
    env->ReleaseDoubleArrayElements(Q_arr, Q_ptr, 0);
    env->ReleaseDoubleArrayElements(R_arr, R_ptr, 0);

    jdoubleArray result = env->NewDoubleArray(inputs * states);
    std::vector<double> outData(inputs * states);
    Eigen::Map<MatrixRM> outMap(outData.data(), inputs, states);
    outMap = K_res;
    env->SetDoubleArrayRegion(result, 0, inputs * states, outData.data());

    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_computePlantInversion(
    JNIEnv* env, jclass,
    jdoubleArray A_arr, jdoubleArray B_arr,
    jdoubleArray r_k_arr, jdoubleArray r_next_arr,
    jint states, jint inputs) {

    jdouble* A_ptr = env->GetDoubleArrayElements(A_arr, nullptr);
    jdouble* B_ptr = env->GetDoubleArrayElements(B_arr, nullptr);
    jdouble* rk_ptr = env->GetDoubleArrayElements(r_k_arr, nullptr);
    jdouble* rn_ptr = env->GetDoubleArrayElements(r_next_arr, nullptr);

    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<MatrixRM> A(A_ptr, states, states);
    Eigen::Map<MatrixRM> B(B_ptr, states, inputs);
    Eigen::Map<Eigen::VectorXd> r_k(rk_ptr, states);
    Eigen::Map<Eigen::VectorXd> r_next(rn_ptr, states);

    Eigen::VectorXd u_ff = lib9427::controllers::Feedforward::ComputePlantInversion(A, B, r_k, r_next);

    env->ReleaseDoubleArrayElements(A_arr, A_ptr, 0);
    env->ReleaseDoubleArrayElements(B_arr, B_ptr, 0);
    env->ReleaseDoubleArrayElements(r_k_arr, rk_ptr, 0);
    env->ReleaseDoubleArrayElements(r_next_arr, rn_ptr, 0);

    jdoubleArray result = env->NewDoubleArray(inputs);
    env->SetDoubleArrayRegion(result, 0, inputs, u_ff.data());

    return result;
}

// ----------------------------------------------------------------------------
// Kalman Filter
// ----------------------------------------------------------------------------
JNIEXPORT jlong JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createKF(
    JNIEnv* env, jclass, jint states, jint inputs, jint outputs) {
    auto* kf = new lib9427::estimators::KalmanFilter(states, inputs, outputs);
    return reinterpret_cast<jlong>(kf);
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_deleteKF(
    JNIEnv* env, jclass, jlong handle) {
    if (handle != 0) {
        auto* kf = reinterpret_cast<lib9427::estimators::KalmanFilter*>(handle);
        delete kf;
    }
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_setKFModel(
    JNIEnv* env, jclass, jlong handle,
    jdoubleArray A_arr, jdoubleArray B_arr, jdoubleArray Q_arr,
    jint states, jint inputs) {
    auto* kf = reinterpret_cast<lib9427::estimators::KalmanFilter*>(handle);
    if (!kf) return;

    jdouble* A_ptr = env->GetDoubleArrayElements(A_arr, nullptr);
    jdouble* B_ptr = env->GetDoubleArrayElements(B_arr, nullptr);
    jdouble* Q_ptr = env->GetDoubleArrayElements(Q_arr, nullptr);

    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<MatrixRM> A(A_ptr, states, states);
    Eigen::Map<MatrixRM> B(B_ptr, states, inputs);
    Eigen::Map<MatrixRM> Q(Q_ptr, states, states);

    kf->SetModel(A, B, Q);

    env->ReleaseDoubleArrayElements(A_arr, A_ptr, 0);
    env->ReleaseDoubleArrayElements(B_arr, B_ptr, 0);
    env->ReleaseDoubleArrayElements(Q_arr, Q_ptr, 0);
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_predictKF(
    JNIEnv* env, jclass, jlong handle, jdoubleArray u_arr, jint size_u) {
    auto* kf = reinterpret_cast<lib9427::estimators::KalmanFilter*>(handle);
    if (!kf) return;
    jdouble* u_ptr = env->GetDoubleArrayElements(u_arr, nullptr);
    Eigen::Map<Eigen::VectorXd> u(u_ptr, size_u);
    kf->Predict(u);
    env->ReleaseDoubleArrayElements(u_arr, u_ptr, 0);
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_correctKF(
    JNIEnv* env, jclass, jlong handle,
    jdoubleArray y_arr, jdoubleArray H_arr, jdoubleArray R_arr,
    jint rows_y, jint cols_H) {
    auto* kf = reinterpret_cast<lib9427::estimators::KalmanFilter*>(handle);
    if (!kf) return;

    jdouble* y_ptr = env->GetDoubleArrayElements(y_arr, nullptr);
    jdouble* H_ptr = env->GetDoubleArrayElements(H_arr, nullptr);
    jdouble* R_ptr = env->GetDoubleArrayElements(R_arr, nullptr);

    Eigen::Map<Eigen::VectorXd> y(y_ptr, rows_y);
    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<MatrixRM> H(H_ptr, rows_y, cols_H);
    Eigen::Map<MatrixRM> R(R_ptr, rows_y, rows_y);

    kf->Correct(y, H, R);

    env->ReleaseDoubleArrayElements(y_arr, y_ptr, 0);
    env->ReleaseDoubleArrayElements(H_arr, H_ptr, 0);
    env->ReleaseDoubleArrayElements(R_arr, R_ptr, 0);
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_getKFXhat(
    JNIEnv* env, jclass, jlong handle, jint states) {
    auto* kf = reinterpret_cast<lib9427::estimators::KalmanFilter*>(handle);
    if (!kf) return nullptr;
    Eigen::VectorXd x_hat = kf->GetXHat();
    jdoubleArray result = env->NewDoubleArray(states);
    env->SetDoubleArrayRegion(result, 0, states, x_hat.data());
    return result;
}

// ----------------------------------------------------------------------------
// Discretization & Modeling
// ----------------------------------------------------------------------------

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_discretizeAB(
    JNIEnv* env, jclass, jdoubleArray A_arr, jdoubleArray B_arr,
    jint states, jint inputs, jdouble dt) {

    jdouble* A_ptr = env->GetDoubleArrayElements(A_arr, nullptr);
    jdouble* B_ptr = env->GetDoubleArrayElements(B_arr, nullptr);

    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    Eigen::Map<MatrixRM> A(A_ptr, states, states);
    Eigen::Map<MatrixRM> B(B_ptr, states, inputs);

    Eigen::MatrixXd Ad(states, states);
    Eigen::MatrixXd Bd(states, inputs);

    lib9427::math::DiscretizeAB(A, B, dt, Ad, Bd);

    env->ReleaseDoubleArrayElements(A_arr, A_ptr, 0);
    env->ReleaseDoubleArrayElements(B_arr, B_ptr, 0);

    int sizeA = states * states;
    int sizeB = states * inputs;
    jdoubleArray result = env->NewDoubleArray(sizeA + sizeB);

    MatrixRM Ad_rm = Ad;
    MatrixRM Bd_rm = Bd;

    env->SetDoubleArrayRegion(result, 0, sizeA, Ad_rm.data());
    env->SetDoubleArrayRegion(result, sizeA, sizeB, Bd_rm.data());
    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createElevator(
    JNIEnv* env, jclass,
    jdouble Kt, jdouble Kv, jdouble R,
    jdouble m, jdouble r, jdouble G) {

    lib9427::modeling::MotorConfig motor{Kt, Kv, R};
    auto sys = lib9427::modeling::SystemFactory::CreateElevator(motor, m, r, G);

    int states = 2; int inputs = 1;
    int sizeA = states * states;
    int sizeB = states * inputs;

    jdoubleArray result = env->NewDoubleArray(sizeA + sizeB);

    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    MatrixRM A_rm = sys.A;
    MatrixRM B_rm = sys.B;

    env->SetDoubleArrayRegion(result, 0, sizeA, A_rm.data());
    env->SetDoubleArrayRegion(result, sizeA, sizeB, B_rm.data());
    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createFlywheel(
    JNIEnv* env, jclass, jdouble Kt, jdouble Kv, jdouble R, jdouble J, jdouble G) {

    lib9427::modeling::MotorConfig motor{Kt, Kv, R};
    auto sys = lib9427::modeling::SystemFactory::CreateFlywheel(motor, J, G);

    int states = 1; int inputs = 1;
    int sizeA = states * states;
    int sizeB = states * inputs;

    jdoubleArray result = env->NewDoubleArray(sizeA + sizeB);

    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    MatrixRM A_rm = sys.A; MatrixRM B_rm = sys.B;

    env->SetDoubleArrayRegion(result, 0, sizeA, A_rm.data());
    env->SetDoubleArrayRegion(result, sizeA, sizeB, B_rm.data());
    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createSingleArm(
    JNIEnv* env, jclass, jdouble Kt, jdouble Kv, jdouble R, jdouble J, jdouble G) {

    lib9427::modeling::MotorConfig motor{Kt, Kv, R};
    auto sys = lib9427::modeling::SystemFactory::CreateSingleArm(motor, J, G);

    int states = 2; int inputs = 1;
    int sizeA = states * states;
    int sizeB = states * inputs;

    jdoubleArray result = env->NewDoubleArray(sizeA + sizeB);
    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    MatrixRM A_rm = sys.A; MatrixRM B_rm = sys.B;

    env->SetDoubleArrayRegion(result, 0, sizeA, A_rm.data());
    env->SetDoubleArrayRegion(result, sizeA, sizeB, B_rm.data());
    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createDrivetrain(
    JNIEnv* env, jclass, jdouble Kt, jdouble Kv, jdouble R, jdouble m, jdouble r, jdouble rb, jdouble J, jdouble G) {

    lib9427::modeling::MotorConfig motor{Kt, Kv, R};
    auto sys = lib9427::modeling::SystemFactory::CreateDrivetrain(motor, m, r, rb, J, G);

    int states = 2; int inputs = 2;
    int sizeA = states * states;
    int sizeB = states * inputs;

    jdoubleArray result = env->NewDoubleArray(sizeA + sizeB);
    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    MatrixRM A_rm = sys.A; MatrixRM B_rm = sys.B;

    env->SetDoubleArrayRegion(result, 0, sizeA, A_rm.data());
    env->SetDoubleArrayRegion(result, sizeA, sizeB, B_rm.data());
    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createDoubleArm(
    JNIEnv* env, jclass,
    jdouble Kt1, jdouble Kv1, jdouble R1,
    jdouble Kt2, jdouble Kv2, jdouble R2,
    jdouble l1, double l2,
    double m1, double m2,
    double I1, double I2,
    double G1, double G2) {

    lib9427::modeling::MotorConfig m_1{Kt1, Kv1, R1};
    lib9427::modeling::MotorConfig m_2{Kt2, Kv2, R2};
    auto sys = lib9427::modeling::SystemFactory::CreateDoubleArm(m_1, m_2, l1, l2, m1, m2, I1, I2, G1, G2);

    int states = 4; int inputs = 2;
    int sizeA = states * states;
    int sizeB = states * inputs;
    jdoubleArray result = env->NewDoubleArray(sizeA + sizeB);
    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    MatrixRM A_rm = sys.A; MatrixRM B_rm = sys.B;
    env->SetDoubleArrayRegion(result, 0, sizeA, A_rm.data());
    env->SetDoubleArrayRegion(result, sizeA, sizeB, B_rm.data());
    return result;
}

// ----------------------------------------------------------------------------
// ES-UKF (Error State Unscented Kalman Filter)
// ----------------------------------------------------------------------------

JNIEXPORT jlong JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createESUKF(
    JNIEnv* env, jclass) {
    auto* kf = new lib9427::estimators::ESUKF();
    return reinterpret_cast<jlong>(kf);
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_deleteESUKF(
    JNIEnv* env, jclass, jlong handle) {
    if (handle != 0) {
        auto* kf = reinterpret_cast<lib9427::estimators::ESUKF*>(handle);
        delete kf;
    }
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_initESUKF(
    JNIEnv* env, jclass, jlong handle,
    jdouble px, jdouble py, jdouble pz,
    jdouble qw, jdouble qx, jdouble qy, jdouble qz) {
    auto* kf = reinterpret_cast<lib9427::estimators::ESUKF*>(handle);
    if (!kf) return;

    kf->Initialize(Eigen::Vector3d(px, py, pz), Eigen::Quaterniond(qw, qx, qy, qz));
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_predictESUKF(
    JNIEnv* env, jclass, jlong handle,
    jdouble ax, jdouble ay, jdouble az,
    jdouble gx, jdouble gy, jdouble gz,
    jdouble dt) {
    auto* kf = reinterpret_cast<lib9427::estimators::ESUKF*>(handle);
    if (!kf) return;

    kf->Predict(Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(gx, gy, gz), dt);
}

JNIEXPORT jboolean JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_correctESUKF(
    JNIEnv* env, jclass, jlong handle,
    jdouble px, jdouble py, jdouble pz,
    jdoubleArray R_arr, jdouble gate_threshold) {
    auto* kf = reinterpret_cast<lib9427::estimators::ESUKF*>(handle);
    if (!kf) return false;

    jdouble* R_ptr = env->GetDoubleArrayElements(R_arr, nullptr);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(R_ptr);

    bool accepted = kf->CorrectPosition(Eigen::Vector3d(px, py, pz), R, gate_threshold);

    env->ReleaseDoubleArrayElements(R_arr, R_ptr, 0);
    return accepted;
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_setESUKFExtrinsics(
    JNIEnv* env, jclass, jlong handle,
    jdouble qw, jdouble qx, jdouble qy, jdouble qz,
    jdouble px, jdouble py, jdouble pz) {
    auto* kf = reinterpret_cast<lib9427::estimators::ESUKF*>(handle);
    if (!kf) return;

    kf->SetExtrinsics(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(px, py, pz));
}

JNIEXPORT jboolean JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_correctESUKFVelocity(
    JNIEnv* env, jclass, jlong handle,
    jdouble vx, jdouble vy, jdouble vz,
    jdoubleArray R_arr, jdouble gate_threshold) {
    auto* kf = reinterpret_cast<lib9427::estimators::ESUKF*>(handle);
    if (!kf) return false;

    jdouble* R_ptr = env->GetDoubleArrayElements(R_arr, nullptr);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(R_ptr);

    bool accepted = kf->CorrectVelocity(Eigen::Vector3d(vx, vy, vz), R, gate_threshold);

    env->ReleaseDoubleArrayElements(R_arr, R_ptr, 0);
    return accepted;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_getESUKFState(
    JNIEnv* env, jclass, jlong handle) {
    auto* kf = reinterpret_cast<lib9427::estimators::ESUKF*>(handle);
    if (!kf) return nullptr;

    // Return Pos(3), Vel(3), Quat(4) -> size 10
    // Biases optional if needed but usually just p/v/q is enough for control
    jdoubleArray result = env->NewDoubleArray(10);
    std::vector<double> outData(10);

    Eigen::Vector3d p = kf->GetPosition();
    Eigen::Vector3d v = kf->GetVelocity();
    Eigen::Quaterniond q = kf->GetQuaternion();

    outData[0] = p.x(); outData[1] = p.y(); outData[2] = p.z();
    outData[3] = v.x(); outData[4] = v.y(); outData[5] = v.z();
    outData[6] = q.w(); outData[7] = q.x(); outData[8] = q.y(); outData[9] = q.z();

    env->SetDoubleArrayRegion(result, 0, 10, outData.data());
    return result;
}

// ----------------------------------------------------------------------------
// Delta U Controller
// ----------------------------------------------------------------------------


// Instantiate generic type for ease
using DeltaU = lib9427::controllers::DeltaUController<Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic>;

JNIEXPORT jlong JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createDeltaU(
    JNIEnv* env, jclass,
    jdoubleArray A_arr, jdoubleArray B_arr, jdoubleArray C_arr,
    jdoubleArray Q_nom_arr, jdoubleArray R_nom_arr,
    jdoubleArray Q_state_arr, jdoubleArray Q_dist_arr, jdoubleArray R_meas_arr,
    jint states, jint inputs, jint outputs) {

    jdouble* A_ptr = env->GetDoubleArrayElements(A_arr, nullptr);
    jdouble* B_ptr = env->GetDoubleArrayElements(B_arr, nullptr);
    jdouble* C_ptr = env->GetDoubleArrayElements(C_arr, nullptr);
    jdouble* Qn_ptr = env->GetDoubleArrayElements(Q_nom_arr, nullptr);
    jdouble* Rn_ptr = env->GetDoubleArrayElements(R_nom_arr, nullptr);
    jdouble* Qs_ptr = env->GetDoubleArrayElements(Q_state_arr, nullptr);
    jdouble* Qd_ptr = env->GetDoubleArrayElements(Q_dist_arr, nullptr);
    jdouble* Rm_ptr = env->GetDoubleArrayElements(R_meas_arr, nullptr);

    using MatrixRM = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    Eigen::Map<MatrixRM> A(A_ptr, states, states);
    Eigen::Map<MatrixRM> B(B_ptr, states, inputs);
    Eigen::Map<MatrixRM> C(C_ptr, outputs, states);

    DeltaU::Config config;
    config.Q_nominal = Eigen::Map<MatrixRM>(Qn_ptr, states, states);
    config.R_nominal = Eigen::Map<MatrixRM>(Rn_ptr, inputs, inputs);
    config.Q_state = Eigen::Map<MatrixRM>(Qs_ptr, states, states);
    config.Q_disturbance = Eigen::Map<MatrixRM>(Qd_ptr, inputs, inputs);
    config.R_measurement = Eigen::Map<MatrixRM>(Rm_ptr, outputs, outputs);

    auto* controller = new DeltaU(A, B, C, config);

    env->ReleaseDoubleArrayElements(A_arr, A_ptr, 0);
    env->ReleaseDoubleArrayElements(B_arr, B_ptr, 0);
    env->ReleaseDoubleArrayElements(C_arr, C_ptr, 0);
    env->ReleaseDoubleArrayElements(Q_nom_arr, Qn_ptr, 0);
    env->ReleaseDoubleArrayElements(R_nom_arr, Rn_ptr, 0);
    env->ReleaseDoubleArrayElements(Q_state_arr, Qs_ptr, 0);
    env->ReleaseDoubleArrayElements(Q_dist_arr, Qd_ptr, 0);
    env->ReleaseDoubleArrayElements(R_meas_arr, Rm_ptr, 0);

    return reinterpret_cast<jlong>(controller);
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_deleteDeltaU(
    JNIEnv* env, jclass, jlong handle) {
    if (handle != 0) {
        auto* ctrl = reinterpret_cast<DeltaU*>(handle);
        delete ctrl;
    }
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_updateDeltaU(
    JNIEnv* env, jclass, jlong handle,
    jdoubleArray y_arr, jdoubleArray r_arr, jdoubleArray u_ff_arr, jdoubleArray u_prev_arr,
    jint states, jint inputs, jint outputs) {

    auto* ctrl = reinterpret_cast<DeltaU*>(handle);
    if (!ctrl) return nullptr;

    jdouble* y_ptr = env->GetDoubleArrayElements(y_arr, nullptr);
    jdouble* r_ptr = env->GetDoubleArrayElements(r_arr, nullptr);
    jdouble* uff_ptr = env->GetDoubleArrayElements(u_ff_arr, nullptr);
    jdouble* uprev_ptr = env->GetDoubleArrayElements(u_prev_arr, nullptr);

    Eigen::Map<Eigen::VectorXd> y(y_ptr, outputs);
    Eigen::Map<Eigen::VectorXd> r(r_ptr, states);
    Eigen::Map<Eigen::VectorXd> u_ff(uff_ptr, inputs);
    Eigen::Map<Eigen::VectorXd> u_prev(uprev_ptr, inputs);

    Eigen::VectorXd u = ctrl->Update(y, r, u_ff, u_prev);

    env->ReleaseDoubleArrayElements(y_arr, y_ptr, 0);
    env->ReleaseDoubleArrayElements(r_arr, r_ptr, 0);
    env->ReleaseDoubleArrayElements(u_ff_arr, uff_ptr, 0);
    env->ReleaseDoubleArrayElements(u_prev_arr, uprev_ptr, 0);

    jdoubleArray result = env->NewDoubleArray(inputs);
    env->SetDoubleArrayRegion(result, 0, inputs, u.data());
    return result;
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_resetDeltaU(
    JNIEnv* env, jclass, jlong handle, jdoubleArray x0_arr, jint states) {
    auto* ctrl = reinterpret_cast<DeltaU*>(handle);
    if (!ctrl) return;
    jdouble* x0_ptr = env->GetDoubleArrayElements(x0_arr, nullptr);
    Eigen::Map<Eigen::VectorXd> x0(x0_ptr, states);
    ctrl->Reset(x0);
    env->ReleaseDoubleArrayElements(x0_arr, x0_ptr, 0);
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_resetDeltaUStateOnly(
    JNIEnv* env, jclass, jlong handle, jdoubleArray x0_arr, jint states) {
    auto* ctrl = reinterpret_cast<DeltaU*>(handle);
    if (!ctrl) return;
    jdouble* x0_ptr = env->GetDoubleArrayElements(x0_arr, nullptr);
    Eigen::Map<Eigen::VectorXd> x0(x0_ptr, states);
    ctrl->ResetStateOnly(x0);
    env->ReleaseDoubleArrayElements(x0_arr, x0_ptr, 0);
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_getDeltaUState(
    JNIEnv* env, jclass, jlong handle, jint states) {
    auto* ctrl = reinterpret_cast<DeltaU*>(handle);
    if (!ctrl) return nullptr;
    Eigen::VectorXd x_hat = ctrl->GetEstimatedState();
    jdoubleArray result = env->NewDoubleArray(states);
    env->SetDoubleArrayRegion(result, 0, states, x_hat.data());
    return result;
}

JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_getDeltaUDisturbance(
    JNIEnv* env, jclass, jlong handle, jint inputs) {
    auto* ctrl = reinterpret_cast<DeltaU*>(handle);
    if (!ctrl) return nullptr;
    Eigen::VectorXd d_hat = ctrl->GetEstimatedDisturbance();
    jdoubleArray result = env->NewDoubleArray(inputs);
    env->SetDoubleArrayRegion(result, 0, inputs, d_hat.data());
    return result;
}

// ----------------------------------------------------------------------------
// Ballistic Solver
// ----------------------------------------------------------------------------

// Type alias for convenience
using BallisticSolver = lib9427::solvers::BallisticSolver;

/**
 * Creates a new BallisticSolver instance.
 *
 * @return Native handle (pointer to BallisticSolver).
 */
JNIEXPORT jlong JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createBallisticSolver(
    JNIEnv* env, jclass) {
    auto* solver = new BallisticSolver();
    return reinterpret_cast<jlong>(solver);
}

/**
 * Deletes a BallisticSolver instance.
 *
 * @param handle Native handle from createBallisticSolver.
 */
JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_deleteBallisticSolver(
    JNIEnv* env, jclass, jlong handle) {
    if (handle != 0) {
        auto* solver = reinterpret_cast<BallisticSolver*>(handle);
        delete solver;
    }
}

/**
 * Configures projectile physical parameters.
 *
 * @param handle Native handle.
 * @param mass Mass [kg].
 * @param diameter Diameter [m].
 * @param cd Drag coefficient.
 * @param cm Magnus coefficient.
 * @param moi Moment of inertia [kg*m^2].
 * @param spinDecay Spin decay coefficient.
 */
JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_configureBallisticSolver(
    JNIEnv* env, jclass, jlong handle,
    jdouble mass, jdouble diameter, jdouble cd, jdouble cm,
    jdouble moi, jdouble spinDecay) {
    auto* solver = reinterpret_cast<BallisticSolver*>(handle);
    if (!solver) return;

    lib9427::solvers::ProjectileConfig config;
    config.mass_kg = mass;
    config.diameter_m = diameter;
    config.drag_coefficient = cd;
    config.magnus_coefficient = cm;
    config.moment_of_inertia = moi;
    config.spin_decay_coeff = spinDecay;
    solver->configure(config);
}

/**
 * Sets shooter mechanism constraints.
 *
 * @param handle Native handle.
 * @param minRpm Minimum RPM.
 * @param maxRpm Maximum RPM.
 * @param wheelRadius Shooter wheel radius [m].
 * @param pitchMin Minimum pitch [rad].
 * @param pitchMax Maximum pitch [rad].
 * @param turretMaxRate Maximum turret slew rate [rad/s].
 */
JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_setBallisticShooterConstraints(
    JNIEnv* env, jclass, jlong handle,
    jdouble minRpm, jdouble maxRpm, jdouble wheelRadius,
    jdouble pitchMin, jdouble pitchMax, jdouble turretMaxRate) {
    auto* solver = reinterpret_cast<BallisticSolver*>(handle);
    if (!solver) return;

    lib9427::solvers::ShooterConstraints constraints;
    constraints.min_rpm = minRpm;
    constraints.max_rpm = maxRpm;
    constraints.wheel_radius_m = wheelRadius;
    constraints.pitch_min_rad = pitchMin;
    constraints.pitch_max_rad = pitchMax;
    constraints.turret_max_rate_radps = turretMaxRate;
    solver->setShooterConstraints(constraints);
}

/**
 * Sets target specification.
 *
 * @param handle Native handle.
 * @param centerX Target center X [m].
 * @param centerY Target center Y [m].
 * @param centerZ Target center height [m].
 */
JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_setBallisticTarget(
    JNIEnv* env, jclass, jlong handle,
    jdouble centerX, jdouble centerY, jdouble centerZ) {
    auto* solver = reinterpret_cast<BallisticSolver*>(handle);
    if (!solver) return;

    lib9427::solvers::TargetSpec target;
    target.center_x_m = centerX;
    target.center_y_m = centerY;
    target.center_z_m = centerZ;
    solver->setTarget(target);
}

/**
 * Sets integration time step.
 *
 * @param handle Native handle.
 * @param dt Time step [s]. Must be <= 0.01s.
 */
JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_setBallisticTimeStep(
    JNIEnv* env, jclass, jlong handle, jdouble dt) {
    auto* solver = reinterpret_cast<BallisticSolver*>(handle);
    if (!solver) return;

    try {
        solver->setTimeStep(dt);
    } catch (const std::exception& e) {
        // Log error but don't throw - use default
        std::cerr << "BallisticSolver: Invalid time step: " << e.what() << std::endl;
    }
}

/**
 * Sets projectile spin rate.
 *
 * @param handle Native handle.
 * @param spinRpm Backspin rate [RPM].
 */
JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_setBallisticSpinRate(
    JNIEnv* env, jclass, jlong handle, jdouble spinRpm) {
    auto* solver = reinterpret_cast<BallisticSolver*>(handle);
    if (!solver) return;
    solver->setSpinRate(spinRpm);
}

/**
 * Solves for optimal shooting parameters.
 *
 * @param handle Native handle.
 * @param robotX Robot X position [m].
 * @param robotY Robot Y position [m].
 * @param robotVx Robot X velocity [m/s].
 * @param robotVy Robot Y velocity [m/s].
 * @param robotOmega Robot angular velocity [rad/s].
 * @param robotHeading Robot heading [rad].
 * @param turretHeight Turret pivot height [m].
 * @param turretYaw Current turret yaw [rad].
 * @param turretYawRate Turret yaw velocity [rad/s].
 * @param turretOffsetX Turret X offset from robot center [m].
 * @param turretOffsetY Turret Y offset from robot center [m].
 * @return Solution array [valid, yaw, pitch, velocity, rpm, tof,
 *         impactX, impactY, impactZ, yawLead, turretTravelTime]
 *         (11 elements). valid = 1.0 if solution exists, 0.0 otherwise.
 */
JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_solveBallisticSolution(
    JNIEnv* env, jclass, jlong handle,
    jdouble robotX, jdouble robotY, jdouble robotVx, jdouble robotVy,
    jdouble robotOmega, jdouble robotHeading,
    jdouble turretHeight, jdouble turretYaw, jdouble turretYawRate,
    jdouble turretOffsetX, jdouble turretOffsetY,
    jdoubleArray results_arr) {

    auto* solver = reinterpret_cast<BallisticSolver*>(handle);
    if (!solver) return;

    // Build robot state
    lib9427::solvers::RobotState robot;
    robot.x_m = robotX;
    robot.y_m = robotY;
    robot.vx_mps = robotVx;
    robot.vy_mps = robotVy;
    robot.omega_radps = robotOmega;
    robot.heading_rad = robotHeading;
    robot.turret_height_m = turretHeight;
    robot.turret_yaw_rad = turretYaw;
    robot.turret_yaw_rate_radps = turretYawRate;
    robot.turret_offset_x_m = turretOffsetX;
    robot.turret_offset_y_m = turretOffsetY;

    // Solve
    auto sol = solver->solve(robot);

    // Pack result: 11 values
    constexpr int RESULT_SIZE = 11;
    // CRITICAL: Writing to pre-allocated buffer to avoid GC
    // We assume the caller ensures the array is at least size 11
    
    std::array<double, RESULT_SIZE> outData;
    outData[0] = sol.valid ? 1.0 : 0.0;
    outData[1] = sol.yaw_rad;
    outData[2] = sol.pitch_rad;
    outData[3] = sol.velocity_mps;
    outData[4] = sol.rpm;
    outData[5] = sol.time_of_flight_s;
    outData[6] = sol.impact_x_m;
    outData[7] = sol.impact_y_m;
    outData[8] = sol.impact_z_m;
    outData[9] = sol.yaw_lead_rad;
    outData[10] = sol.turret_travel_time_s;

    env->SetDoubleArrayRegion(results_arr, 0, RESULT_SIZE, outData.data());
}

/**
 * Simulates trajectory for given shooting parameters.
 *
 * @param handle Native handle.
 * @param yaw Launch yaw [rad].
 * @param pitch Launch pitch [rad].
 * @param velocity Exit velocity [m/s].
 * @param robotX Robot X position [m].
 * @param robotY Robot Y position [m].
 * @param robotVx Robot X velocity [m/s].
 * @param robotVy Robot Y velocity [m/s].
 * @param robotOmega Robot angular velocity [rad/s].
 * @param robotHeading Robot heading [rad].
 * @param turretHeight Turret height [m].
 * @param turretOffsetX Turret X offset [m].
 * @param turretOffsetY Turret Y offset [m].
 * @param maxPoints Maximum trajectory points to return.
 * @return Flattened trajectory array [t0, x0, y0, z0, vx0, vy0, vz0, t1, ...]
 *         Each point is 7 values. Returns numPoints * 7 elements.
 */
JNIEXPORT jdoubleArray JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_simulateBallisticTrajectory(
    JNIEnv* env, jclass, jlong handle,
    jdouble yaw, jdouble pitch, jdouble velocity,
    jdouble robotX, jdouble robotY, jdouble robotVx, jdouble robotVy,
    jdouble robotOmega, jdouble robotHeading,
    jdouble turretHeight, jdouble turretOffsetX, jdouble turretOffsetY,
    jint maxPoints) {

    auto* solver = reinterpret_cast<BallisticSolver*>(handle);
    if (!solver) return nullptr;

    // Build robot state
    lib9427::solvers::RobotState robot;
    robot.x_m = robotX;
    robot.y_m = robotY;
    robot.vx_mps = robotVx;
    robot.vy_mps = robotVy;
    robot.omega_radps = robotOmega;
    robot.heading_rad = robotHeading;
    robot.turret_height_m = turretHeight;
    robot.turret_yaw_rad = 0.0;  // Not needed for simulation
    robot.turret_yaw_rate_radps = 0.0;
    robot.turret_offset_x_m = turretOffsetX;
    robot.turret_offset_y_m = turretOffsetY;

    // Simulate
    lib9427::solvers::TrajectoryBuffer buffer;
    if (!solver->simulateTrajectory(yaw, pitch, velocity, robot, buffer)) {
        return nullptr;
    }

    // Limit output points
    size_t numPoints = std::min(buffer.count, static_cast<size_t>(maxPoints));
    constexpr int POINT_SIZE = 7;  // t, x, y, z, vx, vy, vz

    jdoubleArray result = env->NewDoubleArray(numPoints * POINT_SIZE);
    std::vector<double> outData(numPoints * POINT_SIZE);

    for (size_t i = 0; i < numPoints; ++i) {
        const auto& pt = buffer.points[i];
        outData[i * POINT_SIZE + 0] = pt.t;
        outData[i * POINT_SIZE + 1] = pt.x;
        outData[i * POINT_SIZE + 2] = pt.y;
        outData[i * POINT_SIZE + 3] = pt.z;
        outData[i * POINT_SIZE + 4] = pt.vx;
        outData[i * POINT_SIZE + 5] = pt.vy;
        outData[i * POINT_SIZE + 6] = pt.vz;
    }

    env->SetDoubleArrayRegion(result, 0, numPoints * POINT_SIZE, outData.data());
    return result;
}

// ----------------------------------------------------------------------------
// MPC Solver (OSQP)
// ----------------------------------------------------------------------------

// FORCE_REBUILD_TIMESTAMP_2026_01_19_V3
JNIEXPORT jlong JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_createMPCNative(
    JNIEnv* env, jclass, 
    jint horizon, jdouble dt,
    jdoubleArray Q_arr, jdoubleArray R_arr, jdoubleArray Qf_arr,
    jdoubleArray min_u_arr, jdoubleArray max_u_arr,
    jdoubleArray min_x_arr, jdoubleArray max_x_arr) {

    lib9427::solvers::MPCSolver::Config config;
    config.horizon = horizon;
    config.dt = dt;
    
    // Helper to load array to Vector3d
    auto loadVec3 = [&](jdoubleArray arr, Eigen::Vector3d& vec) {
        jdouble* ptr = env->GetDoubleArrayElements(arr, nullptr);
        vec << ptr[0], ptr[1], ptr[2];
        env->ReleaseDoubleArrayElements(arr, ptr, 0);
    };

    loadVec3(Q_arr, config.Q);
    loadVec3(R_arr, config.R);
    loadVec3(Qf_arr, config.Q_final);
    loadVec3(min_u_arr, config.input_min);
    loadVec3(max_u_arr, config.input_max);
    loadVec3(min_x_arr, config.state_min);
    loadVec3(max_x_arr, config.state_max);

    auto* mpc = new lib9427::solvers::MPCSolver(config);
    return reinterpret_cast<jlong>(mpc);
}

JNIEXPORT void JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_deleteMPC(
    JNIEnv* env, jclass, jlong handle) {
    if (handle != 0) {
        auto* mpc = reinterpret_cast<lib9427::solvers::MPCSolver*>(handle);
        delete mpc;
    }
}

JNIEXPORT jint JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_solveMPCNative(
    JNIEnv* env, jclass, jlong handle,
    jdoubleArray current_state_arr,
    jdoubleArray linearized_mats_arr,
    jdoubleArray ref_states_arr,
    jdoubleArray output_u_arr) {

    auto* mpc = reinterpret_cast<lib9427::solvers::MPCSolver*>(handle);
    if (!mpc) return -1;

    // Use GetPrimitiveArrayCritical for potentially faster access (Zero Copy attempt)
    // Or standard GetDoubleArrayElements. Critical is better for short blocking calls.
    // However, OSQP solve time might be > 1ms, which blocks GC if using Critical.
    // Standard GetDoubleArrayElements often copies, but JNI implementations might pin.
    // Given FRC RoboRIO (OpenJDK), Elements is safer to avoid GC lockup during solve.
    
    jdouble* c_state = env->GetDoubleArrayElements(current_state_arr, nullptr);
    jdouble* l_mats = env->GetDoubleArrayElements(linearized_mats_arr, nullptr);
    jdouble* r_states = env->GetDoubleArrayElements(ref_states_arr, nullptr);
    jdouble* out_u = env->GetDoubleArrayElements(output_u_arr, nullptr);

    int status = mpc->Solve(c_state, l_mats, r_states, out_u);

    // Commit output changes
    env->ReleaseDoubleArrayElements(output_u_arr, out_u, 0); // 0 = copy back
    
    // Release inputs (JNI_ABORT = no copy back)
    env->ReleaseDoubleArrayElements(current_state_arr, c_state, JNI_ABORT);
    env->ReleaseDoubleArrayElements(linearized_mats_arr, l_mats, JNI_ABORT);
    env->ReleaseDoubleArrayElements(ref_states_arr, r_states, JNI_ABORT);

    return status;
}

/**
 * Checks if a point is inside the hexagon target.
 *
 * @param xInch X coordinate relative to hexagon center [inches].
 * @param yInch Y coordinate relative to hexagon center [inches].
 * @return True if inside hexagon.
 */
JNIEXPORT jboolean JNICALL Java_frc_robot_libraries_lib9427_Team9427JNI_isInsideHexagon(
    JNIEnv* env, jclass, jdouble xInch, jdouble yInch) {
    return BallisticSolver::isInsideHexagon(xInch, yInch);
}

} // extern "C"
