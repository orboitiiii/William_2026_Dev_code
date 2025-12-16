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

} // extern "C"
