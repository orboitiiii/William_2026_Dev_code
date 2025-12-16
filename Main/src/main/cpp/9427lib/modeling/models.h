#pragma once

#include <Eigen/Dense>
#include <cmath>
#include "9427lib/math/discretization.h"

namespace lib9427 {
namespace modeling {

struct MotorConfig {
    double Kt; // Nm / Amp
    double Kv; // rad/s / Volt
    double R;  // Ohms
};

struct StateSpaceSystem {
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
};

class SystemFactory {
 public:
  static StateSpaceSystem CreateFlywheel(const MotorConfig& motor, double J, double G) {
      // Equation: V = I*R + w/Kv
      // Torque = Kt*I = J*dw/dt
      // dw/dt = (Kt/J) * (V - w/Kv)/R
      // dw/dt = -Kt/(J*R*Kv) * w + Kt/(J*R) * V
      // With gearing G: w_motor = G*w_load, T_load = G*T_motor
      
      double A_val = -(G * G * motor.Kt) / (J * motor.R * motor.Kv);
      double B_val = (G * motor.Kt) / (J * motor.R);
      
      StateSpaceSystem sys;
      sys.A = Eigen::MatrixXd(1, 1); sys.A(0,0) = A_val;
      sys.B = Eigen::MatrixXd(1, 1); sys.B(0,0) = B_val;
      return sys;
  }

  static StateSpaceSystem CreateElevator(const MotorConfig& motor, double m, double r, double G) {
      double A22 = -(G * G * motor.Kt) / (m * r * r * motor.R * motor.Kv);
      double B2 = (G * motor.Kt) / (m * r * motor.R);

      StateSpaceSystem sys;
      sys.A = Eigen::MatrixXd::Zero(2, 2);
      sys.A(0, 1) = 1.0;
      sys.A(1, 1) = A22;

      sys.B = Eigen::MatrixXd::Zero(2, 1);
      sys.B(1, 0) = B2;
      return sys;
  }
  
  static StateSpaceSystem CreateSingleArm(const MotorConfig& motor, double J, double G) {
      // Similar to Flywheel but state is [theta, omega]
      // A = [0 1; 0 damping]
      double damping = -(G * G * motor.Kt) / (J * motor.R * motor.Kv);
      double B_val = (G * motor.Kt) / (J * motor.R);
      
      StateSpaceSystem sys;
      sys.A = Eigen::MatrixXd::Zero(2, 2);
      sys.A(0, 1) = 1.0;
      sys.A(1, 1) = damping;
      
      sys.B = Eigen::MatrixXd::Zero(2, 1);
      sys.B(1, 0) = B_val;
      return sys;
  }
  
  static StateSpaceSystem CreateDrivetrain(const MotorConfig& motor, double m, double r, double rb, double J, double G) {
      // Standard Drivetrain state-space model
      // C1 = -(G^2 * Kt) / (Kv * R * r^2)
      // C2 = (G * Kt) / (R * r)
      
      double C1 = -(G * G * motor.Kt) / (motor.Kv * motor.R * r * r);
      double C2 = (G * motor.Kt) / (motor.R * r);
      
      double A11 = 0.5 * C1 * (1.0/m + (rb*rb)/J);
      double A12 = 0.5 * C1 * (1.0/m - (rb*rb)/J);
      double B11 = 0.5 * C2 * (1.0/m + (rb*rb)/J);
      double B12 = 0.5 * C2 * (1.0/m - (rb*rb)/J);

      StateSpaceSystem sys;
      sys.A = Eigen::MatrixXd(2, 2);
      sys.A << A11, A12,
               A12, A11;
               
      sys.B = Eigen::MatrixXd(2, 2);
      sys.B << B11, B12,
               B12, B11;
      return sys;
  }
  
  /**
   * Double Joint Arm Linearized Model.
   * Linearized around the vertical equilibrium (theta1=0, theta2=0).
   * 
   * @param l1 Length of arm 1
   * @param l2 Length of arm 2
   * @param m1 Mass of arm 1
   * @param m2 Mass of arm 2
   * @param I1 MOI of arm 1 about pivot
   * @param I2 MOI of arm 2 about pivot
   * @param G1 Gearing 1
   * @param G2 Gearing 2
   */
  static StateSpaceSystem CreateDoubleArm(
      const MotorConfig& motor1, const MotorConfig& motor2,
      double l1, double l2,
      double m1, double m2,
      double I1, double I2,
      double G1, double G2) {
      
      // Constants
      double g = 9.81;
      
      // Center of masses (assumed halfway for uniform rods)
      double r1 = l1 / 2.0;
      double r2 = l2 / 2.0;

      // Mass Matrix M at equilibrium (theta=0)
      // M11 = I1 + m2 * l1^2
      // M12 = m2 * l1 * r2
      // M21 = m2 * l1 * r2
      // M22 = I2
      // Note: If I1/I2 are about pivot, this is correct. If COM, use parallel axis.
      // Assuming I1, I2 are moments of inertia about the joint pivot.
      
      // Correction: Usually inputs are properties of the link.
      // Let's assume input I1, I2 are about the pivot point of that link.
      // But for the coupled system:
      // Kinetic Energy K = 0.5*I1*w1^2 + 0.5*m2*(v_elbow^2) ... no, it's more complex.
      // Simplified Mass Matrix for FRC usage (often modeled as two coupled links):
      
      double M11 = I1 + m2 * l1 * l1;
      double M12 = m2 * l1 * r2;
      double M21 = M12;
      double M22 = I2;
      
      Eigen::Matrix2d M;
      M << M11, M12,
           M21, M22;
           
      // Gravity Matrix Partial Derivative (dG/dq) at theta=0
      // G(q) = [ m1*g*r1*sin(q1) + m2*g*(l1*sin(q1) + r2*sin(q1+q2)) ]
      //        [ m2*g*r2*sin(q1+q2) ]
      // Linearized G (sin(x) -> x):
      // G_lin = [ (m1*g*r1 + m2*g*l1 + m2*g*r2)*q1 + (m2*g*r2)*q2 ]
      //         [ (m2*g*r2)*q1 + (m2*g*r2)*q2 ]
      
      // Actually, be careful with q2 definition.
      // If q2 is relative to q1 (elbow angle):
      // sin(q1+q2) -> q1+q2
      
      double term1 = m1*g*r1 + m2*g*l1 + m2*g*r2;
      double term2 = m2*g*r2;
      
      Eigen::Matrix2d dG_dq;
      dG_dq << term1, term2,
               term2, term2;
               
      // Since equation is M*q_dd + G(q) = tau
      // q_dd = M^-1 * (tau - G(q))
      // A (part) = -M^-1 * dG_dq
      
      Eigen::Matrix2d Minv = M.inverse();
      Eigen::Matrix2d A_gravity = -Minv * dG_dq;
      
      // Damping / Back-EMF
      // Tau_motor = (Kt/R)*(V - Kv*G*w) * G
      // Tau = (G*Kt/R)*V - (G*G*Kt/(R*Kv))*w
      // Let K_torque = G*Kt/R
      // Let K_backemf = G*G*Kt/(R*Kv)
      
      double K_emf1 = (G1 * G1 * motor1.Kt) / (motor1.R * motor1.Kv);
      double K_emf2 = (G2 * G2 * motor2.Kt) / (motor2.R * motor2.Kv);
      
      Eigen::Matrix2d Damping;
      Damping << K_emf1, 0,
                 0, K_emf2;
                 
      Eigen::Matrix2d A_damping = -Minv * Damping;
      
      // B Matrix part
      // Tau = [ K_torque1 * V1 ]
      //       [ K_torque2 * V2 ]
      
      double K_t1 = (G1 * motor1.Kt) / motor1.R;
      double K_t2 = (G2 * motor2.Kt) / motor2.R;
      
      Eigen::Matrix2d B_torque;
      B_torque << K_t1, 0,
                  0, K_t2;
                  
      Eigen::Matrix2d B_sub = Minv * B_torque;
      
      // Construct Full 4x4 A and 4x2 B
      // X = [q1, q2, w1, w2]
      // A = [ 0   I ]
      //     [ Ag  Ad]
      
      StateSpaceSystem sys;
      sys.A = Eigen::MatrixXd::Zero(4, 4);
      sys.A.block<2,2>(0,2) = Eigen::Matrix2d::Identity();
      sys.A.block<2,2>(2,0) = A_gravity;
      sys.A.block<2,2>(2,2) = A_damping;
      
      sys.B = Eigen::MatrixXd::Zero(4, 2);
      sys.B.block<2,2>(2,0) = B_sub;
      
      return sys;
  }
};

} // namespace modeling
} // namespace lib9427
