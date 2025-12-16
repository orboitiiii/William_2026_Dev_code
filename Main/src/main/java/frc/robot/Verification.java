package frc.robot;

import java.util.Arrays;

import frc.robot.libraries.lib9427.NativeKalmanFilter;
import frc.robot.libraries.lib9427.Team9427JNI;

public class Verification {
  public static void main(String[] args) {
    System.out.println("Running 9427 Strong Control Library Verification (Phase 6)...");
    System.out.println("Java Library Path: " + System.getProperty("java.library.path"));

    // --- PART 1: LQR Verification ---
    System.out.println("1. LQR Verification");
    double[] A = {1.0};
    double[] B = {0.1};
    double[] Q = {1.0};
    double[] R = {0.1};
    int states = 1;
    int inputs = 1;

    double[] K = Team9427JNI.computeLQR(A, B, Q, R, states, inputs);
    System.out.println("Result K: " + Arrays.toString(K));

    // --- PART 2: Kalman Filter Verification ---
    System.out.println("\n2. Kalman Filter Verification");
    try (NativeKalmanFilter kf = new NativeKalmanFilter(1, 1, 1)) {
      double[] A_kf = {1.0};
      double[] B_kf = {0.02};
      double[] Q_kf = {0.001};
      kf.setModel(A_kf, B_kf, Q_kf);
      System.out.println("KF Created successfully.");
    }

    // --- PART 3: Modeling & Full Pipeline ---
    System.out.println("\n3. Modeling & Pipeline Verification");
    edu.wpi.first.math.system.plant.DCMotor bagMotor =
        edu.wpi.first.math.system.plant.DCMotor.getBag(2);
    double mass = 5.0;
    double radius = 0.05;
    double gearing = 10.0;

    var elevatorSys =
        frc.robot.libraries.lib9427.modeling.SystemFactory.createElevator(
            bagMotor, mass, radius, gearing);
    System.out.println("Elevator Continuous A: " + Arrays.toString(elevatorSys.getA()));

    // Discretize
    var elevatorDiscrete = elevatorSys.discretize(0.02);
    System.out.println("Elevator Discrete A: " + Arrays.toString(elevatorDiscrete.getA()));

    // --- PART 4: Double Joint Arm ---
    System.out.println("\n4. Double Joint Arm Verification");
    System.out.println("Checking Double Arm Model...");
    var darm =
        frc.robot.libraries.lib9427.modeling.SystemFactory.createDoubleJointArm(
            bagMotor, bagMotor, 0.5, 0.4, 2.0, 1.5, 1.0, 0.5, 100.0, 80.0);
    double a20 = darm.getA()[2 * 4 + 0];
    double a21 = darm.getA()[2 * 4 + 1];

    System.out.printf("Double Arm A[2,0] (Gravity q1->ddq1): %.4f%n", a20);

    // --- PART 5: Feedforward Verification (Phase 6) ---
    System.out.println("\n5. Feedforward (Plant Inversion) Verification");
    // Elevator Discrete Model
    // Maintain 1.0 m/s velocity. Constant velocity.
    // x = [pos, vel]
    // r_k = [0, 1.0]. Next step should be [0.02, 1.0].
    double[] r_k = {0.0, 1.0};
    double[] r_next = {0.02, 1.0};

    double[] u_ff =
        Team9427JNI.computePlantInversion(
            elevatorDiscrete.getA(), elevatorDiscrete.getB(), r_k, r_next, 2, 1);

    System.out.println("Elevator FF (Hold 1m/s): " + Arrays.toString(u_ff));
    // Check reasonable voltage (e.g., Back EMF + Friction compensation)
    // Back EMF roughly: Kv_rad_s_v * ...
    if (Math.abs(u_ff[0]) > 0.1) {
      System.out.println("Feedforward Calculated Successfully.");
    } else {
      System.err.println("Feedforward WARNING (Value too small?)");
    }
  }
}
