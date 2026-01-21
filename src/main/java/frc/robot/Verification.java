package frc.robot;

import frc.robot.libraries.lib9427.NativeKalmanFilter;
import frc.robot.libraries.lib9427.Team9427JNI;
import java.util.Arrays;

/**
 * System Verification Suite.
 *
 * <p>This class serves as the integration test entry point for the team's custom native control
 * library (Team9427JNI). It validates the mathematical integrity of Linear Quadratic Regulators
 * (LQR), Kalman Filters, and Physics Plant Inversion before deploying code to the robot.
 *
 * <p><strong>Usage:</strong> Run as a standard Java application main class.
 */
public class Verification {

  /**
   * Executes the verification sequence for control theory primitives.
   *
   * @param args Command line arguments (Unused).
   */
  public static void main(String[] args) {
    System.out.println("Running 9427 Strong Control Library Verification (Phase 6)...");
    System.out.println("Java Library Path: " + System.getProperty("java.library.path"));

    /*
     * ------------------------------------------------------------------------
     * Linear Quadratic Regulator (LQR) Verification
     * ------------------------------------------------------------------------
     * Solves the Algebraic Riccati Equation (ARE) to determine optimal feedback gain K.
     *
     * Model: x_dot = Ax + Bu
     * Cost: J = integral(x'Qx + u'Ru)
     */
    System.out.println("1. LQR Verification");
    double[] A = {1.0};
    double[] B = {0.1};
    double[] Q = {1.0};
    double[] R = {0.1};
    int states = 1;
    int inputs = 1;

    double[] K = Team9427JNI.computeLQR(A, B, Q, R, states, inputs);
    System.out.println("Result K: " + Arrays.toString(K));

    /*
     * ------------------------------------------------------------------------
     * Kalman Filter Verification
     * ------------------------------------------------------------------------
     * Validates the instantiation and model setting of the Native Kalman Filter.
     * Ensures JNI memory allocation for the estimator is successful.
     */
    System.out.println("\n2. Kalman Filter Verification");
    try (NativeKalmanFilter kf = new NativeKalmanFilter(1, 1, 1)) {
      double[] A_kf = {1.0};
      double[] B_kf = {0.02};
      double[] Q_kf = {0.001};
      kf.setModel(A_kf, B_kf, Q_kf);
      System.out.println("KF Created successfully.");
    }

    /*
     * ------------------------------------------------------------------------
     * Physics Modeling & Pipeline Integration
     * ------------------------------------------------------------------------
     * Verifies the SystemFactory creation of standard subsystems (Elevator).
     * Validates Continuous-to-Discrete discretization using zero-order hold.
     */
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

    // Verify Discretization: A_d = e^(A * dt)
    var elevatorDiscrete = elevatorSys.discretize(0.02);
    System.out.println("Elevator Discrete A: " + Arrays.toString(elevatorDiscrete.getA()));

    /*
     * ------------------------------------------------------------------------
     * Complex System Modeling (Double Jointed Arm)
     * ------------------------------------------------------------------------
     * Verify the state-space representation of a coupled 2-DOF arm.
     * Checks specific matrix elements (A[2,0]) representing gravity coupling (q1 -> ddq1).
     */
    System.out.println("\n4. Double Joint Arm Verification");
    System.out.println("Checking Double Arm Model...");
    var darm =
        frc.robot.libraries.lib9427.modeling.SystemFactory.createDoubleJointArm(
            bagMotor, bagMotor, 0.5, 0.4, 2.0, 1.5, 1.0, 0.5, 100.0, 80.0);
    double a20 = darm.getA()[2 * 4 + 0];

    // Reference unused variable to suppress warnings or remove if redundant.
    // Preserved here for accessing matrix structure if needed in future debugging.
    // double a21 = darm.getA()[2 * 4 + 1];

    System.out.printf("Double Arm A[2,0] (Gravity q1->ddq1): %.4f%n", a20);

    /*
     * ------------------------------------------------------------------------
     * Feedforward (Plant Inversion) Verification
     * ------------------------------------------------------------------------
     * Verifies the model-based feedforward calculation.
     * Goal: Calculate required voltage 'u' to maintain state 'x' from r_k to r_next.
     *
     * Scenario: Elevator holding constant velocity (1.0 m/s).
     */
    System.out.println("\n5. Feedforward (Plant Inversion) Verification");
    // r_k = [pos, vel] -> [0, 1.0].
    // r_next = [0.02, 1.0] (Position increases by 1.0 * 0.02).
    double[] r_k = {0.0, 1.0};
    double[] r_next = {0.02, 1.0};

    double[] u_ff =
        Team9427JNI.computePlantInversion(
            elevatorDiscrete.getA(), elevatorDiscrete.getB(), r_k, r_next, 2, 1);

    System.out.println("Elevator FF (Hold 1m/s): " + Arrays.toString(u_ff));

    // Validate output plausibility against Back-EMF expectation.
    if (Math.abs(u_ff[0]) > 0.1) {
      System.out.println("Feedforward Calculated Successfully.");
    } else {
      System.err.println("Feedforward WARNING (Value too small?)");
    }
  }
}
