package frc.robot.libraries.lib9427.modeling;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.libraries.lib9427.Team9427JNI;

/**
 * Factory for creating common FRC mechanism state-space models.
 *
 * <p>Each factory method generates a continuous-time {@link LinearSystem} that models the
 * mechanism's dynamics from first principles (motor physics, masses, inertias, gear ratios).
 *
 * <p><strong>DC Motor Model</strong> (basis for all factories):
 *
 * <pre>
 * Electrical: V = I * R + Kv * ω
 * Mechanical: τ = Kt * I = J * α + τ_friction
 *
 * where:
 *   Kt = motor torque constant (N⋅m/A)
 *   Kv = back-EMF constant (V/(rad/s))
 *   R  = winding resistance (Ω)
 *   J  = moment of inertia (kg⋅m²)
 * </pre>
 *
 * <p><strong>Usage</strong>:
 *
 * <pre>{@code
 * LinearSystem elevator = SystemFactory.createElevator(
 *     DCMotor.getNEO(2), 10.0, 0.02, 10.0);
 * }</pre>
 *
 * @see LinearSystem
 * @see DCMotor
 */
public class SystemFactory {

  /**
   * Creates a flywheel (single rotating mass) system model.
   *
   * <p><strong>State</strong>: [angular velocity (rad/s)]
   *
   * <p><strong>Input</strong>: [voltage (V)]
   *
   * <p><strong>Output</strong>: [angular velocity]
   *
   * @param motor The motor driving the flywheel.
   * @param jKgMetersSquared Moment of inertia (kg⋅m²).
   * @param gearing Gear ratio (motor rotations per flywheel rotation).
   * @return Continuous-time linear system.
   */
  public static LinearSystem createFlywheel(
      DCMotor motor, double jKgMetersSquared, double gearing) {

    double[] AB =
        Team9427JNI.createFlywheel(
            motor.KtNMPerAmp, motor.KvRadPerSecPerVolt, motor.rOhms, jKgMetersSquared, gearing);

    // 1 State, 1 Input
    double[] A = {AB[0]};
    double[] B = {AB[1]};
    double[] C = {1.0};
    double[] D = {0.0};
    return new LinearSystem(A, B, C, D, 1, 1, 1);
  }

  /**
   * Creates an elevator (linear motion) system model.
   *
   * <p><strong>States</strong>: [position (m), velocity (m/s)]
   *
   * <p><strong>Input</strong>: [voltage (V)]
   *
   * <p><strong>Output</strong>: [position]
   *
   * @param motor The motor driving the elevator.
   * @param mKg Carriage mass (kg).
   * @param rMeters Drum/spool radius (m).
   * @param gearing Gear ratio.
   * @return Continuous-time linear system.
   */
  public static LinearSystem createElevator(
      DCMotor motor, double mKg, double rMeters, double gearing) {

    double[] AB =
        Team9427JNI.createElevator(
            motor.KtNMPerAmp, motor.KvRadPerSecPerVolt, motor.rOhms, mKg, rMeters, gearing);

    // 2 States, 1 Input
    double[] A = new double[4];
    double[] B = new double[2];
    System.arraycopy(AB, 0, A, 0, 4);
    System.arraycopy(AB, 4, B, 0, 2);

    double[] C = {1.0, 0.0};
    double[] D = {0.0};
    return new LinearSystem(A, B, C, D, 2, 1, 1);
  }

  /**
   * Creates a single-joint arm system model.
   *
   * <p><strong>States</strong>: [angle (rad), angular velocity (rad/s)]
   *
   * <p><strong>Input</strong>: [voltage (V)]
   *
   * <p><strong>Output</strong>: [angle]
   *
   * <p><strong>Note</strong>: This is a linearized model that does not include gravity
   * compensation. For pivoting arms, add a feedforward term for gravity.
   *
   * @param motor The motor driving the arm.
   * @param jKgMetersSquared Moment of inertia about the pivot (kg⋅m²).
   * @param gearing Gear ratio.
   * @return Continuous-time linear system.
   */
  public static LinearSystem createSingleJointArm(
      DCMotor motor, double jKgMetersSquared, double gearing) {

    double[] AB =
        Team9427JNI.createSingleArm(
            motor.KtNMPerAmp, motor.KvRadPerSecPerVolt, motor.rOhms, jKgMetersSquared, gearing);

    // 2 States, 1 Input
    double[] A = new double[4];
    double[] B = new double[2];
    System.arraycopy(AB, 0, A, 0, 4);
    System.arraycopy(AB, 4, B, 0, 2);

    double[] C = {1.0, 0.0};
    double[] D = {0.0};
    return new LinearSystem(A, B, C, D, 2, 1, 1);
  }

  /**
   * Creates a differential drivetrain velocity model.
   *
   * <p><strong>States</strong>: [left velocity (m/s), right velocity (m/s)]
   *
   * <p><strong>Inputs</strong>: [left voltage, right voltage]
   *
   * <p><strong>Outputs</strong>: [left velocity, right velocity]
   *
   * @param motor The motors on each side (combined).
   * @param mKg Robot mass (kg).
   * @param rMeters Wheel radius (m).
   * @param rbMeters Half of trackwidth (m).
   * @param JKgMetersSquared Rotational inertia (kg⋅m²).
   * @param gearing Gear ratio.
   * @return Continuous-time linear system.
   */
  public static LinearSystem createDrivetrainVelocity(
      DCMotor motor,
      double mKg,
      double rMeters,
      double rbMeters,
      double JKgMetersSquared,
      double gearing) {

    double[] AB =
        Team9427JNI.createDrivetrain(
            motor.KtNMPerAmp,
            motor.KvRadPerSecPerVolt,
            motor.rOhms,
            mKg,
            rMeters,
            rbMeters,
            JKgMetersSquared,
            gearing);

    // 2 States, 2 Inputs
    double[] A = new double[4];
    double[] B = new double[4];
    System.arraycopy(AB, 0, A, 0, 4);
    System.arraycopy(AB, 4, B, 0, 4);

    double[] C = {1.0, 0.0, 0.0, 1.0};
    double[] D = {0.0, 0.0, 0.0, 0.0};
    return new LinearSystem(A, B, C, D, 2, 2, 2);
  }

  /**
   * Creates a double-jointed arm system model.
   *
   * <p><strong>States</strong>: [θ₁, ω₁, θ₂, ω₂]
   *
   * <p><strong>Inputs</strong>: [voltage₁, voltage₂]
   *
   * <p><strong>Outputs</strong>: [θ₁, θ₂]
   *
   * <p><strong>Note</strong>: This is a linearized model around the specified operating point. The
   * dynamics are highly nonlinear.
   *
   * @param motor1 Joint 1 motor.
   * @param motor2 Joint 2 motor.
   * @param l1 Link 1 length (m).
   * @param l2 Link 2 length (m).
   * @param m1 Link 1 mass (kg).
   * @param m2 Link 2 mass (kg).
   * @param j1 Link 1 inertia about pivot (kg⋅m²).
   * @param j2 Link 2 inertia about pivot (kg⋅m²).
   * @param g1 Joint 1 gear ratio.
   * @param g2 Joint 2 gear ratio.
   * @return Continuous-time linear system.
   */
  public static LinearSystem createDoubleJointArm(
      DCMotor motor1,
      DCMotor motor2,
      double l1,
      double l2,
      double m1,
      double m2,
      double j1,
      double j2,
      double g1,
      double g2) {

    double[] AB =
        Team9427JNI.createDoubleArm(
            motor1.KtNMPerAmp,
            motor1.KvRadPerSecPerVolt,
            motor1.rOhms,
            motor2.KtNMPerAmp,
            motor2.KvRadPerSecPerVolt,
            motor2.rOhms,
            l1,
            l2,
            m1,
            m2,
            j1,
            j2,
            g1,
            g2);

    // 4 States, 2 Inputs
    double[] A = new double[16];
    double[] B = new double[8];
    System.arraycopy(AB, 0, A, 0, 16);
    System.arraycopy(AB, 16, B, 0, 8);

    double[] C = {1, 0, 0, 0, 0, 0, 1, 0}; // Output: angles
    double[] D = new double[4]; // 2x2 zeros

    return new LinearSystem(A, B, C, D, 4, 2, 2);
  }
}
