package frc.robot.libraries.lib9427.modeling;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.libraries.lib9427.Team9427JNI;

public class SystemFactory {

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
    // int sizeA = 4; // 2x2
    // int sizeB = 4; // 2x2
    double[] A = new double[4];
    double[] B = new double[4];
    System.arraycopy(AB, 0, A, 0, 4);
    System.arraycopy(AB, 4, B, 0, 4);

    double[] C = {1.0, 0.0, 0.0, 1.0};
    double[] D = {0.0, 0.0, 0.0, 0.0};
    return new LinearSystem(A, B, C, D, 2, 2, 2);
  }

  public static LinearSystem createDoubleJointArm(
      DCMotor motor1,
      DCMotor motor2,
      double l1,
      double l2, // Lengths
      double m1,
      double m2, // Masses
      double j1,
      double j2, // MOIs (about pivot)
      double g1,
      double g2) { // Gearings

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
    // A=4x4=16, B=4x2=8
    double[] A = new double[16];
    double[] B = new double[8];
    System.arraycopy(AB, 0, A, 0, 16);
    System.arraycopy(AB, 16, B, 0, 8);

    double[] C = {1, 0, 0, 0, 0, 0, 1, 0}; // Output angles
    double[] D = new double[4]; // 2x2

    return new LinearSystem(A, B, C, D, 4, 2, 2);
  }
}
