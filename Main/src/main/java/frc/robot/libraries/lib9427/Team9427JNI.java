package frc.robot.libraries.lib9427;

import java.io.File;

public class Team9427JNI {
  static {
    try {
      System.loadLibrary("Team9427JNI");
    } catch (UnsatisfiedLinkError e) {
      try {
        // Fallback for local testing (hardcoded path relative to project root)
        // Assuming run from project root
        File dll = new File("build/libs/Team9427JNI/shared/release/Team9427JNI.dll");
        // DEBUG: Print path
        System.out.println("Debugging JNI Load: Check Path: " + dll.getAbsolutePath());
        if (dll.exists()) {
          System.out.println("Debugging JNI Load: File Found. Loading...");
          System.load(dll.getAbsolutePath());
        } else {
          System.err.println("Debugging JNI Load: File NOT FOUND at " + dll.getAbsolutePath());
          throw e; // Rethrow original if fallback not found
        }
      } catch (Throwable t) {
        t.printStackTrace();
        throw new RuntimeException("Failed to load Team9427JNI library", t);
      }
    }
  }

  // --- Controllers ---
  public static native double[] computeLQR(
      double[] A, double[] B, double[] Q, double[] R, int states, int inputs);

  public static native double[] computePlantInversion(
      double[] A, double[] B, double[] r_k, double[] r_next, int states, int inputs);

  // --- Kalman Filter ---
  public static native long createKF(int states, int inputs, int outputs);

  public static native void deleteKF(long handle);

  public static native void setKFModel(
      long handle, double[] A, double[] B, double[] Q, int states, int inputs);

  public static native void predictKF(long handle, double[] u, int size_u);

  public static native void correctKF(
      long handle, double[] y, double[] H, double[] R, int rows_y, int cols_H);

  public static native double[] getKFXhat(long handle, int states);

  // --- Modeling & Discretization (Phase 4) ---
  // Returns [Ad, Bd] concatenated
  public static native double[] discretizeAB(
      double[] A, double[] B, int states, int inputs, double dt);

  // Returns [A, B] concatenated
  public static native double[] createElevator(
      double Kt, double Kv, double R, double m, double r, double G);

  public static native double[] createFlywheel(double Kt, double Kv, double R, double J, double G);

  public static native double[] createSingleArm(double Kt, double Kv, double R, double J, double G);

  public static native double[] createDrivetrain(
      double Kt, double Kv, double R, double m, double r, double rb, double J, double G);

  public static native double[] createDoubleArm(
      double Kt1,
      double Kv1,
      double R1,
      double Kt2,
      double Kv2,
      double R2,
      double l1,
      double l2,
      double m1,
      double m2,
      double I1,
      double I2,
      double G1,
      double G2);

  // --- ES-UKF ---
  public static native long createESUKF();

  public static native void deleteESUKF(long handle);

  public static native void initESUKF(
      long handle, double px, double py, double pz, double qw, double qx, double qy, double qz);

  public static native void predictESUKF(
      long handle, double ax, double ay, double az, double gx, double gy, double gz, double dt);

  public static native boolean correctESUKF(
      long handle, double px, double py, double pz, double[] R, double gateThreshold);

  public static native double[] getESUKFState(long handle);
}
