package frc.robot.libraries.lib9427;

import java.io.File;

/**
 * JNI bridge to native C++ control library.
 *
 * <p>This class provides native method declarations for high-performance control algorithms
 * implemented in C++. The native library handles:
 *
 * <ul>
 *   <li>LQR gain computation (solving DARE)
 *   <li>Kalman Filter state estimation
 *   <li>System discretization (matrix exponential)
 *   <li>ES-UKF sensor fusion
 *   <li>Delta-U disturbance observer control
 * </ul>
 *
 * <p><strong>Library Loading</strong>: The native library is loaded from the system library path or
 * a fallback location for development.
 *
 * <p><strong>Array Packing Convention</strong>: All matrices are passed as 1D arrays in row-major
 * order. For a matrix M with r rows and c columns:
 *
 * <pre>
 * M[i][j] = array[i * c + j]
 * </pre>
 *
 * <p><strong>Memory Management</strong>: Functions returning handles (createKF, createESUKF, etc.)
 * allocate native memory that must be freed by calling the corresponding delete function.
 */
public class Team9427JNI {
  static {
    try {
      System.loadLibrary("Team9427JNI");
    } catch (UnsatisfiedLinkError e) {
      try {
        String osName = System.getProperty("os.name");
        // Fallback for development on Windows
        if (osName.startsWith("Windows")) {
          File dll =
              new File("build/libs/team9427JNI/shared/windowsx86-64/release/Team9427JNI.dll");
          if (!dll.exists()) {
            dll = new File("build/libs/team9427JNI/shared/windowsx86-64/debug/Team9427JNI.dll");
          }
          if (dll.exists()) {
            System.load(dll.getAbsolutePath());
          } else {
            System.err.println("CRITICAL: JNI DLL not found in release or debug paths.");
            throw e;
          }
        } else {
          throw e;
        }
      } catch (Throwable t) {
        t.printStackTrace();
        throw new RuntimeException(
            "Failed to load Team9427JNI library. java.library.path="
                + System.getProperty("java.library.path"),
            t);
      }
    }
  }

  // --- LQR Controller ---

  /**
   * Computes the LQR gain matrix by solving the Discrete Algebraic Riccati Equation (DARE).
   *
   * <p><strong>LQR Problem</strong>: Find K that minimizes J = Σ (xᵀQx + uᵀRu)
   *
   * @param A Discrete state matrix (states ? states).
   * @param B Discrete input matrix (states ? inputs).
   * @param Q State cost matrix (states ? states).
   * @param R Input cost matrix (inputs ? inputs).
   * @param states Number of states.
   * @param inputs Number of inputs.
   * @return Gain matrix K (inputs ? states).
   */
  public static native double[] computeLQR(
      double[] A, double[] B, double[] Q, double[] R, int states, int inputs);

  /**
   * Computes feedforward using plant inversion.
   *
   * <p><strong>Formula</strong>: u_ff = B??(r_{next} - A * r_k)
   *
   * @param A Discrete state matrix.
   * @param B Discrete input matrix.
   * @param r_k Current reference state.
   * @param r_next Next reference state.
   * @param states Number of states.
   * @param inputs Number of inputs.
   * @return Feedforward input vector.
   */
  public static native double[] computePlantInversion(
      double[] A, double[] B, double[] r_k, double[] r_next, int states, int inputs);

  // --- Kalman Filter ---

  /**
   * Creates a native Kalman Filter instance.
   *
   * @param states Number of states.
   * @param inputs Number of inputs.
   * @param outputs Number of outputs.
   * @return Native handle (pointer).
   */
  public static native long createKF(int states, int inputs, int outputs);

  /**
   * Deletes a native Kalman Filter instance.
   *
   * @param handle Native handle from createKF.
   */
  public static native void deleteKF(long handle);

  /**
   * Sets the Kalman Filter system model.
   *
   * @param handle Native handle.
   * @param A State transition matrix.
   * @param B Input matrix.
   * @param Q Process noise covariance.
   * @param states Number of states.
   * @param inputs Number of inputs.
   */
  public static native void setKFModel(
      long handle, double[] A, double[] B, double[] Q, int states, int inputs);

  /**
   * Performs Kalman Filter prediction step.
   *
   * @param handle Native handle.
   * @param u Control input vector.
   * @param size_u Size of input vector.
   */
  public static native void predictKF(long handle, double[] u, int size_u);

  /**
   * Performs Kalman Filter correction step.
   *
   * @param handle Native handle.
   * @param y Measurement vector.
   * @param H Observation matrix.
   * @param R Measurement noise covariance.
   * @param rows_y Number of measurements.
   * @param cols_H Number of states.
   */
  public static native void correctKF(
      long handle, double[] y, double[] H, double[] R, int rows_y, int cols_H);

  /**
   * Gets the current state estimate.
   *
   * @param handle Native handle.
   * @param states Number of states.
   * @return State estimate vector.
   */
  public static native double[] getKFXhat(long handle, int states);

  // --- System Modeling & Discretization ---

  /**
   * Discretizes continuous-time matrices A and B using matrix exponential.
   *
   * @param A Continuous state matrix.
   * @param B Continuous input matrix.
   * @param states Number of states.
   * @param inputs Number of inputs.
   * @param dt Time step.
   * @return Concatenated [Ad, Bd].
   */
  public static native double[] discretizeAB(
      double[] A, double[] B, int states, int inputs, double dt);

  /**
   * Creates an elevator linear system model.
   *
   * @param Kt Motor torque constant (N?m/A).
   * @param Kv Motor velocity constant (rad/s/V).
   * @param R Motor resistance (Ω).
   * @param m Mass (kg).
   * @param r Drum radius (m).
   * @param G Gear ratio.
   * @return Concatenated [A, B] matrices.
   */
  public static native double[] createElevator(
      double Kt, double Kv, double R, double m, double r, double G);

  /**
   * Creates a flywheel linear system model.
   *
   * @param Kt Motor torque constant.
   * @param Kv Motor velocity constant.
   * @param R Motor resistance.
   * @param J Moment of inertia (kg?m²).
   * @param G Gear ratio.
   * @return Concatenated [A, B].
   */
  public static native double[] createFlywheel(double Kt, double Kv, double R, double J, double G);

  /**
   * Creates a single-joint arm linear system model.
   *
   * @param Kt Motor torque constant.
   * @param Kv Motor velocity constant.
   * @param R Motor resistance.
   * @param J Moment of inertia.
   * @param G Gear ratio.
   * @return Concatenated [A, B].
   */
  public static native double[] createSingleArm(double Kt, double Kv, double R, double J, double G);

  /**
   * Creates a differential drivetrain velocity model.
   *
   * @param Kt Motor torque constant.
   * @param Kv Motor velocity constant.
   * @param R Motor resistance.
   * @param m Robot mass.
   * @param r Wheel radius.
   * @param rb Trackwidth / 2.
   * @param J Rotational inertia.
   * @param G Gear ratio.
   * @return Concatenated [A, B].
   */
  public static native double[] createDrivetrain(
      double Kt, double Kv, double R, double m, double r, double rb, double J, double G);

  /**
   * Creates a double-jointed arm linear system model.
   *
   * @param Kt1 Joint 1 motor torque constant.
   * @param Kv1 Joint 1 motor velocity constant.
   * @param R1 Joint 1 motor resistance.
   * @param Kt2 Joint 2 motor torque constant.
   * @param Kv2 Joint 2 motor velocity constant.
   * @param R2 Joint 2 motor resistance.
   * @param l1 Link 1 length.
   * @param l2 Link 2 length.
   * @param m1 Link 1 mass.
   * @param m2 Link 2 mass.
   * @param I1 Link 1 inertia.
   * @param I2 Link 2 inertia.
   * @param G1 Joint 1 gear ratio.
   * @param G2 Joint 2 gear ratio.
   * @return Concatenated [A, B].
   */
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

  // --- Delta-U Controller ---

  /**
   * Creates a native Delta-U controller.
   *
   * @param A Discrete state matrix.
   * @param B Discrete input matrix.
   * @param C Output matrix.
   * @param Q_nom State cost for LQR.
   * @param R_nom Input cost for LQR.
   * @param Q_state State process noise.
   * @param Q_distrubance Disturbance process noise.
   * @param R_meas Measurement noise.
   * @param states Number of states.
   * @param inputs Number of inputs.
   * @param outputs Number of outputs.
   * @return Native handle.
   */
  public static native long createDeltaU(
      double[] A,
      double[] B,
      double[] C,
      double[] Q_nom,
      double[] R_nom,
      double[] Q_state,
      double[] Q_distrubance,
      double[] R_meas,
      int states,
      int inputs,
      int outputs);

  /**
   * Deletes a native Delta-U controller.
   *
   * @param handle Native handle.
   */
  public static native void deleteDeltaU(long handle);

  /**
   * Updates the Delta-U controller and returns the control command.
   *
   * @param handle Native handle.
   * @param y Current measurement.
   * @param r Reference state.
   * @param u_ff Feedforward input.
   * @param u_prev Previous applied input (for anti-windup).
   * @param states Number of states.
   * @param inputs Number of inputs.
   * @param outputs Number of outputs.
   * @return Computed control input.
   */
  public static native double[] updateDeltaU(
      long handle,
      double[] y,
      double[] r,
      double[] u_ff,
      double[] u_prev,
      int states,
      int inputs,
      int outputs);

  /**
   * Resets the Delta-U controller state.
   *
   * @param handle Native handle.
   * @param x0 Initial state.
   * @param states Number of states.
   */
  public static native void resetDeltaU(long handle, double[] x0, int states);

  /**
   * Resets only the state estimate (not disturbance).
   *
   * @param handle Native handle.
   * @param x0 Initial state.
   * @param states Number of states.
   */
  public static native void resetDeltaUStateOnly(long handle, double[] x0, int states);

  /**
   * Gets the estimated state from the Delta-U observer.
   *
   * @param handle Native handle.
   * @param states Number of states.
   * @return State estimate.
   */
  public static native double[] getDeltaUState(long handle, int states);

  /**
   * Gets the estimated disturbance from the Delta-U observer.
   *
   * @param handle Native handle.
   * @param inputs Number of inputs.
   * @return Disturbance estimate.
   */
  public static native double[] getDeltaUDisturbance(long handle, int inputs);

  // --- Ballistic Solver ---

  /**
   * Creates a native BallisticSolver instance.
   *
   * @return Native handle.
   */
  public static native long createBallisticSolver();

  /**
   * Deletes a native BallisticSolver instance.
   *
   * @param handle Native handle.
   */
  public static native void deleteBallisticSolver(long handle);

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
  public static native void configureBallisticSolver(
      long handle,
      double mass,
      double diameter,
      double cd,
      double cm,
      double moi,
      double spinDecay);

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
  public static native void setBallisticShooterConstraints(
      long handle,
      double minRpm,
      double maxRpm,
      double wheelRadius,
      double pitchMin,
      double pitchMax,
      double turretMaxRate);

  /**
   * Sets target specification.
   *
   * @param handle Native handle.
   * @param centerX Target center X [m].
   * @param centerY Target center Y [m].
   * @param centerZ Target center height [m].
   */
  public static native void setBallisticTarget(
      long handle, double centerX, double centerY, double centerZ);

  /**
   * Sets integration time step.
   *
   * @param handle Native handle.
   * @param dt Time step [s]. Must be <= 0.01s.
   */
  public static native void setBallisticTimeStep(long handle, double dt);

  /**
   * Sets projectile spin rate.
   *
   * @param handle Native handle.
   * @param spinRpm Backspin rate [RPM].
   */
  public static native void setBallisticSpinRate(long handle, double spinRpm);

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
   * @param turretOffsetY Turret Y offset from robot center [m].
   * @param results Output buffer (at least 11 elements).
   */
  public static native void solveBallisticSolution(
      long handle,
      double robotX,
      double robotY,
      double robotVx,
      double robotVy,
      double robotOmega,
      double robotHeading,
      double turretHeight,
      double turretYaw,
      double turretYawRate,
      double turretOffsetX,
      double turretOffsetY,
      double[] results);

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
   * @param maxPoints Maximum trajectory points.
   * @return Flattened trajectory [t, x, y, z, vx, vy, vz, ...]. Each point is 7 values.
   */
  public static native double[] simulateBallisticTrajectory(
      long handle,
      double yaw,
      double pitch,
      double velocity,
      double robotX,
      double robotY,
      double robotVx,
      double robotVy,
      double robotOmega,
      double robotHeading,
      double turretHeight,
      double turretOffsetX,
      double turretOffsetY,
      int maxPoints);

  /**
   * Checks if a point is inside the hexagon target.
   *
   * @param xInch X coordinate relative to hexagon center [inches].
   * @param yInch Y coordinate relative to hexagon center [inches].
   * @return True if inside hexagon.
   */
  public static native boolean isInsideHexagon(double xInch, double yInch);
}
