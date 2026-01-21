package frc.robot.libraries.lib9427.controllers;

import frc.robot.libraries.lib9427.Team9427JNI;

/**
 * Java wrapper for native Delta-U Controller with disturbance observer.
 *
 * <p>The Delta-U controller is an advanced LQR variant that explicitly estimates and compensates
 * for unmodeled disturbances (friction, payload changes, etc.).
 *
 * <p><strong>Algorithm Overview</strong>:
 *
 * <ol>
 *   <li>Standard LQR computes optimal feedback gain K
 *   <li>An augmented observer estimates both state x̂ and disturbance d̂
 *   <li>Control law: u = -K(x̂ - r) + u_ff - d̂
 * </ol>
 *
 * <p><strong>Benefits over Standard LQR</strong>:
 *
 * <ul>
 *   <li>Implicit integral action eliminates steady-state error
 *   <li>Disturbance estimation provides feed-forward compensation
 *   <li>More robust to model uncertainty
 * </ul>
 *
 * <p><strong>References</strong>:
 *
 * <ul>
 *   <li>Wang, L., "Model Predictive Control System Design and Implementation Using MATLAB"
 * </ul>
 *
 * @see Team9427JNI
 */
public class NativeDeltaUController implements AutoCloseable {
  /** Handle to native C++ object. */
  private long handle;

  private final int states;
  private final int inputs;
  private final int outputs;

  /**
   * Constructs the Delta-U controller by calling native initialization.
   *
   * @param A Discrete state matrix (states × states).
   * @param B Discrete input matrix (states × inputs).
   * @param C Output matrix (outputs × states).
   * @param Q_nom State cost for LQR gain computation.
   * @param R_nom Input cost for LQR gain computation.
   * @param Q_state Process noise for state estimation.
   * @param Q_dist Process noise for disturbance estimation.
   * @param R_meas Measurement noise covariance.
   * @param states Number of states.
   * @param inputs Number of inputs.
   * @param outputs Number of outputs.
   */
  public NativeDeltaUController(
      double[] A,
      double[] B,
      double[] C,
      double[] Q_nom,
      double[] R_nom,
      double[] Q_state,
      double[] Q_dist,
      double[] R_meas,
      int states,
      int inputs,
      int outputs) {

    this.states = states;
    this.inputs = inputs;
    this.outputs = outputs;

    handle =
        Team9427JNI.createDeltaU(
            A, B, C, Q_nom, R_nom, Q_state, Q_dist, R_meas, states, inputs, outputs);
  }

  @Override
  public void close() {
    if (handle != 0) {
      Team9427JNI.deleteDeltaU(handle);
      handle = 0;
    }
  }

  /**
   * Computes the control command for the current timestep.
   *
   * @param y Current measurement vector.
   * @param r Reference state vector.
   * @param u_ff Feedforward input (from trajectory).
   * @param u_prev Previously applied input (for saturation handling).
   * @return Computed control input.
   */
  public double[] update(double[] y, double[] r, double[] u_ff, double[] u_prev) {
    return Team9427JNI.updateDeltaU(handle, y, r, u_ff, u_prev, states, inputs, outputs);
  }

  /**
   * Resets the controller state and disturbance estimate.
   *
   * @param x0 Initial state estimate.
   */
  public void reset(double[] x0) {
    Team9427JNI.resetDeltaU(handle, x0, states);
  }

  /**
   * Resets only the state estimate (preserves disturbance).
   *
   * @param x0 Initial state estimate.
   */
  public void resetStateOnly(double[] x0) {
    Team9427JNI.resetDeltaUStateOnly(handle, x0, states);
  }

  /**
   * Returns the current state estimate from the observer.
   *
   * @return Estimated state vector.
   */
  public double[] getEstimatedState() {
    return Team9427JNI.getDeltaUState(handle, states);
  }

  /**
   * Returns the current disturbance estimate.
   *
   * @return Estimated disturbance vector.
   */
  public double[] getEstimatedDisturbance() {
    return Team9427JNI.getDeltaUDisturbance(handle, inputs);
  }
}
