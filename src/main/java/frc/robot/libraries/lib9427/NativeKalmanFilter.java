package frc.robot.libraries.lib9427;

/**
 * Java wrapper for native Kalman Filter implementation.
 *
 * <p>This class provides a generic discrete-time Kalman Filter for linear systems. The computation
 * is performed in native C++ for efficiency.
 *
 * <p><strong>Kalman Filter Equations</strong>:
 *
 * <pre>
 * Prediction (Time Update):
 *   x̂⁻ = A x̂ + B u        (State prediction)
 *   P⁻  = A P Aᵀ + Q       (Covariance prediction)
 *
 * Correction (Measurement Update):
 *   K = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹   (Kalman gain)
 *   x̂ = x̂⁻ + K (y - H x̂⁻)       (State update)
 *   P = (I - K H) P⁻             (Covariance update)
 * </pre>
 *
 * <p><strong>Memory Management</strong>: The native filter is allocated on construction and freed
 * on {@link #close()}. Use try-with-resources.
 *
 * @see Team9427JNI
 */
public class NativeKalmanFilter implements AutoCloseable {
  /** Handle to native C++ object. */
  private final long handle;

  private final int states;
  private final int inputs;
  private final int outputs;

  private boolean closed = false;

  /**
   * Creates a new Kalman Filter.
   *
   * @param states Number of state variables.
   * @param inputs Number of control inputs.
   * @param outputs Number of observable outputs.
   */
  public NativeKalmanFilter(int states, int inputs, int outputs) {
    this.states = states;
    this.inputs = inputs;
    this.outputs = outputs;
    this.handle = Team9427JNI.createKF(states, inputs, outputs);
  }

  /**
   * Sets the system model matrices.
   *
   * <p>Call this during initialization or when the model changes.
   *
   * @param A State transition matrix (states × states, row-major).
   * @param B Input matrix (states × inputs, row-major).
   * @param Q Process noise covariance (states × states, row-major).
   */
  public void setModel(double[] A, double[] B, double[] Q) {
    if (closed) return;
    Team9427JNI.setKFModel(handle, A, B, Q, states, inputs);
  }

  /**
   * Prediction step (time update).
   *
   * <p>Propagates the state estimate forward: x̂ = A x̂ + B u
   *
   * @param u Control input vector.
   */
  public void predict(double[] u) {
    if (closed) return;
    Team9427JNI.predictKF(handle, u, u.length);
  }

  /**
   * Correction step (measurement update).
   *
   * <p>Updates the state estimate with a measurement: x̂ = x̂ + K(y - Hx̂)
   *
   * @param y Measurement vector.
   * @param H Observation matrix (outputs × states, row-major).
   * @param R Measurement noise covariance (outputs × outputs, row-major).
   */
  public void correct(double[] y, double[] H, double[] R) {
    if (closed) return;
    Team9427JNI.correctKF(handle, y, H, R, y.length, states);
  }

  /**
   * Returns the current state estimate.
   *
   * @return The state vector x̂.
   */
  public double[] getXHat() {
    if (closed) return new double[states];
    return Team9427JNI.getKFXhat(handle, states);
  }

  /**
   * Returns the number of outputs.
   *
   * @return Output dimension.
   */
  public int getOutputs() {
    return outputs;
  }

  @Override
  public void close() {
    if (!closed) {
      Team9427JNI.deleteKF(handle);
      closed = true;
    }
  }
}
