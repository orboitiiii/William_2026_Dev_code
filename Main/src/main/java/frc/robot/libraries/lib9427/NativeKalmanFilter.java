package frc.robot.libraries.lib9427;

public class NativeKalmanFilter implements AutoCloseable {
  private final long handle;
  private final int states;
  private final int inputs;
  private final int outputs;

  private boolean closed = false;

  /** Creates a new Native Kalman Filter. */
  public NativeKalmanFilter(int states, int inputs, int outputs) {
    this.states = states;
    this.inputs = inputs;
    this.outputs = outputs;
    this.handle = Team9427JNI.createKF(states, inputs, outputs);
  }

  /** Sets the system model (A, B, Q). Call this in constructor or when model changes. */
  public void setModel(double[] A, double[] B, double[] Q) {
    if (closed) return;
    Team9427JNI.setKFModel(handle, A, B, Q, states, inputs);
  }

  /** Prediction Step (Time Update). x = Ax + Bu P = APA' + Q */
  public void predict(double[] u) {
    if (closed) return;
    Team9427JNI.predictKF(handle, u, u.length);
  }

  /** Correction Step (Measurement Update). K = PH'(HPH' + R)^-1 x = x + K(y - Hx) P = (I - KH)P */
  public void correct(double[] y, double[] H, double[] R) {
    if (closed) return;
    Team9427JNI.correctKF(handle, y, H, R, y.length, states);
  }

  /** Returns the current estimated state vector x_hat. */
  public double[] getXHat() {
    if (closed) return new double[states];
    return Team9427JNI.getKFXhat(handle, states);
  }

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
