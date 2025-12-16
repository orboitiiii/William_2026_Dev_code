package frc.robot.libraries.lib9427.modeling;

import frc.robot.libraries.lib9427.Team9427JNI;

/** Represents a State-Space System (Continuous or Discrete). */
public class LinearSystem {
  private final double[] A;
  private final double[] B;
  private final double[] C;
  private final double[] D;

  public final int states;
  public final int inputs;
  public final int outputs;

  public LinearSystem(
      double[] A, double[] B, double[] C, double[] D, int states, int inputs, int outputs) {
    this.A = A;
    this.B = B;
    this.C = C;
    this.D = D;
    this.states = states;
    this.inputs = inputs;
    this.outputs = outputs;
  }

  public double[] getA() {
    return A;
  }

  public double[] getB() {
    return B;
  }

  public double[] getC() {
    return C;
  }

  public double[] getD() {
    return D;
  }

  /**
   * Discretizes the system using simple Euler or Taylor series (Matrix Exponential). For high
   * fidelity, we'd wrap a JNI call to DiscretizeAB, but for now simple Euler: Ad = I + A * dt Bd =
   * B * dt
   *
   * <p>Actually, let's use a standard approximation sufficient for FRC (First Order Hold or Euler).
   * Accuracy: Euler (Ad = I + A*dt) is okay for 20ms if dynamics aren't too fast.
   *
   * <p>Note: WPILib's LinearSystemId returns Continuous. We need Discrete for DARE.
   */
  public LinearSystem discretize(double dt) {
    // High-Precision Discretization using C++ Matrix Exponential
    double[] AB_d = Team9427JNI.discretizeAB(this.A, this.B, states, inputs, dt);

    int sizeA = states * states;
    int sizeB = states * inputs;

    double[] Ad = new double[sizeA];
    double[] Bd = new double[sizeB];

    System.arraycopy(AB_d, 0, Ad, 0, sizeA);
    System.arraycopy(AB_d, sizeA, Bd, 0, sizeB);

    return new LinearSystem(Ad, Bd, this.C, this.D, states, inputs, outputs);
  }
}
