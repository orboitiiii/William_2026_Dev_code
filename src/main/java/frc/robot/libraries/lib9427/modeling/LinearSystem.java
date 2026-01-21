package frc.robot.libraries.lib9427.modeling;

import frc.robot.libraries.lib9427.Team9427JNI;

/**
 * State-space representation of a linear time-invariant (LTI) system.
 *
 * <p>A state-space model describes system dynamics in the form:
 *
 * <pre>
 * Continuous:
 *   ẋ = A x + B u    (State equation)
 *   y = C x + D u    (Output equation)
 *
 * Discrete:
 *   x[k+1] = A_d x[k] + B_d u[k]
 *   y[k]   = C x[k] + D u[k]
 * </pre>
 *
 * <p><strong>Usage</strong>:
 *
 * <pre>{@code
 * LinearSystem elevator = SystemFactory.createElevator(motor, mass, radius, gearing);
 * LinearSystem discrete = elevator.discretize(0.02);
 * }</pre>
 *
 * <p><strong>Array Packing</strong>: All matrices are stored in row-major 1D arrays.
 *
 * @see SystemFactory
 */
public class LinearSystem {
  private final double[] A;
  private final double[] B;
  private final double[] C;
  private final double[] D;

  /** Number of state variables. */
  public final int states;

  /** Number of control inputs. */
  public final int inputs;

  /** Number of observable outputs. */
  public final int outputs;

  /**
   * Constructs a linear system with the given matrices.
   *
   * @param A State matrix (states × states).
   * @param B Input matrix (states × inputs).
   * @param C Output matrix (outputs × states).
   * @param D Feedthrough matrix (outputs × inputs).
   * @param states Number of states.
   * @param inputs Number of inputs.
   * @param outputs Number of outputs.
   */
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

  /**
   * Returns the state transition matrix A.
   *
   * @return A matrix (row-major).
   */
  public double[] getA() {
    return A;
  }

  /**
   * Returns the input matrix B.
   *
   * @return B matrix (row-major).
   */
  public double[] getB() {
    return B;
  }

  /**
   * Returns the output matrix C.
   *
   * @return C matrix (row-major).
   */
  public double[] getC() {
    return C;
  }

  /**
   * Returns the feedthrough matrix D.
   *
   * @return D matrix (row-major).
   */
  public double[] getD() {
    return D;
  }

  /**
   * Discretizes the continuous system using matrix exponential (ZOH).
   *
   * <p><strong>Method</strong>: Zero-Order Hold (ZOH) discretization.
   *
   * <pre>
   * A_d = e^(A * dt)
   * B_d = A^(-1) * (A_d - I) * B
   * </pre>
   *
   * <p>This is more accurate than Euler discretization (A_d = I + A*dt) for systems with fast
   * dynamics.
   *
   * @param dt Sample period in seconds.
   * @return A new discretized LinearSystem.
   */
  public LinearSystem discretize(double dt) {
    // High-precision discretization using native matrix exponential
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
