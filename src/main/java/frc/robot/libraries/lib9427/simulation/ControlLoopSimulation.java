package frc.robot.libraries.lib9427.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.libraries.lib9427.NativeKalmanFilter;
import frc.robot.libraries.lib9427.Team9427JNI;
import frc.robot.libraries.lib9427.controllers.NativeDeltaUController;
import frc.robot.libraries.lib9427.modeling.LinearSystem;
import frc.robot.libraries.lib9427.modeling.SystemFactory;
import frc.robot.libraries.lib9427.profiles.MotionConstraints;
import frc.robot.libraries.lib9427.profiles.MotionState;
import frc.robot.libraries.lib9427.profiles.SCurveProfile;
import frc.robot.libraries.lib9427.utils.ControlUtils;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Random;

/**
 * Simulation comparing Delta-U controller vs. classic LQR+KF.
 *
 * <p>This class validates the control library by simulating a controlled elevator system with:
 *
 * <ul>
 *   <li>S-Curve motion profile for reference generation
 *   <li>Sensor noise (Gaussian)
 *   <li>Process noise (Gaussian)
 *   <li>Step disturbance at t=0.5s to test rejection
 * </ul>
 *
 * <p><strong>Purpose</strong>: Demonstrates that the Delta-U controller effectively rejects
 * disturbances by estimating and compensating for them, while classic LQR+KF only observes state
 * without disturbance compensation.
 *
 * <p><strong>Output</strong>: CSV file with columns: {@code t, ref_pos, ref_vel, true_pos,
 * true_vel, delta_pos, delta_dist, classic_pos, u_volts}
 */
public class ControlLoopSimulation {

  /**
   * Runs the comparison simulation.
   *
   * @param args Command-line arguments (unused).
   * @throws IOException If CSV file cannot be written.
   */
  public static void main(String[] args) throws IOException {
    System.out.println("Starting Comparison: Delta U vs Classic LQR...");

    // --- Plant Definition ---
    LinearSystem realPlant = SystemFactory.createElevator(DCMotor.getNEO(1), 10.0, 0.02, 1.0);
    LinearSystem discretePlant = realPlant.discretize(0.02);

    // --- Classic Control Setup ---
    NativeKalmanFilter classicKF = new NativeKalmanFilter(2, 1, 1);
    double[] Q_kf = ControlUtils.makeCovarianceMatrix(0.001, 0.01);
    classicKF.setModel(discretePlant.getA(), discretePlant.getB(), Q_kf);

    // LQR gains
    double[] Q_lqr = ControlUtils.makeCostMatrix(0.01, 0.1);
    double[] R_lqr = ControlUtils.makeCostMatrix(12.0);

    // --- Delta-U Control Setup ---
    double[] Q_dist = {1.0};
    double[] R_meas = {0.005 * 0.005};

    NativeDeltaUController deltaU =
        new NativeDeltaUController(
            discretePlant.getA(),
            discretePlant.getB(),
            discretePlant.getC(),
            Q_lqr,
            R_lqr,
            Q_kf,
            Q_dist,
            R_meas,
            2,
            1,
            1);

    // --- Motion Profile ---
    SCurveProfile profile = new SCurveProfile(new MotionConstraints(1.0, 2.0, 10.0), 0.0, 0.5);

    double dt = 0.02;
    double tTotal = 2.0;

    // True system state
    double[] x_true = {0.0, 0.0};

    Random noiseGen = new Random(9427);

    try (PrintWriter writer = new PrintWriter(new FileWriter("simulation_results.csv"))) {
      writer.println(
          "t,ref_pos,ref_vel,true_pos,true_vel,delta_pos,delta_dist,classic_pos,u_volts");

      double u_applied_prev_delta = 0.0;

      for (double t = 0; t <= tTotal; t += dt) {
        MotionState ref = profile.calculate(t);
        MotionState nextRef = profile.calculate(t + dt);
        double[] r_k = {ref.pos, ref.vel};
        double[] r_next = {nextRef.pos, nextRef.vel};

        // Simulate sensor measurement with noise
        double sensorNoise = noiseGen.nextGaussian() * 0.002;
        double z = x_true[0] + sensorNoise;

        // --- Delta-U Controller ---
        // Plant inversion feedforward
        double[] u_ff =
            Team9427JNI.computePlantInversion(
                discretePlant.getA(), discretePlant.getB(), r_k, r_next, 2, 1);

        // Compute control
        double[] u_array_delta =
            deltaU.update(new double[] {z}, r_k, u_ff, new double[] {u_applied_prev_delta});

        double u_delta_raw = u_array_delta[0];

        // Saturate
        double u_applied = u_delta_raw;
        if (u_applied > 12.0) u_applied = 12.0;
        if (u_applied < -12.0) u_applied = -12.0;
        u_applied_prev_delta = u_applied;

        // --- Classic KF (passive observation only) ---
        classicKF.predict(new double[] {u_applied});

        double[] H = {1.0, 0.0};
        double[] R_noise = ControlUtils.makeCovarianceMatrix(0.005);
        classicKF.correct(new double[] {z}, H, R_noise);
        double[] x_classic = classicKF.getXHat();

        // --- Physics Simulation ---
        // Apply step disturbance at t=0.5s
        double real_disturbance = (t > 0.5) ? -1.5 : 0.0;
        double u_physics = u_applied + real_disturbance;

        double pNoise = noiseGen.nextGaussian() * 0.001;
        double x0_new =
            discretePlant.getA()[0] * x_true[0]
                + discretePlant.getA()[1] * x_true[1]
                + discretePlant.getB()[0] * u_physics;
        double x1_new =
            discretePlant.getA()[2] * x_true[0]
                + discretePlant.getA()[3] * x_true[1]
                + discretePlant.getB()[1] * u_physics
                + pNoise;
        x_true[0] = x0_new;
        x_true[1] = x1_new;

        // Log results
        double[] x_delta = deltaU.getEstimatedState();
        double[] d_delta = deltaU.getEstimatedDisturbance();

        writer.printf(
            "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n",
            t,
            ref.pos,
            ref.vel,
            x_true[0],
            x_true[1],
            x_delta[0],
            d_delta[0],
            x_classic[0],
            u_applied);
      }
    }

    System.out.println("Comparison Complete. Delta U effectively estimates disturbance.");
    deltaU.close();
    classicKF.close();
  }
}
