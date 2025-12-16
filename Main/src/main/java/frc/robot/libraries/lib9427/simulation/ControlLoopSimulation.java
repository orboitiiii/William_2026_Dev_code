package frc.robot.libraries.lib9427.simulation;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Random;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.libraries.lib9427.NativeKalmanFilter;
import frc.robot.libraries.lib9427.Team9427JNI;
import frc.robot.libraries.lib9427.modeling.LinearSystem;
import frc.robot.libraries.lib9427.modeling.SystemFactory;
import frc.robot.libraries.lib9427.profiles.MotionConstraints;
import frc.robot.libraries.lib9427.profiles.MotionState;
import frc.robot.libraries.lib9427.profiles.SCurveProfile;
import frc.robot.libraries.lib9427.utils.ControlUtils;

/** Validates the full control stack: Profile -> Feedforward -> LQR -> Plant -> Kalman Filter. */
public class ControlLoopSimulation {

  public static void main(String[] args) throws IOException {
    System.out.println("Starting Full Stack 9427 Control Simulation...");

    // 1. Define Physics (The Plant)
    // Elevator: 5kg mass, 2x Bag Motors, 10:1 Gearing, 0.05m radius spool
    LinearSystem realPlant =
        SystemFactory.createElevator(DCMotor.getKrakenX60(2), 10.0, 0.05, 10.0);
    LinearSystem discretePlant = realPlant.discretize(0.02);

    // 2. Define Estimator (Kalman Filter)
    NativeKalmanFilter kf = new NativeKalmanFilter(2, 1, 1);
    double[] Q_kf = ControlUtils.makeCovarianceMatrix(0.001, 0.01); // State noise
    kf.setModel(discretePlant.getA(), discretePlant.getB(), Q_kf);

    // 3. Define Controller (LQR)
    double[] Q_lqr = ControlUtils.makeCostMatrix(0.01, 0.1); // Max err: 1cm, 10cm/s
    double[] R_lqr = ControlUtils.makeCostMatrix(12.0); // Max err: 12V
    double[] K =
        Team9427JNI.computeLQR(discretePlant.getA(), discretePlant.getB(), Q_lqr, R_lqr, 2, 1);

    // 4. Define Motion Profile (Reference)
    // Move 0.5 meters in roughly 1 second. Max V=1.0, Max A=2.0, Max J=10.0
    SCurveProfile profile = new SCurveProfile(new MotionConstraints(1.0, 2.0, 10.0), 0.0, 0.5);

    // Simulation State
    double dt = 0.02; // 20ms loop
    double tTotal = 2.0; // Run for 2 seconds
    double[] x_true = {0.0, 0.0}; // True state [pos, vel]

    Random noiseGen = new Random(9427);

    try (PrintWriter writer = new PrintWriter(new FileWriter("simulation_results.csv"))) {
      writer.println("t,ref_pos,ref_vel,true_pos,true_vel,est_pos,est_vel,u_volts");

      for (double t = 0; t <= tTotal; t += dt) {
        // A. Get Reference from Profile
        MotionState ref = profile.calculate(t);
        MotionState nextRef = profile.calculate(t + dt);

        double[] r_k = {ref.pos, ref.vel};
        double[] r_next = {nextRef.pos, nextRef.vel};

        // B. Get Estimated State from KF (Simulation Step: Read Sensor First)
        // Simulate Sensor Measurement (True Pos + Noise)
        double sensorNoise = noiseGen.nextGaussian() * 0.002; // 2mm noise
        double z = x_true[0] + sensorNoise;

        // KF Correct
        double[] H = {1.0, 0.0}; // Measure position only
        double[] R_noise = ControlUtils.makeCovarianceMatrix(0.005); // Assert 5mm noise std dev
        kf.correct(new double[] {z}, H, R_noise);

        double[] x_hat = kf.getXHat();

        // C. Calculate Feedforward (Plant Inversion)
        double[] u_ff =
            Team9427JNI.computePlantInversion(
                discretePlant.getA(), discretePlant.getB(), r_k, r_next, 2, 1);

        // D. Calculate Feedback (LQR)
        // u_fb = -K * (x_hat - r)
        double u_fb = -(K[0] * (x_hat[0] - ref.pos) + K[1] * (x_hat[1] - ref.vel));

        // E. Total Input
        double u = u_ff[0] + u_fb;
        // Clamp Voltage
        if (u > 12.0) u = 12.0;
        if (u < -12.0) u = -12.0;

        // F. Update Plant (Simulation Physics)
        // x_next = A*x + B*u + ProcessNoise
        double pNoise = noiseGen.nextGaussian() * 0.001;

        double x0_new =
            discretePlant.getA()[0] * x_true[0]
                + discretePlant.getA()[1] * x_true[1]
                + discretePlant.getB()[0] * u;

        double x1_new =
            discretePlant.getA()[2] * x_true[0]
                + discretePlant.getA()[3] * x_true[1]
                + discretePlant.getB()[1] * u
                + pNoise;

        x_true[0] = x0_new;
        x_true[1] = x1_new;

        // G. KF Predict (for next step)
        kf.predict(new double[] {u});

        // Log
        writer.printf(
            "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n",
            t, ref.pos, ref.vel, x_true[0], x_true[1], x_hat[0], x_hat[1], u);
      }
    }

    System.out.println("Simulation Complete. Data saved to simulation_results.csv.");
    kf.close();
  }
}
