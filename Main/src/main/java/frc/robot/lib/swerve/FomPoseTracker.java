package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class FomPoseTracker {
  public Pose2d pose = new Pose2d();
  public double fomOdom = 0.3;
  public double fomVision = 1e9;

  public void reset(Pose2d start) {
    pose = start;
    fomOdom = 0.3;
    fomVision = 1e9;
  }

  public void predict(Translation2d dField, double dYawRad) {
    pose =
        new Pose2d(
            pose.getX() + dField.getX(),
            pose.getY() + dField.getY(),
            pose.getRotation().plus(new Rotation2d(dYawRad)));
  }

  public void setOdomFom(double newFomMeters) {
    fomOdom = Math.max(0.0, newFomMeters);
  }

  public void correctWithVisionUsingFoms(
      Pose2d visionPose, double odomFomExternal, double visionFom) {
    if (visionFom >= 1e8) return;
    double wO = 1.0 / Math.max(1e-6, odomFomExternal);
    double wV = 1.0 / Math.max(1e-6, visionFom);
    double alphaV = wV / (wO + wV);
    fusePose(visionPose, alphaV);
    fomVision = visionFom;
    if (visionFom < fomOdom) fomOdom = Math.max(0.0, visionFom * 0.9);
  }

  public void correctWithVisionUsingWeights(Pose2d visionPose, double wOdom, double wVision) {
    double wO = Math.max(0.0, wOdom);
    double wV = Math.max(0.0, wVision);
    if (wO + wV <= 1e-9) return;
    double alphaV = wV / (wO + wV);
    fusePose(visionPose, alphaV);
  }

  public void correctWithVision(Pose2d visionPose, double visionFom) {
    if (visionFom >= 1e8) return;
    double wO = 1.0 / Math.max(1e-6, fomOdom);
    double wV = 1.0 / Math.max(1e-6, visionFom);
    double alphaV = wV / (wO + wV);
    fusePose(visionPose, alphaV);
    fomVision = visionFom;
    if (visionFom < fomOdom) fomOdom = Math.max(0.0, visionFom * 0.9);
  }

  private void fusePose(Pose2d visionPose, double alphaVision) {
    double aV = SwerveUtil.clamp(alphaVision, 0.0, 1.0);
    double nx = SwerveUtil.lerp(pose.getX(), visionPose.getX(), aV);
    double ny = SwerveUtil.lerp(pose.getY(), visionPose.getY(), aV);
    double dth = SwerveUtil.wrap(visionPose.getRotation().minus(pose.getRotation()).getRadians());
    Rotation2d nth = pose.getRotation().plus(new Rotation2d(aV * dth));
    pose = new Pose2d(nx, ny, nth);
  }
}
