package frc.lib.team3061.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.team3061.RobotConfig;
import org.littletonrobotics.junction.Logger;

@java.lang.SuppressWarnings({"java:S6548"})

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by subsystems
 * (drivetrain and vision)
 */
public class RobotOdometry {
  private static final RobotOdometry robotOdometry = new RobotOdometry();
  private SwerveDrivePoseEstimator estimator = null;
  private CustomPoseEstimator customEstimator = null;
  private SwerveModulePosition[] defaultPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private static final boolean INCLUDE_VISION_POSE_ESTIMATES = false;
  private static final boolean INCLUDE_VISION_POSE_ESTIMATES_IN_CUSTOM_ESTIMATOR = false;

  private RobotOdometry() {
    estimator =
        new SwerveDrivePoseEstimator(
            RobotConfig.getInstance().getSwerveDriveKinematics(),
            new Rotation2d(),
            defaultPositions,
            new Pose2d());
  }

  public Pose2d getEstimatedPose() {
    return this.estimator.getEstimatedPosition();
  }

  public Pose2d getCustomEstimatedPose() {
    return this.customEstimator != null
        ? this.customEstimator.getCustomEstimatedPose()
        : new Pose2d();
  }

  public void resetPose(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    if (this.customEstimator != null) this.customEstimator.resetCustomPose(poseMeters);
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    Logger.recordOutput("RobotOdometry/updateTime", currentTimeSeconds);
    return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      double latencyAdjustmentSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    double adjustedTimestamp = timestampSeconds + latencyAdjustmentSeconds;
    Logger.recordOutput("RobotOdometry/visionTime", adjustedTimestamp);

    if (INCLUDE_VISION_POSE_ESTIMATES) {
      this.estimator.addVisionMeasurement(
          visionRobotPoseMeters, adjustedTimestamp, visionMeasurementStdDevs);

      if (INCLUDE_VISION_POSE_ESTIMATES_IN_CUSTOM_ESTIMATOR && this.customEstimator != null) {
        this.customEstimator.addVisionMeasurement(
            visionRobotPoseMeters, adjustedTimestamp, visionMeasurementStdDevs);
      }
    }

    // log the difference between the vision pose estimate and the pose estimate corresponding to
    // the same timestamp
    if (this.customEstimator != null) {
      var sample = this.customEstimator.samplePoseAt(adjustedTimestamp);
      if (!sample.isEmpty()) {
        Pose2d pastPose = sample.get();
        Transform2d diff = pastPose.minus(visionRobotPoseMeters);
        Logger.recordOutput("RobotOdometry/pastPose", pastPose);
        Logger.recordOutput("RobotOdometry/visionPoseDiff", diff);
      }
    }
  }

  public void setCustomEstimator(CustomPoseEstimator customOdometry) {
    this.customEstimator = customOdometry;
  }

  public static RobotOdometry getInstance() {
    return robotOdometry;
  }
}
