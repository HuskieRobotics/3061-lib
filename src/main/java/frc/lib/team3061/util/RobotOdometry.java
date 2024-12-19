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
  private CustomOdometry customOdometry = null;
  private SwerveModulePosition[] defaultPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private static final boolean INCLUDE_VISION_POSE_ESTIMATES = false;
  private static final boolean INCLUDE_VISION_POSE_ESTIMATES_IN_CUSTOM_ODOMETRY = false;

  private RobotOdometry() {
    estimator =
        new SwerveDrivePoseEstimator(
            RobotConfig.getInstance().getSwerveDriveKinematics(),
            new Rotation2d(),
            defaultPositions,
            new Pose2d());
  }

  public Pose2d getEstimatedPose() {
    // changed to not consider custom odometry at all
    return this.estimator.getEstimatedPosition();
  }

  // new method to get estimated position of the custom odometry (in order to compare with the
  // default estimator)
  public Pose2d getCustomEstimatedPose() {
    return this.customOdometry != null ? this.customOdometry.getCustomEstimatedPose() : null;
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {

    // resetting position on both estimators at the same time since we are considering both in
    // parallel now
    this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    if (this.customOdometry != null) this.customOdometry.resetCustomPose(poseMeters);
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      double latencyAdjustmentSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    double adjustedTimestamp = timestampSeconds + latencyAdjustmentSeconds;
    if (INCLUDE_VISION_POSE_ESTIMATES) {
      this.estimator.addVisionMeasurement(
          visionRobotPoseMeters, adjustedTimestamp, visionMeasurementStdDevs);

      if (INCLUDE_VISION_POSE_ESTIMATES_IN_CUSTOM_ODOMETRY && this.customOdometry != null) {
        this.customOdometry.addVisionMeasurement(
            visionRobotPoseMeters, adjustedTimestamp, visionMeasurementStdDevs);
      }
    }

    // log the difference between the vision pose estimate and the pose estimate corresponding to
    // the same timestamp
    var sample = this.customOdometry.samplePoseAt(adjustedTimestamp);
    if (!sample.isEmpty()) {
      Pose2d pastPose = sample.get();
      Transform2d diff = pastPose.minus(visionRobotPoseMeters);
      Logger.recordOutput("RobotOdometry/visionPoseDiff", diff);
    }
  }

  public void setCustomOdometry(CustomOdometry customOdometry) {
    this.customOdometry = customOdometry;
  }

  public static RobotOdometry getInstance() {
    return robotOdometry;
  }
}
