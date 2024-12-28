package frc.lib.team3061.util;

import static frc.robot.Constants.TUNING_MODE;

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
 * (drivetrain and vision). This class supports two pose estimators: the main one and a custom one.
 * The assumption is that the custom one is below the AdvantageKit hardware abstraction layer and,
 * therefore doesn't support tuning and debugging via replay. The pose estimator defined in this
 * class is above the AdvantageKit hardware abstraction layer and supports tuning and debugging via
 * replay.
 */
public class RobotOdometry {
  private static final RobotOdometry robotOdometry = new RobotOdometry();
  private SwerveDrivePoseEstimator estimator = null;
  private CustomPoseEstimator customEstimator = null;

  /**
   * When tuning vision, it is useful to log vision pose estimates and display them in
   * AdvantageScope but not add them to the pose estimators.
   */
  private static final boolean INCLUDE_VISION_POSE_ESTIMATES = true;

  /**
   * When tuning vision, it is useful to log vision pose estimates and display them in
   * AdvantageScope but not add them to the custom pose estimator. This flag controls whether vision
   * pose estimates are added to the custom pose estimator.
   */
  private static final boolean INCLUDE_VISION_POSE_ESTIMATES_IN_CUSTOM_ESTIMATOR = true;

  private RobotOdometry() {
    estimator =
        new SwerveDrivePoseEstimator(
            RobotConfig.getInstance().getSwerveDriveKinematics(),
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d());
  }

  /**
   * Returns the estimated pose of the robot as determined by this class's pose estimator.
   *
   * @return the estimated pose of the robot
   */
  public Pose2d getEstimatedPose() {
    return this.estimator.getEstimatedPosition();
  }

  /**
   * Returns the estimated pose of the robot as determined by the custom pose estimator. If there is
   * no custom pose estimator, this method returns a zero pose.
   *
   * @return the estimated pose of the robot as determined by the custom pose estimator
   */
  public Pose2d getCustomEstimatedPose() {
    return this.customEstimator != null
        ? this.customEstimator.getCustomEstimatedPose()
        : new Pose2d();
  }

  /**
   * Resets the pose of the robot to the specified pose. This method resets both the main pose
   * estimator and the custom pose estimator.
   *
   * @param gyroAngle the current raw heading of the gyro
   * @param modulePositions the current positions of the swerve modules
   * @param poseMeters the new pose of the robot
   */
  public void resetPose(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    if (this.customEstimator != null) this.customEstimator.resetCustomPose(poseMeters);
  }

  /**
   * Updates the pose estimator with the current time, gyro angle, and module positions. The custom
   * pose estimator will be updated via its own mechanism.
   *
   * @param currentTimeSeconds the current time in seconds. Note that you must use a timestamp with
   *     an epoch since system startup (i.e., the epoch of this timestamp is the same epoch as
   *     Utils.getCurrentTimeSeconds()). This means that you should use
   *     Utils.getCurrentTimeSeconds() as your time source or sync the epochs. An FPGA timestamp can
   *     be converted to the correct timebase using Utils.fpgaToCurrentTime(double).
   * @param gyroAngle the current raw heading of the gyro
   * @param modulePositions the current positions of the swerve modules
   * @return the estimated pose of the robot
   */
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    Logger.recordOutput("RobotOdometry/updateTime", currentTimeSeconds);
    return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  /**
   * Adds a vision measurement to the pose estimator if including vision pose estimates is enabled.
   * Also adds the vision measurement to the custom pose estimator if enabled. The timestamp is
   * adjusted by the specified latency. The specified standard deviations are passed to the pose
   * estimators.
   *
   * <p>When tuning, the difference between the vision pose estimate and the pose estimate
   * corresponding to the same timestamp is logged. This is useful when tuning the vision system;
   * especially, the latency.
   *
   * @param visionRobotPoseMeters the pose of the robot as determined by vision
   * @param timestampSeconds the timestamp of the vision measurement. . Note that you must use a
   *     timestamp with an epoch since system startup (i.e., the epoch of this timestamp is the same
   *     epoch as Utils.getCurrentTimeSeconds()). This means that you should use
   *     Utils.getCurrentTimeSeconds() as your time source or sync the epochs. An FPGA timestamp can
   *     be converted to the correct timebase using Utils.fpgaToCurrentTime(double).
   * @param latencyAdjustmentSeconds the latency adjustment of the vision system in seconds
   * @param visionMeasurementStdDevs the standard deviations of the vision measurement
   */
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
    if (TUNING_MODE && this.customEstimator != null) {
      var sample = this.customEstimator.samplePoseAt(adjustedTimestamp);
      if (!sample.isEmpty()) {
        Pose2d pastPose = sample.get();
        Transform2d diff = pastPose.minus(visionRobotPoseMeters);
        Logger.recordOutput("RobotOdometry/pastPose", pastPose);
        Logger.recordOutput("RobotOdometry/visionPoseDiff", diff);
      }
    }
  }

  /**
   * Sets the custom pose estimator.
   *
   * @param customOdometry the custom pose estimator
   */
  public void setCustomEstimator(CustomPoseEstimator customOdometry) {
    this.customEstimator = customOdometry;
  }

  /**
   * Returns the singleton instance of this class.
   *
   * @return the singleton instance of this class
   */
  public static RobotOdometry getInstance() {
    return robotOdometry;
  }
}
