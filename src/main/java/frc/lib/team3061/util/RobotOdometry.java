package frc.lib.team3061.util;

import static frc.robot.Constants.TUNING_MODE;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
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
public abstract class RobotOdometry {
  private static RobotOdometry robotOdometry;
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

  protected RobotOdometry() {}

  /**
   * Returns the estimated pose of the robot as determined by this class's pose estimator.
   *
   * @return the estimated pose of the robot
   */
  public abstract Pose2d getEstimatedPose();

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
  protected void resetCustomPose(Pose2d poseMeters) {
    if (this.customEstimator != null) this.customEstimator.resetCustomPose(poseMeters);
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
   *     timestamp aligned with the FPGA timebase. Note that this is different than the timebase
   *     used by CTRE which uses an epoch since system startup (i.e., the epoch of this timestamp is
   *     the same epoch as Utils.getCurrentTimeSeconds()).
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
      this.addVisionMeasurement(visionRobotPoseMeters, adjustedTimestamp, visionMeasurementStdDevs);

      if (INCLUDE_VISION_POSE_ESTIMATES_IN_CUSTOM_ESTIMATOR && this.customEstimator != null) {
        this.customEstimator.addVisionMeasurement(
            visionRobotPoseMeters,
            Utils.fpgaToCurrentTime(adjustedTimestamp),
            visionMeasurementStdDevs);
      }
    }

    // log the difference between the vision pose estimate and the pose estimate corresponding to
    // the same timestamp
    if (TUNING_MODE) {
      var sample = this.sampleAt(adjustedTimestamp);
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

  protected abstract void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);

  protected abstract Optional<Pose2d> sampleAt(double timestampSeconds);

  /**
   * Returns the singleton instance of this class.
   *
   * @return the singleton instance of this class
   */
  public static RobotOdometry getInstance() {
    if (robotOdometry == null) {
      throw new IllegalStateException("RobotOdometry instance not set");
    }
    return robotOdometry;
  }

  public static void setInstance(RobotOdometry robotOdometry) {
    if (RobotOdometry.robotOdometry == null) {
      RobotOdometry.robotOdometry = robotOdometry;
    } else {
      throw new IllegalStateException("RobotOdometry instance already set");
    }
  }
}
