package frc.lib.team3061.util;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;

public interface CustomPoseEstimator {
  /**
   * Returns the pose as determined by the custom pose estimator of the robot. The origin of the
   * field to the lower left corner (i.e., the corner of the field to the driver's right). Zero
   * degrees is away from the driver and increases in the CCW direction.
   *
   * @return the pose as determined by the custom pose estimator of the robot
   */
  Pose2d getCustomEstimatedPose();

  /**
   * Return the custom pose at a given timestamp, if the buffer is not empty.
   *
   * @param timestampSeconds The pose's timestamp. Note that you must use a timestamp with an epoch
   *     since system startup (i.e., the epoch of this timestamp is the same epoch as {@link
   *     Utils#getCurrentTimeSeconds}). This means that you should use {@link
   *     Utils#getCurrentTimeSeconds} as your time source in this case. An FPGA timestamp can be
   *     converted to the correct timebase using {@link Utils#fpgaToCurrentTime}.
   * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
   */
  Optional<Pose2d> samplePoseAt(double timestamp);

  /**
   * Sets the custom pose estimator of the robot to the specified pose. This method should only be
   * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
   * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
   * right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param pose the specified pose to which is set the odometry
   */
  void resetCustomPose(Pose2d pose);

  /**
   * Adds a vision measurement to the Kalman Filter for the custom pose estimator. This will correct
   * the odometry pose estimate while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that you must
   *     use a timestamp with an epoch since system startup (i.e., the epoch of this timestamp is
   *     the same epoch as {@link Utils#getCurrentTimeSeconds}). This means that you should use
   *     {@link Utils#getCurrentTimeSeconds} as your time source or sync the epochs. An FPGA
   *     timestamp can be converted to the correct timebase using {@link Utils#fpgaToCurrentTime}.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
}
