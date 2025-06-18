package frc.lib.team3061.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.team3061.RobotConfig;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by subsystems
 * (drivetrain and vision). This class supports two pose estimators: the main one and a custom one.
 * The assumption is that the custom one is below the AdvantageKit hardware abstraction layer and,
 * therefore doesn't support tuning and debugging via replay. The pose estimator defined in this
 * class is above the AdvantageKit hardware abstraction layer and supports tuning and debugging via
 * replay.
 */
public class SwerveRobotOdometry extends RobotOdometry {
  private SwerveDrivePoseEstimator estimator = null;

  public SwerveRobotOdometry() {
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
  @Override
  public Pose2d getEstimatedPose() {
    return this.estimator.getEstimatedPosition();
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
    super.resetCustomPose(poseMeters);
    this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
  }

  /**
   * Updates the pose estimator with the current time, gyro angle, and module positions. The custom
   * pose estimator will be updated via its own mechanism.
   *
   * @param currentTimeSeconds the current time in seconds. Note that you must use a timestamp
   *     aligned with the FPGA timebase. Note that this is different than the timebase used by CTRE
   *     which uses an epoch since system startup (i.e., the epoch of this timestamp is the same
   *     epoch as Utils.getCurrentTimeSeconds()).
   * @param gyroAngle the current raw heading of the gyro
   * @param modulePositions the current positions of the swerve modules
   * @return the estimated pose of the robot
   */
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    Logger.recordOutput("RobotOdometry/updateTime", currentTimeSeconds);
    return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  protected void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    this.estimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  protected Optional<Pose2d> sampleAt(double timestampSeconds) {
    return this.estimator.sampleAt(timestampSeconds);
  }
}
