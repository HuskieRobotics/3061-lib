package frc.lib.team3061.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.team3061.RobotConfig;

@java.lang.SuppressWarnings({"java:S6548"})

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by subsystems
 * (drivetrain and vision)
 */
public class RobotOdometry {
  private static final RobotOdometry robotOdometry = new RobotOdometry();
  private SwerveDrivePoseEstimator estimator = null;
  private SwerveModulePosition[] defaultPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private RobotOdometry() {
    estimator =
        new SwerveDrivePoseEstimator(
            RobotConfig.getInstance().getSwerveDriveKinematics(),
            new Rotation2d(),
            defaultPositions,
            new Pose2d());
  }

  /**
   * Returns the estimated pose of the robot (e.g., x and y position of the robot on the field and
   * the robot's rotation). The origin of the field to the lower left corner (i.e., the corner of
   * the field to the driver's right). Zero degrees is away from the driver and increases in the CCW
   * direction.
   *
   * @return the pose of the robot
   */
  public Pose2d getEstimatedPose() {
    return this.estimator.getEstimatedPosition();
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    this.estimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public static RobotOdometry getInstance() {
    return robotOdometry;
  }
}
