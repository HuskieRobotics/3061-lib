package frc.lib.team3061.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.team3061.RobotConfig;
import java.util.concurrent.locks.ReadWriteLock;

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by subsystems
 * (drivetrain and vision)
 */
public class RobotOdometry {
  private static final RobotOdometry robotOdometry = new RobotOdometry();
  private SwerveDrivePoseEstimator estimator;
  private boolean lockRequired = false;
  private ReadWriteLock lock = null;
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

  public Pose2d getEstimatedPosition() {
    if (lockRequired) {
      try {
        this.lock.writeLock().lock();
        return this.estimator.getEstimatedPosition();
      } finally {
        this.lock.writeLock().unlock();
      }
    } else {
      return this.estimator.getEstimatedPosition();
    }
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    if (lockRequired) {
      try {
        this.lock.writeLock().lock();
        this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
      } finally {
        this.lock.writeLock().unlock();
      }
    } else {
      this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    }
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    if (lockRequired) {
      try {
        this.lock.writeLock().lock();
        return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
      } finally {
        this.lock.writeLock().unlock();
      }
    } else {
      return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    }
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    if (lockRequired) {
      try {
        this.lock.writeLock().lock();
        this.estimator.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
      } finally {
        this.lock.writeLock().unlock();
      }
    } else {
      this.estimator.addVisionMeasurement(
          visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
  }

  /**
   * Returns the SwerveDrivePoseEstimator object. If locking is required and this object is used
   * directly, the locking condition will be violated and operations will no longer be properly
   * synchronized. Currently, only the DrivetrainIOCTRE class should invoke this method as it shares
   * its lock with this singleton.
   *
   * @return the pose estimator
   */
  public SwerveDrivePoseEstimator getPoseEstimator() {
    return this.estimator;
  }

  /**
   * Ideally this singleton would create its own lock. However, the lock managed by the
   * SwerveDrivetrain class must be used.
   *
   * @param lock
   */
  public void setLock(ReadWriteLock lock) {
    this.lockRequired = true;
    this.lock = lock;
  }

  public static RobotOdometry getInstance() {
    return robotOdometry;
  }
}
