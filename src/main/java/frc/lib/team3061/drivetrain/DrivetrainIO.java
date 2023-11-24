package frc.lib.team3061.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.team3061.gyro.GyroIO.GyroIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
  public static class SwerveIOInputs {
    double driveDistanceMeters = 0.0;
    double driveVelocityMetersPerSec = 0.0;
    double driveVelocityReferenceMetersPerSec = 0.0;
    double driveVelocityErrorMetersPerSec = 0.0;
    double driveAppliedVolts = 0.0;
    double driveStatorCurrentAmps = 0.0;
    double driveSupplyCurrentAmps = 0.0;
    double driveTempCelsius = 0.0;

    double steerAbsolutePositionDeg = 0.0;
    double steerPositionDeg = 0.0;
    double steerPositionReferenceDeg = 0.0;
    double steerPositionErrorDeg = 0.0;
    double steerAppliedVolts = 0.0;
    double steerStatorCurrentAmps = 0.0;
    double steerSupplyCurrentAmps = 0.0;
    double steerTempCelsius = 0.0;
  }

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class DrivetrainIOInputs {
    SwerveIOInputs frontLeft = new SwerveIOInputs();
    SwerveIOInputs frontRight = new SwerveIOInputs();
    SwerveIOInputs backLeft = new SwerveIOInputs();
    SwerveIOInputs backRight = new SwerveIOInputs();

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    SwerveModuleState[] swerveReferenceStates = new SwerveModuleState[4];
    SwerveModuleState[] swerveMeasuredStates = new SwerveModuleState[4];

    Pose2d robotPoseWithoutGyro = new Pose2d();
    Pose2d robotPose = new Pose2d();
    Pose3d robotPose3D = new Pose3d();

    double averageDriveCurrent = 0.0;

    GyroIOInputs gyro = new GyroIOInputs();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DrivetrainIOInputs inputs) {}

  /**
   * Sets the swerve modules in the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This prevents the robot from rolling on an inclined surface and makes it more
   * difficult for other robots to push the robot, which is useful when shooting.
   */
  public default void holdXStance() {}

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities are specified from the field's frame of reference. In the
   * field frame of reference, the origin of the field to the lower left corner (i.e., the corner of
   * the field to the driver's right). Zero degrees is away from the driver and increases in the CCW
   * direction.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param rotationalVelocity the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public default void driveFieldRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {}

  public default void driveFieldRelativeFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {}

  public default void pointWheelsAt(Rotation2d targetDirection) {}

  public default void driveRobotRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {}

  public default void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {}

  /**
   * Sets the rotation of the robot to the specified value. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
   * facing away from the driver station; CCW is positive.
   *
   * @param expectedYaw the rotation of the robot (in degrees)
   */
  public default void setGyroOffset(double expectedYaw) {}
}
