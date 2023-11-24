package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
}
