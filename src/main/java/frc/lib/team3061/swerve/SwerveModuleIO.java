package frc.lib.team3061.swerve;

import org.littletonrobotics.junction.AutoLog;

/** Swerve module hardware abstraction interface. */
public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    double drivePositionDeg = 0.0;
    double driveDistanceMeters = 0.0;
    double driveVelocityMetersPerSec = 0.0;
    double driveAppliedPercentage = 0.0;
    double driveStatorCurrentAmps = 0.0;
    double driveSupplyCurrentAmps = 0.0;
    double driveTempCelsius = 0.0;

    double angleAbsolutePositionDeg = 0.0;
    double anglePositionDeg = 0.0;
    double angleVelocityRevPerMin = 0.0;
    double angleAppliedPercentage = 0.0;
    double angleStatorCurrentAmps = 0.0;
    double angleSupplyCurrentAmps = 0.0;
    double angleTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified percentage of full power. */
  public default void setDriveMotorPercentage(double percentage) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocity) {}

  /** Run the turn motor to the specified angle. */
  public default void setAnglePosition(double degrees) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setAngleBrakeMode(boolean enable) {}
}
