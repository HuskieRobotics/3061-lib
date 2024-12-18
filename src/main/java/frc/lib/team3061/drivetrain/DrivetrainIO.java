package frc.lib.team3061.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.team3061.gyro.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

@java.lang.SuppressWarnings({"java:S1104"})
public interface DrivetrainIO {
  @AutoLog
  public static class SwerveIOInputs {
    public boolean driveEnabled = false;
    public double driveDistanceMeters = 0.0;
    public double driveVelocityMetersPerSec = 0.0;
    public double driveVelocityReferenceMetersPerSec = 0.0;
    public double driveVelocityErrorMetersPerSec = 0.0;
    public double driveAccelerationMetersPerSecPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;

    public boolean steerEnabled = false;
    public double steerAbsolutePositionDeg = 0.0;
    public double steerPositionDeg = 0.0;
    public double steerPositionReferenceDeg = 0.0;
    public double steerPositionErrorDeg = 0.0;
    public double steerVelocityRevPerMin = 0.0;
    public double steerAccelerationMetersPerSecPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerStatorCurrentAmps = 0.0;
    public double steerSupplyCurrentAmps = 0.0;
    public double steerTempCelsius = 0.0;

    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class DrivetrainIOInputs {
    double targetVXMetersPerSec = 0.0;
    double targetVYMetersPerSec = 0.0;
    double targetAngularVelocityRadPerSec = 0.0;

    double measuredVXMetersPerSec = 0.0;
    double measuredVYMetersPerSec = 0.0;
    double measuredAngularVelocityRadPerSec = 0.0;

    SwerveModuleState[] swerveReferenceStates = {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
    SwerveModuleState[] swerveMeasuredStates = {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };

    double averageDriveCurrent = 0.0;

    double[] odometryTimestamps = new double[] {};
  }

  public static class DrivetrainIOInputsCollection {
    SwerveIOInputsAutoLogged[] swerve = {
      new SwerveIOInputsAutoLogged(),
      new SwerveIOInputsAutoLogged(),
      new SwerveIOInputsAutoLogged(),
      new SwerveIOInputsAutoLogged()
    }; // FL, FR, BL, BR

    GyroIOInputsAutoLogged gyro = new GyroIOInputsAutoLogged();

    DrivetrainIOInputsAutoLogged drivetrain = new DrivetrainIOInputsAutoLogged();
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DrivetrainIOInputsCollection inputs) {}

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

  /**
   * Sets the robot's center of rotation. The origin is at the center of the robot. The positive x
   * direction is forward; the positive y direction, left.
   *
   * @param centerOfRotation the center of rotation of the robot (in units of meters)
   */
  public default void setCenterOfRotation(Translation2d centerOfRotation) {}

  /**
   * Sets the odometry of the robot to the specified pose. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). The origin of
   * the field to the lower left corner (i.e., the corner of the field to the driver's right). Zero
   * degrees is away from the driver and increases in the CCW direction.
   *
   * @param pose the specified pose to which is set the odometry
   */
  public default void resetPose(Pose2d pose) {}

  /**
   * Supplies the drive motors with the specified voltage. Used for drivetrain characterization.
   *
   * @param volts the commanded voltage
   */
  public default void setDriveMotorVoltage(double volts) {}

  /**
   * Supplies the steer motors with the specified voltage. Used for drivetrain characterization.
   *
   * @param volts the commanded voltage
   */
  public default void setSteerMotorVoltage(double volts) {}

  /**
   * Supplies the drive motors with the specified current. Used for drivetrain characterization with
   * TorqueCurrentFOC.
   *
   * @param amps the commanded current
   */
  public default void setDriveMotorCurrent(double amps) {}

  /**
   * Supplies the steer motors with the specified current. Used for drivetrain characterization with
   * TorqueCurrentFOC.
   *
   * @param amps the commanded current
   */
  public default void setSteerMotorCurrent(double amps) {}

  public default void setBrakeMode(boolean enable) {}
}
