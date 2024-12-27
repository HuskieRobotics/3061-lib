package frc.lib.team3061.drivetrain;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Force;
import frc.lib.team3061.drivetrain.DrivetrainConstants.SysIDCharacterizationMode;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

@java.lang.SuppressWarnings({"java:S1104"})
public interface DrivetrainIO {
  @AutoLog
  public static class SwerveIOInputs {
    public boolean driveEnabled = false;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;

    public double steerAbsolutePositionDeg = 0.0;

    public boolean steerEnabled = false;
    public double steerStatorCurrentAmps = 0.0;
    public double steerSupplyCurrentAmps = 0.0;
    public double steerTempCelsius = 0.0;

    public double[] odometryDrivePositionsMeters = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class DrivetrainIOInputs {
    ChassisSpeeds referenceChassisSpeeds = new ChassisSpeeds();
    ChassisSpeeds measuredChassisSpeeds = new ChassisSpeeds();

    SwerveModulePosition[] swerveModulePositions = {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
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
    double rawHeadingDeg = 0.0;

    Pose2d customPose = new Pose2d();

    double[] odometryTimestamps = new double[] {};
    Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public static class DrivetrainIOInputsCollection {
    SwerveIOInputsAutoLogged[] swerve = {
      new SwerveIOInputsAutoLogged(),
      new SwerveIOInputsAutoLogged(),
      new SwerveIOInputsAutoLogged(),
      new SwerveIOInputsAutoLogged()
    }; // FL, FR, BL, BR

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
   * field frame of reference, The origin of the field is always the blue origin (i.e., the positive
   * x-axis points away from the blue alliance wall). Zero degrees is aligned to the positive x axis
   * and increases in the CCW direction.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param rotationalVelocity the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public default void driveFieldRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {}

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x and y
   * directions, while maintaining the specified rotation. The velocities are specified from the
   * field's frame of reference. In the field frame of reference, The origin of the field is always
   * the blue origin (i.e., the positive x-axis points away from the blue alliance wall). Zero
   * degrees is aligned to the positive x axis and increases in the CCW direction.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param targetDirection the desired direction the robot should face
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public default void driveFieldRelativeFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {}

  /**
   * Sets the swerve modules wheels to point in the specified direction. The direction is specified
   * from the field's frame of reference. In the field frame of reference, The origin of the field
   * is always the blue origin (i.e., the positive x-axis points away from the blue alliance wall).
   * Zero degrees is aligned to the positive x axis and increases in the CCW direction.
   *
   * @param targetDirection the desired direction each swerve module wheel should face
   */
  public default void pointWheelsAt(Rotation2d targetDirection) {}

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities are specified from the robot's frame of reference. In the
   * robot frame of reference, The origin of the robot is always the center of the robot. The
   * positive x direction is forward; the positive y direction, left. Zero degrees is aligned to the
   * positive x axis and increases in the CCW direction.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param rotationalVelocity the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public default void driveRobotRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {}

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities are specified from the robot's frame of reference. In the
   * robot frame of reference, The origin of the robot is always the center of the robot. The
   * positive x direction is forward; the positive y direction, left. Zero degrees is aligned to the
   * positive x axis and increases in the CCW direction.
   *
   * @param speeds the desired chassis speeds
   * @param forcesX the robot-centric wheel forces in the x direction
   * @param forcesY the robot-centric wheel forces in the y direction
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public default void applyRobotSpeeds(
      ChassisSpeeds speeds, Force[] forcesX, Force[] forcesY, boolean isOpenLoop) {}

  /**
   * Applies the specified characterization mode to the drivetrain with the specified value. This is
   * used to characterize the robot for the purpose of system identification.
   *
   * @param mode the specified characterization mode
   * @param value the value to apply to the characterization mode
   */
  public default void applySysIdCharacterization(SysIDCharacterizationMode mode, double value) {}

  /**
   * Sets the robot's center of rotation. The origin is at the center of the robot. The positive x
   * direction is forward; the positive y direction, left.
   *
   * @param centerOfRotation the center of rotation of the robot (in units of meters)
   */
  public default void setCenterOfRotation(Translation2d centerOfRotation) {}

  /**
   * Sets the brake mode of the drivetrain.
   *
   * @param enable true to enable brake mode; false to disable brake mode
   */
  public default void setBrakeMode(boolean enable) {}

  /**
   * Sets the custom odometry of the robot to the specified pose. This method should only be invoked
   * when the rotation of the robot is known (e.g., at the start of an autonomous path). The origin
   * of the field to the lower left corner (i.e., the corner of the field to the driver's right).
   * Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param pose the specified pose to which is set the odometry
   */
  public default void resetPose(Pose2d pose) {}

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
  public default Optional<Pose2d> samplePoseAt(double timestamp) {
    return Optional.empty();
  }

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
  public default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {}
}
