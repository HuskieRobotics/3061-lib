package frc.lib.team3061.differential_drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Force;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainConstants.SysIDCharacterizationMode;
import org.littletonrobotics.junction.AutoLog;

public interface DifferentialDrivetrainIO {
  @AutoLog
  public static class DifferentialDrivetrainIOInputs {
    double leftMotorSpeed = 0.0;
    double rightMotorSpeed = 0.0;

    double leftEncoderCount = 0.0;
    double rightEncoderCount = 0.0;

    double leftPositionMeters = 0.0;
    double rightPositionMeters = 0.0;

    double leftVelocityMetersPerSecond = 0.0;
    double rightVelocityMetersPerSecond = 0.0;

    double headingDeg = 0.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;

    double xAccelerationG = 0.0;
    double yAccelerationG = 0.0;
    double zAccelerationG = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DifferentialDrivetrainIOInputs inputs) {}

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities are specified from the robot's frame of reference. In the
   * robot frame of reference, The origin of the robot is always the center of the robot. The
   * positive x direction is forward; the positive y direction, left. Zero degrees is aligned to the
   * positive x axis and increases in the CCW direction.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param rotationalVelocity the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public default void driveRobotRelative(
      double xVelocity, double rotationalVelocity, boolean isOpenLoop) {}

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
}
