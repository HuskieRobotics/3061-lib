package frc.lib.team3061.differential_drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainConstants.SysIDCharacterizationMode;
import org.littletonrobotics.junction.AutoLog;

public interface DifferentialDrivetrainIO {
  @AutoLog
  public static class DifferentialDrivetrainIOInputs {
    double leftMotorSpeed = 0.0;
    double rightMotorSpeed = 0.0;

    double leftEncoderCount = 0.0;
    double rightEncoderCount = 0.0;

    Distance leftPosition = Meters.of(0.0);
    Distance rightPosition = Meters.of(0.0);

    LinearVelocity leftVelocity = MetersPerSecond.of(0.0);
    LinearVelocity rightVelocity = MetersPerSecond.of(0.0);

    Angle heading = Degrees.of(0.0);
    Angle pitch = Degrees.of(0.0);
    Angle roll = Degrees.of(0.0);

    LinearAcceleration xAccelerationG = MetersPerSecondPerSecond.of(0.0);
    LinearAcceleration yAccelerationG = MetersPerSecondPerSecond.of(0.0);
    LinearAcceleration zAccelerationG = MetersPerSecondPerSecond.of(0.0);
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
   * @param xVelocity the desired velocity in the x direction
   * @param rotationalVelocity the desired rotational velocity
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public default void driveRobotRelative(
      LinearVelocity xVelocity, AngularVelocity rotationalVelocity, boolean isOpenLoop) {}

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
