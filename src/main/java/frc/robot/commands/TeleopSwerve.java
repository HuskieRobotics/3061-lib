package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team6328.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive based on the specified
 * values from the controller(s) and enabled features. Configurable deadband and power function are
 * applied to the controller inputs. If the robot isn't in "turbo" mode, the acceleration is limited
 * based on the configurable constraints. This command is designed to be the default command for the
 * drivetrain subsystem.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: never
 *
 * <p>At End: stops the drivetrain
 */
public class TeleopSwerve extends Command {

  private final SwerveDrivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final Supplier<Rotation2d> angleSupplier;
  private final boolean driveFacingAngle;

  private static final double DEADBAND = 0.1;

  private final LinearVelocity maxVelocity = RobotConfig.getInstance().getRobotMaxVelocity();
  private final AngularVelocity maxAngularVelocity =
      RobotConfig.getInstance().getRobotMaxAngularVelocity();
  private final LoggedTunableNumber joystickPower =
      new LoggedTunableNumber("TeleopSwerve/joystickPower", 2.0);

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param translationYSupplier the supplier of the translation y value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param rotationSupplier the supplier of the rotation value as a percentage of the maximum
   *     rotational velocity as defined by the standard field or robot coordinate system
   */
  public TeleopSwerve(
      SwerveDrivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.angleSupplier = null;
    this.driveFacingAngle = false;

    addRequirements(drivetrain);
  }

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param translationYSupplier the supplier of the translation y value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param rotationSupplier the supplier of the rotation value as a percentage of the maximum
   *     rotational velocity as defined by the standard field or robot coordinate system
   */
  public TeleopSwerve(
      SwerveDrivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      Supplier<Rotation2d> angleSupplier) {
    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = null;
    this.angleSupplier = angleSupplier;
    this.driveFacingAngle = true;

    addRequirements(drivetrain);
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the supplied values and enabled features and invokes the drivetrain
   * subsystem's drive method.
   */
  @Override
  public void execute() {

    // invert the controller input and apply the deadband and exponential function to make the robot
    // more responsive to small changes in the controller
    double xPercentage = modifyAxis(translationXSupplier.getAsDouble(), joystickPower.get());
    double yPercentage = modifyAxis(translationYSupplier.getAsDouble(), joystickPower.get());

    LinearVelocity xVelocity = maxVelocity.times(xPercentage);
    LinearVelocity yVelocity = maxVelocity.times(yPercentage);
    AngularVelocity rotationalVelocity = RadiansPerSecond.of(0.0);

    if (!this.driveFacingAngle) {
      double rotationPercentage = modifyAxis(rotationSupplier.getAsDouble(), joystickPower.get());
      rotationalVelocity = maxAngularVelocity.times(rotationPercentage);
    }

    Logger.recordOutput("TeleopSwerve/xVelocity", xVelocity);
    Logger.recordOutput("TeleopSwerve/yVelocity", yVelocity);
    Logger.recordOutput("TeleopSwerve/rotationalVelocity", rotationalVelocity);

    if (this.driveFacingAngle) {
      drivetrain.driveFacingAngle(xVelocity, yVelocity, angleSupplier.get(), true);
    } else {
      drivetrain.drive(
          xVelocity, yVelocity, rotationalVelocity, true, drivetrain.getFieldRelative());
    }
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value the raw controller value
   * @param power the power to which the value should be raised
   * @return the modified value
   */
  public static double modifyAxis(double value, double power) {
    double modifiedValue = deadband(value, DEADBAND);
    modifiedValue = Math.copySign(Math.pow(modifiedValue, power), modifiedValue);
    return modifiedValue;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
