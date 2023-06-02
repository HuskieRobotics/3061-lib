package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;
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
public class TeleopSwerve extends CommandBase {

  private final Drivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private static final double DEADBAND = 0.1;
  private double lastAngularVelocity;
  private double lastXVelocity;
  private double lastYVelocity;

  private final double maxVelocityMetersPerSecond = RobotConfig.getInstance().getRobotMaxVelocity();
  private final double maxAngularVelocityRadiansPerSecond =
      RobotConfig.getInstance().getRobotMaxAngularVelocity();

  private final TunableNumber maxTurnAcceleration =
      new TunableNumber(
          "TeleopSwerve/maxTurnAcceleration",
          RobotConfig.getInstance().getRobotMaxTurnAcceleration());
  private final TunableNumber joystickPower = new TunableNumber("TeleopSwerve/joystickPower", 2.0);
  private final TunableNumber maxDriveAcceleration =
      new TunableNumber(
          "TeleopSwerve/maxDriveAcceleration",
          RobotConfig.getInstance().getRobotMaxDriveAcceleration());

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
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", true);
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
    double rotationPercentage = modifyAxis(rotationSupplier.getAsDouble(), joystickPower.get());

    double xVelocity = xPercentage * maxVelocityMetersPerSecond;
    double yVelocity = yPercentage * maxVelocityMetersPerSecond;
    double rotationalVelocity = rotationPercentage * maxAngularVelocityRadiansPerSecond;

    Logger.getInstance().recordOutput("TeleopSwerve/xVelocity", xVelocity);
    Logger.getInstance().recordOutput("TeleopSwerve/yVelocity", yVelocity);
    Logger.getInstance().recordOutput("TeleopSwerve/rotationalVelocity", rotationalVelocity);

    // if the robot is not in turbo mode, limit the acceleration
    if (!drivetrain.getTurbo()) {
      double driveAccelerationMetersPer20Ms =
          maxDriveAcceleration.get() * Constants.LOOP_PERIOD_SECS;

      if (Math.abs(xVelocity - lastXVelocity) > driveAccelerationMetersPer20Ms) {
        xVelocity =
            lastXVelocity
                + Math.copySign(driveAccelerationMetersPer20Ms, xVelocity - lastXVelocity);
      }

      if (Math.abs(yVelocity - lastYVelocity) > driveAccelerationMetersPer20Ms) {
        yVelocity =
            lastYVelocity
                + Math.copySign(driveAccelerationMetersPer20Ms, yVelocity - lastYVelocity);
      }

      double turnAccelerationRadiansPer20Ms =
          maxTurnAcceleration.get() * Constants.LOOP_PERIOD_SECS;

      if (Math.abs(rotationalVelocity - lastAngularVelocity) > turnAccelerationRadiansPer20Ms) {
        rotationalVelocity =
            lastAngularVelocity
                + Math.copySign(
                    turnAccelerationRadiansPer20Ms, rotationalVelocity - lastAngularVelocity);
      }
    }

    lastXVelocity = xVelocity;
    lastYVelocity = yVelocity;
    lastAngularVelocity = rotationalVelocity;

    drivetrain.drive(xVelocity, yVelocity, rotationalVelocity, true, false);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. The drivetrain is
   * left in whatever state it was in when the command finished.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
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
