package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.robot.operator_interface.OperatorInterface;

public class DifferentialDrivetrainCommandFactory {

  private DifferentialDrivetrainCommandFactory() {}

  public static void registerCommands(
      OperatorInterface oi, DifferentialDrivetrain differentialDrivetrain) {

    differentialDrivetrain.setDefaultCommand(
        new ArcadeDrive(differentialDrivetrain, oi::getTranslateX, oi::getRotate));

    Rotation2d[] startHeading = {new Rotation2d()};

    oi.getSpinOneRevolutionButton()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(() -> startHeading[0] = differentialDrivetrain.getHeading()),
                    Commands.run(
                            () ->
                                differentialDrivetrain.arcadeDrive(
                                    MetersPerSecond.of(0), RadiansPerSecond.of(2.0)),
                            differentialDrivetrain)
                        .until(
                            () ->
                                Math.abs(
                                        differentialDrivetrain
                                            .getHeading()
                                            .minus(startHeading[0])
                                            .getRadians())
                                    >= 2 * Math.PI),
                    Commands.runOnce(
                        () ->
                            differentialDrivetrain.arcadeDrive(
                                MetersPerSecond.of(0), RadiansPerSecond.of(0))))
                .withName("SpinOneRevolution"));
  }
}
