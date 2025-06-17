package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import java.util.List;

public class CrossSubsystemsCommandsFactory {

  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveKp", RobotConfig.getInstance().getDriveToPoseDriveKP());
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveKd", RobotConfig.getInstance().getDriveToPoseDriveKD());
  private static final LoggedTunableNumber driveKi =
      new LoggedTunableNumber("DriveToPoseExample/DriveKi", 0);
  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber(
          "DriveToPoseExample/ThetaKp", RobotConfig.getInstance().getDriveToPoseThetaKP());
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber(
          "DriveToPoseExample/ThetaKd", RobotConfig.getInstance().getDriveToPoseThetaKD());
  private static final LoggedTunableNumber thetaKi =
      new LoggedTunableNumber(
          "DriveToPoseExample/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());

  private static PIDController xController =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get(), LOOP_PERIOD_SECS);
  private static PIDController yController =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get(), LOOP_PERIOD_SECS);
  private static PIDController thetaController =
      new PIDController(thetaKp.get(), thetaKi.get(), thetaKd.get(), LOOP_PERIOD_SECS);

  private CrossSubsystemsCommandsFactory() {}

  public static void registerCommands(
      OperatorInterface oi,
      Drivetrain drivetrain,
      Vision vision,
      Elevator elevator,
      Manipulator manipulator) {

    oi.getInterruptAll()
        .onTrue(getInterruptAllCommand(drivetrain, vision, elevator, manipulator, oi));

    oi.getDriveToPoseButton().onTrue(getDriveToPoseCommand(drivetrain, elevator, oi));

    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(drivetrain, oi));
  }

  private static Command getInterruptAllCommand(
      Drivetrain drivetrain,
      Vision vision,
      Elevator elevator,
      Manipulator manipulator,
      OperatorInterface oi) {
    return Commands.parallel(
            new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
            Commands.runOnce(
                () -> elevator.goToPosition(ElevatorConstants.Positions.BOTTOM), elevator),
            Commands.runOnce(() -> manipulator.resetStateMachine(), manipulator))
        .withName("interrupt all");
  }

  private static Command getDriveToPoseCommand(
      Drivetrain drivetrain, Elevator elevator, OperatorInterface oi) {
    return new DriveToPose(
            CrossSubsystemsCommandsFactory::getTargetPose,
            CrossSubsystemsCommandsFactory::calculateXVelocity,
            CrossSubsystemsCommandsFactory::calculateYVelocity,
            CrossSubsystemsCommandsFactory::calculateThetaVelocity,
            new Transform2d(0.10, 0.05, Rotation2d.fromDegrees(5.0)),
            true,
            (atPose) ->
                LEDs.getInstance()
                    .requestState(atPose ? LEDs.States.AT_POSE : LEDs.States.AUTO_DRIVING_TO_POSE),
            (poseDifference) -> {
              /* do nothing */
            },
            5.0)
        .withName("drive to pose");
  }

  private static Command getDriveToPoseOverrideCommand(
      SwerveDrivetrain drivetrain, OperatorInterface oi) {
    return new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)
        .withName("Override driveToPose");
  }

  private static Pose2d getTargetPose() {
    return new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(90.0));
  }

  private static Double calculateXVelocity(Double currentX, Double targetX) {
    // Update from tunable numbers
    LoggedTunableNumber.ifChanged(
        xController.hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
          yController.setPID(pid[0], pid[1], pid[2]);
        },
        driveKp,
        driveKi,
        driveKd);
    return xController.calculate(currentX, targetX);
  }

  private static Double calculateYVelocity(double currentY, double targetY) {
    LoggedTunableNumber.ifChanged(
        yController.hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
          yController.setPID(pid[0], pid[1], pid[2]);
        },
        driveKp,
        driveKi,
        driveKd);
    return yController.calculate(currentY, targetY);
  }

  private static Double calculateThetaVelocity(double currentTheta, double targetTheta) {
    LoggedTunableNumber.ifChanged(
        thetaController.hashCode(),
        pid -> {
          thetaController.setPID(pid[0], pid[1], pid[2]);
        },
        thetaKp,
        thetaKi,
        thetaKd);
    return thetaController.calculate(currentTheta, targetTheta);
  }
}
