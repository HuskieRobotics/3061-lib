package frc.robot.commands;

import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.shooter.Shooter;
import java.util.List;
import java.util.Optional;

public class CrossSubsystemsCommandsFactory {

  private static final LoggedTunableNumber driveXKp =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveXKp", RobotConfig.getInstance().getDriveToPoseDriveXKP());
  private static final LoggedTunableNumber driveYKp =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveYKp", RobotConfig.getInstance().getDriveToPoseDriveYKP());
  private static final LoggedTunableNumber driveXKd =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveXKd", RobotConfig.getInstance().getDriveToPoseDriveXKD());
  private static final LoggedTunableNumber driveYKd =
      new LoggedTunableNumber(
          "DriveToPoseExample/DriveYKd", RobotConfig.getInstance().getDriveToPoseDriveYKD());
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

  private static final LoggedTunableNumber driveToPoseMaxVelocity =
      new LoggedTunableNumber(
          "DriveToBank/DriveToPoseMaxVelocity",
          RobotConfig.getInstance().getDriveToPoseDriveMaxVelocityMPS());
  private static final LoggedTunableNumber driveToPoseMaxAcceleration =
      new LoggedTunableNumber(
          "DriveToBank/DriveToPoseMaxAcceleration",
          RobotConfig.getInstance().getDriveToPoseDriveMaxAccelerationMPSPS());

  public static final ProfiledPIDController xController =
      new ProfiledPIDController(
          driveXKp.get(),
          driveKi.get(),
          driveXKd.get(),
          new TrapezoidProfile.Constraints(
              driveToPoseMaxVelocity.get(), driveToPoseMaxAcceleration.get()));
  public static final ProfiledPIDController yController =
      new ProfiledPIDController(
          driveYKp.get(),
          driveKi.get(),
          driveYKd.get(),
          new TrapezoidProfile.Constraints(
              driveToPoseMaxVelocity.get(), driveToPoseMaxAcceleration.get()));
  public static final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(
              RobotConfig.getInstance().getDriveToPoseTurnMaxVelocityRPS(),
              RobotConfig.getInstance().getDriveToPoseTurnMaxAccelerationRPSPS()));

  private CrossSubsystemsCommandsFactory() {}

  public static void registerCommands(
      OperatorInterface oi,
      SwerveDrivetrain swerveDrivetrain,
      Vision vision,
      Arm arm,
      Elevator elevator,
      Manipulator manipulator,
      Shooter shooter) {

    oi.getClearAllFaults()
        .onTrue(FaultReporter.getInstance().getClearAllFaultsCommand().ignoringDisable(true));
    oi.getCheckForFaults()
        .onTrue(FaultReporter.getInstance().getCheckForFaultsCommand().ignoringDisable(true));

    oi.getDriveToPoseButton().onTrue(getDriveToPoseCommand(swerveDrivetrain, elevator, oi));
    oi.getOverrideDriveToPoseButton().onTrue(getDriveToPoseOverrideCommand(swerveDrivetrain, oi));

    oi.getSimulateCollisionButton()
        .onTrue(
            Commands.runOnce(
                () ->
                    swerveDrivetrain.resetPose(
                        RobotOdometry.getInstance()
                            .getEstimatedPose()
                            .plus(new Transform2d(3.0, 3.0, new Rotation2d())))));

    configureCrossSubsystemsTriggers(arm, elevator, manipulator, shooter, swerveDrivetrain);

    oi.getInterruptAll()
        .onTrue(
            getInterruptAllCommand(
                arm, elevator, manipulator, shooter, swerveDrivetrain, vision, oi));

    oi.getSnakeDriveButton().toggleOnTrue(getSnakeDriveCommand(oi, swerveDrivetrain));

    registerSysIdCommands(oi);
  }

  public static void registerCommands(
      OperatorInterface oi, DifferentialDrivetrain differentialDrivetrain, Vision vision, Arm arm) {

    oi.getInterruptAll().onTrue(getInterruptAllCommand(arm, differentialDrivetrain, vision, oi));

    registerSysIdCommands(oi);
  }

  public static Command getSnakeDriveCommand(OperatorInterface oi, SwerveDrivetrain drivetrain) {
    return new TeleopSwerve(
            drivetrain,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            () -> {
              if (Math.hypot(oi.getTranslateX(), oi.getTranslateY()) > 0.06) {
                return Optional.of(new Rotation2d(oi.getTranslateX(), oi.getTranslateY()));
              } else {
                return Optional.empty();
              }
            })
        .withName("Snake Drive Command");
  }

  private static void configureCrossSubsystemsTriggers(
      Arm arm,
      Elevator elevator,
      Manipulator manipulator,
      Shooter shooter,
      SwerveDrivetrain swerveDrivetrain) {
    /* add triggers for cross-subsystem interactions */
  }

  private static void registerSysIdCommands(OperatorInterface oi) {
    oi.getSysIdDynamicForward().whileTrue(SysIdRoutineChooser.getInstance().getDynamicForward());
    oi.getSysIdDynamicReverse().whileTrue(SysIdRoutineChooser.getInstance().getDynamicReverse());
    oi.getSysIdQuasistaticForward()
        .whileTrue(SysIdRoutineChooser.getInstance().getQuasistaticForward());
    oi.getSysIdQuasistaticReverse()
        .whileTrue(SysIdRoutineChooser.getInstance().getQuasistaticReverse());
  }

  private static Command getInterruptAllCommand(
      Arm arm,
      Elevator elevator,
      Manipulator manipulator,
      Shooter shooter,
      SwerveDrivetrain swerveDrivetrain,
      Vision vision,
      OperatorInterface oi) {
    return Commands.parallel(
            SwerveDrivetrainCommandFactory.getDefaultTeleopSwerveCommand(oi, swerveDrivetrain),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
            Commands.runOnce(() -> arm.setAngleRotations(0.0), arm),
            Commands.runOnce(
                () -> elevator.goToPosition(ElevatorConstants.Positions.BOTTOM), elevator),
            Commands.runOnce(manipulator::resetStateMachine, manipulator),
            Commands.runOnce(shooter::setIdleVelocity, shooter))
        .withName("interrupt all");
  }

  private static Command getInterruptAllCommand(
      Arm arm, DifferentialDrivetrain differentialDrivetrain, Vision vision, OperatorInterface oi) {
    return Commands.parallel(
            new ArcadeDrive(differentialDrivetrain, oi::getTranslateX, oi::getRotate),
            Commands.runOnce(() -> vision.specifyCamerasToConsider(List.of(0, 1, 2, 3))),
            Commands.runOnce(() -> arm.setAngleRotations(0.0), arm))
        .withName("interrupt all");
  }

  private static Command getDriveToPoseCommand(
      SwerveDrivetrain swerveDrivetrain, Elevator elevator, OperatorInterface oi) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return new DriveToPose(
            swerveDrivetrain,
            CrossSubsystemsCommandsFactory::getTargetPose,
            xController,
            yController,
            thetaController,
            new Transform2d(0.10, 0.05, Rotation2d.fromDegrees(5.0)),
            true,
            atPose ->
                LEDs.getInstance()
                    .requestState(atPose ? LEDs.States.AT_POSE : LEDs.States.AUTO_DRIVING_TO_POSE),
            CrossSubsystemsCommandsFactory::updatePIDConstants,
            5.0)
        .withName("drive to pose");
  }

  private static Command getDriveToPoseOverrideCommand(
      SwerveDrivetrain drivetrain, OperatorInterface oi) {
    return SwerveDrivetrainCommandFactory.getDefaultTeleopSwerveCommand(oi, drivetrain)
        .withName("Override driveToPose");
  }

  private static Pose2d getTargetPose() {
    return new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(90.0));
  }

  private static void updatePIDConstants(Transform2d poseDifference) {
    // Update from tunable numbers
    LoggedTunableNumber.ifChanged(
        xController.hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
          yController.setPID(pid[0], pid[1], pid[2]);
        },
        driveXKp,
        driveKi,
        driveXKd);

    LoggedTunableNumber.ifChanged(
        yController.hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
          yController.setPID(pid[0], pid[1], pid[2]);
        },
        driveYKp,
        driveKi,
        driveYKd);

    LoggedTunableNumber.ifChanged(
        thetaController.hashCode(),
        pid -> thetaController.setPID(pid[0], pid[1], pid[2]),
        thetaKp,
        thetaKi,
        thetaKd);
  }
}
