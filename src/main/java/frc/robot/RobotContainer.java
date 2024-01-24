// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainIO;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import frc.lib.team3061.drivetrain.DrivetrainIOGeneric;
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIO;
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIOTalonFXPhoenix6;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2Phoenix6;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.pneumatics.Pneumatics;
import frc.lib.team3061.pneumatics.PneumaticsIORev;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.TeleopSwerve;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.configs.NovaCTRERobotConfig;
import frc.robot.configs.NovaCTRETCFRobotConfig;
import frc.robot.configs.NovaRobotConfig;
import frc.robot.configs.PracticeRobotConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.subsystem.Subsystem;
import frc.robot.subsystems.subsystem.SubsystemIO;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotConfig config;
  private Drivetrain drivetrain;
  private Alliance lastAlliance = DriverStation.Alliance.Red;
  private Vision vision;
  private Subsystem subsystem;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 20.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 10.0);

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */
    createRobotConfig();

    LEDs.getInstance();

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {

      switch (Constants.getRobot()) {
        case ROBOT_2023_NOVA_CTRE:
        case ROBOT_2023_NOVA_CTRE_FOC:
        case ROBOT_PRACTICE:
          {
            createCTRESubsystems();
            break;
          }
        case ROBOT_DEFAULT:
        case ROBOT_2023_NOVA:
        case ROBOT_SIMBOT:
          {
            createSubsystems();
            break;
          }
        case ROBOT_SIMBOT_CTRE:
          {
            createCTRESimSubsystems();

            break;
          }
        default:
          break;
      }

    } else {
      drivetrain = new Drivetrain(new DrivetrainIO() {});

      String[] cameraNames = config.getCameraNames();
      VisionIO[] visionIOs = new VisionIO[cameraNames.length];
      for (int i = 0; i < visionIOs.length; i++) {
        visionIOs[i] = new VisionIO() {};
      }
      vision = new Vision(visionIOs);
      subsystem = new Subsystem(new SubsystemIO() {});
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    configureAutoCommands();
  }

  /**
   * The RobotConfig subclass object *must* be created before any other objects that use it directly
   * or indirectly. If this isn't done, a null pointer exception will result.
   */
  private void createRobotConfig() {
    switch (Constants.getRobot()) {
      case ROBOT_DEFAULT:
        config = new DefaultRobotConfig();
        break;
      case ROBOT_2023_NOVA_CTRE:
      case ROBOT_SIMBOT_CTRE:
        config = new NovaCTRERobotConfig();
        break;
      case ROBOT_2023_NOVA_CTRE_FOC:
        config = new NovaCTRETCFRobotConfig();
        break;
      case ROBOT_2023_NOVA:
      case ROBOT_SIMBOT:
        config = new NovaRobotConfig();
        break;
      case ROBOT_PRACTICE:
        config = new PracticeRobotConfig();
        break;
    }
  }

  private void createCTRESubsystems() {
    DrivetrainIO drivetrainIO = new DrivetrainIOCTRE();
    drivetrain = new Drivetrain(drivetrainIO);

    // String[] cameraNames = config.getCameraNames();
    // Transform3d[] robotToCameraTransforms = config.getRobotToCameraTransforms();
    // VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    // AprilTagFieldLayout layout;
    // try {
    //   layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    // } catch (IOException e) {
    //   layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    // }
    // for (int i = 0; i < visionIOs.length; i++) {
    //   visionIOs[i] = new VisionIOPhotonVision(cameraNames[i], layout,
    // robotToCameraTransforms[i]);
    // }
    // vision = new Vision(visionIOs);

    // FIXME: re-enable cameras when installed
    String[] cameraNames = config.getCameraNames();
    VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIO() {};
    }
    vision = new Vision(visionIOs);

    // FIXME: create the hardware-specific subsystem class
    subsystem = new Subsystem(new SubsystemIO() {});
  }

  private void createSubsystems() {
    int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
    int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
    int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
    double[] steerOffsets = config.getSwerveSteerOffsets();
    SwerveModuleIO flModule =
        new SwerveModuleIOTalonFXPhoenix6(
            0, driveMotorCANIDs[0], steerMotorCANDIDs[0], steerEncoderCANDIDs[0], steerOffsets[0]);

    SwerveModuleIO frModule =
        new SwerveModuleIOTalonFXPhoenix6(
            1, driveMotorCANIDs[1], steerMotorCANDIDs[1], steerEncoderCANDIDs[1], steerOffsets[1]);

    SwerveModuleIO blModule =
        new SwerveModuleIOTalonFXPhoenix6(
            2, driveMotorCANIDs[2], steerMotorCANDIDs[2], steerEncoderCANDIDs[2], steerOffsets[2]);

    SwerveModuleIO brModule =
        new SwerveModuleIOTalonFXPhoenix6(
            3, driveMotorCANIDs[3], steerMotorCANDIDs[3], steerEncoderCANDIDs[3], steerOffsets[3]);

    GyroIO gyro = new GyroIOPigeon2Phoenix6(config.getGyroCANID());
    DrivetrainIO drivetrainIO =
        new DrivetrainIOGeneric(gyro, flModule, frModule, blModule, brModule);
    drivetrain = new Drivetrain(drivetrainIO);

    // FIXME: create the hardware-specific subsystem class
    subsystem = new Subsystem(new SubsystemIO() {});

    if (Constants.getRobot() == Constants.RobotType.ROBOT_DEFAULT) {
      new Pneumatics(new PneumaticsIORev());
    }

    if (Constants.getRobot() == Constants.RobotType.ROBOT_SIMBOT) {
      AprilTagFieldLayout layout;
      try {
        layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      } catch (IOException e) {
        layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      }
      vision =
          new Vision(
              new VisionIO[] {
                new VisionIOSim(
                    layout,
                    drivetrain::getPose,
                    RobotConfig.getInstance().getRobotToCameraTransforms()[0])
              });
    } else {
      String[] cameraNames = config.getCameraNames();
      Transform3d[] robotToCameraTransforms = config.getRobotToCameraTransforms();
      VisionIO[] visionIOs = new VisionIO[cameraNames.length];
      AprilTagFieldLayout layout;
      try {
        layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      } catch (IOException e) {
        layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      }
      for (int i = 0; i < visionIOs.length; i++) {
        visionIOs[i] = new VisionIOPhotonVision(cameraNames[i], layout, robotToCameraTransforms[i]);
      }
      vision = new Vision(visionIOs);
    }
  }

  private void createCTRESimSubsystems() {
    DrivetrainIO drivetrainIO = new DrivetrainIOCTRE();
    drivetrain = new Drivetrain(drivetrainIO);

    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    }
    vision =
        new Vision(
            new VisionIO[] {
              new VisionIOSim(
                  layout,
                  drivetrain::getPose,
                  RobotConfig.getInstance().getRobotToCameraTransforms()[0])
            });

    // FIXME: create the hardware-specific subsystem class
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   *
   * <p>FIXME: update for 2024 regions
   */
  private void constructField() {
    Field2d.getInstance().setRegions(new Region2d[] {});
  }

  /**
   * This method scans for any changes to the connected operator interface (e.g., joysticks). If
   * anything changed, it creates a new OI object and binds all of the buttons to commands.
   */
  public void updateOI() {
    OperatorInterface prevOI = oi;
    oi = OISelector.getOperatorInterface();
    if (oi == prevOI) {
      return;
    }

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    configureDrivetrainCommands();

    configureSubsystemCommands();

    configureVisionCommands();

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.run(() -> LEDs.getInstance().setEndgameAlert(true))
                .withTimeout(1.5)
                .andThen(
                    Commands.run(() -> LEDs.getInstance().setEndgameAlert(false))
                        .withTimeout(1.0)));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(true)).withTimeout(0.5),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(false)).withTimeout(0.5),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(true)).withTimeout(0.5),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(false)).withTimeout(1.0)));

    // interrupt all commands by running a command that requires every subsystem. This is used to
    // recover to a known state if the robot becomes "stuck" in a command.
    oi.getInterruptAll()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(drivetrain::disableXstance),
                Commands.runOnce(() -> subsystem.setMotorPower(0)),
                new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Waypoints
    NamedCommands.registerCommand("command1", Commands.print("passed marker 1"));
    NamedCommands.registerCommand("command2", Commands.print("passed marker 2"));
    NamedCommands.registerCommand(
        "enableXStance", Commands.runOnce(drivetrain::enableXstance, drivetrain));
    NamedCommands.registerCommand(
        "disableXStance", Commands.runOnce(drivetrain::disableXstance, drivetrain));
    NamedCommands.registerCommand("wait5Seconds", Commands.waitSeconds(5.0));

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /************ Test Path ************
     *
     * demonstration of PathPlanner auto with event markers
     *
     */
    Command autoTest = new PathPlannerAuto("TestAuto");
    autoChooser.addOption("Test Auto", autoTest);

    /************ Choreo Test Path ************
     *
     * demonstration of PathPlanner hosted Choreo path
     *
     */
    Command choreoAutoTest = new PathPlannerAuto("ChoreoTest");
    autoChooser.addOption("Choreo Auto", choreoAutoTest);

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    Command startPoint =
        Commands.runOnce(
            () ->
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("StartPoint").getPreviewStartingHolonomicPose()),
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Drive Characterization ************
     *
     * useful for characterizing the swerve modules for driving (i.e, determining kS and kV)
     *
     */
    autoChooser.addOption(
        "Swerve Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runDriveCharacterizationVolts,
            drivetrain::getDriveCharacterizationVelocity,
            drivetrain::getDriveCharacterizationAcceleration));

    /************ Swerve Rotate Characterization ************
     *
     * useful for characterizing the swerve modules for rotating (i.e, determining kS and kV)
     *
     */
    autoChooser.addOption(
        "Swerve Rotate Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("rotate"),
            drivetrain::runRotateCharacterizationVolts,
            drivetrain::getRotateCharacterizationVelocity,
            drivetrain::getRotateCharacterizationAcceleration));

    /************ Distance Test ************
     *
     * used for empirically determining the wheel diameter
     *
     */
    Command distanceTestPathCommand = new PathPlannerAuto("DistanceTest");
    autoChooser.addOption("Distance Path", distanceTestPathCommand);

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    Command tuningCommand = new PathPlannerAuto("Tuning");
    autoChooser.addOption("Auto Tuning", tuningCommand);

    /************ Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.repeatingSequence(
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(2.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(-0.5, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(3.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(
                        () -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)))));

    /************ Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    autoChooser.addOption(
        "Swerve Rotation Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
            Commands.repeatingSequence(
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(0.1, 0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-0.1, 0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-0.1, -0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(
                        () -> drivetrain.drive(0.1, -0.1, 0.0, true, false), drivetrain)))));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  private void configureDrivetrainCommands() {
    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    // lock rotation to the nearest 180° while driving
    oi.getLock180Button()
        .whileTrue(
            new TeleopSwerve(
                drivetrain,
                oi::getTranslateX,
                oi::getTranslateY,
                () ->
                    (drivetrain.getPose().getRotation().getDegrees() > -90
                            && drivetrain.getPose().getRotation().getDegrees() < 90)
                        ? Rotation2d.fromDegrees(0.0)
                        : Rotation2d.fromDegrees(180.0)));

    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // slow-mode toggle
    oi.getTranslationSlowModeButton()
        .onTrue(Commands.runOnce(drivetrain::enableTranslationSlowMode, drivetrain));
    oi.getTranslationSlowModeButton()
        .onFalse(Commands.runOnce(drivetrain::disableTranslationSlowMode, drivetrain));
    oi.getRotationSlowModeButton()
        .onTrue(Commands.runOnce(drivetrain::enableRotationSlowMode, drivetrain));
    oi.getRotationSlowModeButton()
        .onFalse(Commands.runOnce(drivetrain::disableRotationSlowMode, drivetrain));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // reset pose based on vision
    oi.getResetPoseToVisionButton()
        .onTrue(
            Commands.runOnce(() -> drivetrain.resetPoseToVision(() -> vision.getBestRobotPose())));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // turbo
    oi.getTurboButton().onTrue(Commands.runOnce(drivetrain::enableTurbo, drivetrain));
    oi.getTurboButton().onFalse(Commands.runOnce(drivetrain::disableTurbo, drivetrain));
  }

  private void configureSubsystemCommands() {
    // FIXME: add commands for the subsystem
  }

  private void configureVisionCommands() {
    // enable/disable vision
    oi.getVisionIsEnabledSwitch().onTrue(Commands.runOnce(() -> vision.enable(true)));
    oi.getVisionIsEnabledSwitch()
        .onFalse(
            Commands.parallel(
                Commands.runOnce(() -> vision.enable(false), vision),
                Commands.runOnce(drivetrain::resetPoseRotationToGyro)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() != lastAlliance) {
      this.lastAlliance = alliance.get();
      this.drivetrain.updateAlliance(this.lastAlliance);
    }
  }
}
