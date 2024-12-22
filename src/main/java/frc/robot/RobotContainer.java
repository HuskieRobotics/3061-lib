// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainIO;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.WheelDiameterCharacterization;
import frc.robot.configs.ArtemisRobotConfig;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.configs.NewPracticeRobotConfig;
import frc.robot.configs.PracticeBoardConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.subsystem.Subsystem;
import frc.robot.subsystems.subsystem.SubsystemIO;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
  private Alliance lastAlliance = Field2d.getInstance().getAlliance();
  private Vision vision;
  private Subsystem subsystem;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

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
        case ROBOT_PRACTICE_BOARD:
          {
            createPracticeBoardSubsystem();
            break;
          }
        case ROBOT_DEFAULT, ROBOT_PRACTICE, ROBOT_COMPETITION:
          {
            createCTRESubsystems();
            break;
          }
        case ROBOT_SIMBOT:
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
      case ROBOT_PRACTICE:
        config = new NewPracticeRobotConfig();
        break;
      case ROBOT_COMPETITION, ROBOT_SIMBOT:
        config = new ArtemisRobotConfig();
        break;
      case ROBOT_PRACTICE_BOARD:
        config = new PracticeBoardConfig();
        break;
      default:
        break;
    }
  }

  private void createCTRESubsystems() {
    drivetrain = new Drivetrain(new DrivetrainIOCTRE());

    String[] cameraNames = config.getCameraNames();
    VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIOPhotonVision(cameraNames[i], layout);
    }
    vision = new Vision(visionIOs);

    // FIXME: create the hardware-specific subsystem class
    subsystem = new Subsystem(new SubsystemIO() {});
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

  private void createPracticeBoardSubsystem() {
    // change the following to connect the subsystem being tested to actual hardware
    drivetrain = new Drivetrain(new DrivetrainIO() {});
    vision = new Vision(new VisionIO[] {new VisionIO() {}});
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
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
            Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                .withTimeout(1));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                    .withTimeout(0.5),
                Commands.waitSeconds(0.25),
                Commands.run(() -> LEDs.getInstance().requestState(LEDs.States.ENDGAME_ALERT))
                    .withTimeout(0.5)));

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
    NamedCommands.registerCommand(
        "EnableRotationOverride", Commands.runOnce(drivetrain::enableRotationOverride));
    NamedCommands.registerCommand(
        "DisableRotationOverride", Commands.runOnce(drivetrain::disableRotationOverride));

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    Command startPoint =
        Commands.runOnce(
            () -> {
              try {
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
              } catch (Exception e) {
                // FIXME: generate an alert about the missing path file;
                System.out.println("Path file not found: Start Point");
              }
            },
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Distance Test ************
     *
     * used for empirically determining the wheel diameter
     *
     */
    autoChooser.addOption("Distance Test Slow", createTuningAutoPath("DistanceTestSlow", true));
    autoChooser.addOption("Distance Test Med", createTuningAutoPath("DistanceTestMed", true));
    autoChooser.addOption("Distance Test Fast", createTuningAutoPath("DistanceTestFast", true));

    autoChooser.addOption("Rotation Test Slow", createTuningAutoPath("RotationTestSlow", false));
    autoChooser.addOption("Rotation Test Fast", createTuningAutoPath("RotationTestFast", false));

    autoChooser.addOption("Oval Test Slow", createTuningAutoPath("OvalTestSlow", false));
    autoChooser.addOption("Oval Test Fast", createTuningAutoPath("OvalTestFast", false));

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

    /************ Drive Wheel Diameter Characterization ************
     *
     * useful for characterizing the drive wheel diameter
     *
     */
    autoChooser.addOption( // start by driving slowing in a circle to align wheels
        "Drive Wheel Diameter Characterization",
        Commands.sequence(
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(0.0, 0.0, 0.1, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.25),
                    Commands.run(() -> drivetrain.drive(0.0, 0.0, 0.0, true, false), drivetrain)),
                new WheelDiameterCharacterization(drivetrain))
            .withName("Drive Wheel Diameter Characterization"));
  }

  private Command createTuningAutoPath(String autoName, boolean measureDistance) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::captureInitialConditions),
        new PathPlannerAuto(autoName),
        Commands.runOnce(() -> drivetrain.captureFinalConditions(autoName, measureDistance)));
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
                            : Rotation2d.fromDegrees(180.0))
                .withName("lock 180"));

    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                    Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                    Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                    drivetrain::getFieldRelative)
                .withName("toggle field relative"));

    // slow-mode toggle
    oi.getTranslationSlowModeButton()
        .onTrue(
            Commands.runOnce(drivetrain::enableTranslationSlowMode, drivetrain)
                .withName("enable translation slow mode"));
    oi.getTranslationSlowModeButton()
        .onFalse(
            Commands.runOnce(drivetrain::disableTranslationSlowMode, drivetrain)
                .withName("disable translation slow mode"));
    oi.getRotationSlowModeButton()
        .onTrue(
            Commands.runOnce(drivetrain::enableRotationSlowMode, drivetrain)
                .withName("enable rotation slow mode"));
    oi.getRotationSlowModeButton()
        .onFalse(
            Commands.runOnce(drivetrain::disableRotationSlowMode, drivetrain)
                .withName("disable rotation slow mode"));

    // reset gyro to 0 degrees
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain).withName("zero gyro"));

    // reset pose based on vision
    oi.getResetPoseToVisionButton()
        .onTrue(
            Commands.repeatingSequence(Commands.none())
                .until(() -> vision.getBestRobotPose() != null)
                .andThen(
                    Commands.runOnce(
                        () -> drivetrain.resetPoseToVision(() -> vision.getBestRobotPose())))
                .ignoringDisable(true)
                .withName("reset pose to vision"));

    // x-stance
    oi.getXStanceButton()
        .onTrue(
            Commands.runOnce(drivetrain::enableXstance, drivetrain).withName("enable x-stance"));
    oi.getXStanceButton()
        .onFalse(
            Commands.runOnce(drivetrain::disableXstance, drivetrain).withName("disable x-stance"));

    // turbo
    oi.getTurboButton()
        .onTrue(Commands.runOnce(drivetrain::enableTurbo, drivetrain).withName("enable turbo"));
    oi.getTurboButton()
        .onFalse(Commands.runOnce(drivetrain::disableTurbo, drivetrain).withName("disable turbo"));
  }

  private void configureSubsystemCommands() {
    // FIXME: add commands for the subsystem
  }

  private void configureVisionCommands() {
    // enable/disable vision
    oi.getVisionIsEnabledSwitch()
        .onTrue(
            Commands.runOnce(() -> vision.enable(true))
                .ignoringDisable(true)
                .withName("enable vision"));
    oi.getVisionIsEnabledSwitch()
        .onFalse(
            Commands.parallel(
                    Commands.runOnce(() -> vision.enable(false), vision),
                    Commands.runOnce(drivetrain::resetPoseRotationToGyro))
                .ignoringDisable(true)
                .withName("disable vision"));
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
      Field2d.getInstance().updateAlliance(this.lastAlliance);
    }
  }

  public void periodic() {}

  public void autonomousInit() {}

  public void teleopInit() {
    // check if the alliance color has changed based on the FMS data; if the robot power cycled
    // during a match, this would be the first opportunity to check the alliance color based on FMS
    // data.
    this.checkAllianceColor();

    // ensure that x-stance is disabled at the start of teleop as there is a possibility if the
    //  auto command is interrupted, we could still be in x-stance
    drivetrain.disableXstance();
  }
}
