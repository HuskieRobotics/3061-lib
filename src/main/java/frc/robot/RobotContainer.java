// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainIO;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionIO;
import frc.robot.Constants.Mode;
import frc.robot.commands.CharacterizationCommands;
import frc.robot.commands.TeleopSwerve;
import frc.robot.configs.ArtemisRobotConfig;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.configs.NewPracticeRobotConfig;
import frc.robot.configs.PracticeBoardConfig;
import frc.robot.configs.VisionTestPlatformConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.subsystem.Subsystem;
import frc.robot.subsystems.subsystem.SubsystemIO;
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

  private Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);
  private static final String LAYOUT_FILE_MISSING =
      "Could not find the specified AprilTags layout file";
  private Alert layoutFileMissingAlert = new Alert(LAYOUT_FILE_MISSING, AlertType.kError);

  private Alert tuningAlert = new Alert("Tuning mode enabled", AlertType.kInfo);

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
        case ROBOT_PRACTICE_BOARD:
          {
            createPracticeBoardSubsystems();
            break;
          }
        case ROBOT_VISION_TEST_PLATFORM:
          {
            createVisionTestPlatformSubsystems();
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

    // Alert when tuning
    if (Constants.TUNING_MODE) {
      this.tuningAlert.set(true);
    }
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
      case ROBOT_VISION_TEST_PLATFORM:
        config = new VisionTestPlatformConfig();
        break;
      default:
        break;
    }
  }

  private void createCTRESubsystems() {
    drivetrain = new Drivetrain(new DrivetrainIOCTRE());

    String[] cameraNames = config.getCameraNames();
    VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIO() {};
    }
    vision = new Vision(visionIOs);

    // FIXME: create the hardware-specific subsystem class
    subsystem = new Subsystem(new SubsystemIO() {});
  }

  private void createCTRESimSubsystems() {
    DrivetrainIO drivetrainIO = new DrivetrainIOCTRE();
    drivetrain = new Drivetrain(drivetrainIO);

    String[] cameraNames = config.getCameraNames();
    VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIO() {};
    }
    vision = new Vision(visionIOs);

    // FIXME: create the hardware-specific subsystem class
  }

  private void createPracticeBoardSubsystems() {
    // change the following to connect the subsystem being tested to actual hardware
    drivetrain = new Drivetrain(new DrivetrainIO() {});
    vision = new Vision(new VisionIO[] {new VisionIO() {}});
  }

  private void createVisionTestPlatformSubsystems() {
    // change the following to connect the subsystem being tested to actual hardware
    drivetrain = new Drivetrain(new DrivetrainIO() {});

    String[] cameraNames = config.getCameraNames();
    VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIO() {};
    }
    vision = new Vision(visionIOs);
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

    // clear the list of composed commands since we are about to rebind them to potentially new
    // triggers
    CommandScheduler.getInstance().clearComposedCommands();
    configureButtonBindings();
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
                Commands.runOnce(() -> subsystem.setMotorVoltage(0)),
                new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Event Markers
    new EventTrigger("Marker").onTrue(Commands.print("reached event marker"));
    new EventTrigger("ZoneMarker").onTrue(Commands.print("entered zone"));
    new EventTrigger("ZoneMarker").onFalse(Commands.print("left zone"));

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
                pathFileMissingAlert.setText("Could not find the specified path file: Start Point");
                pathFileMissingAlert.set(true);
              }
            },
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Distance Test ************
     *
     * used for empirically determining the wheel radius
     *
     */
    autoChooser.addOption("Distance Test Slow", createTuningAutoPath("DistanceTestSlow", true));
    autoChooser.addOption("Distance Test Med", createTuningAutoPath("DistanceTestMed", true));
    autoChooser.addOption("Distance Test Fast", createTuningAutoPath("DistanceTestFast", true));

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    autoChooser.addOption("Rotation Test Slow", createTuningAutoPath("RotationTestSlow", false));
    autoChooser.addOption("Rotation Test Fast", createTuningAutoPath("RotationTestFast", false));

    autoChooser.addOption("Oval Test Slow", createTuningAutoPath("OvalTestSlow", false));
    autoChooser.addOption("Oval Test Fast", createTuningAutoPath("OvalTestFast", false));

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

    /************ Drive Wheel Radius Characterization ************
     *
     * useful for characterizing the drive wheel Radius
     *
     */
    autoChooser.addOption( // start by driving slowing in a circle to align wheels
        "Drive Wheel Radius Characterization",
        CharacterizationCommands.wheelRadiusCharacterization(drivetrain)
            .withName("Drive Wheel Radius Characterization"));
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
        .whileTrue(Commands.run(drivetrain::holdXstance, drivetrain).withName("hold x-stance"));

    oi.getSysIdDynamicForward().whileTrue(SysIdRoutineChooser.getInstance().getDynamicForward());
    oi.getSysIdDynamicReverse().whileTrue(SysIdRoutineChooser.getInstance().getDynamicReverse());
    oi.getSysIdQuasistaticForward()
        .whileTrue(SysIdRoutineChooser.getInstance().getQuasistaticForward());
    oi.getSysIdQuasistaticReverse()
        .whileTrue(SysIdRoutineChooser.getInstance().getQuasistaticReverse());
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
            Commands.runOnce(() -> vision.enable(false), vision)
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

  public void periodic() {
    // add robot-wide periodic code here
  }

  public void autonomousInit() {
    // add robot-wide code here that will be executed when autonomous starts
  }

  public void teleopInit() {
    // check if the alliance color has changed based on the FMS data; if the robot power cycled
    // during a match, this would be the first opportunity to check the alliance color based on FMS
    // data.
    this.checkAllianceColor();
  }
}
