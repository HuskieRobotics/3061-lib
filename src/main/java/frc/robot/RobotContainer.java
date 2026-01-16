// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.RobotConfig.CameraConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrainIOXRP;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainIO;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainIOCTRE;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIONorthstar;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.ArmCommandFactory;
import frc.robot.commands.AutonomousCommandsFactory;
import frc.robot.commands.CrossSubsystemsCommandsFactory;
import frc.robot.commands.DifferentialDrivetrainCommandFactory;
import frc.robot.commands.ElevatorCommandsFactory;
import frc.robot.commands.SwerveDrivetrainCommandFactory;
import frc.robot.configs.CalypsoRobotConfig;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.configs.NewPracticeRobotConfig;
import frc.robot.configs.NorthstarTestPlatformConfig;
import frc.robot.configs.PracticeBoardConfig;
import frc.robot.configs.VisionTestPlatformConfig;
import frc.robot.configs.XRPRobotConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.arm.ArmIOXRP;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.visualizations.RobotVisualization;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
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
  private SwerveDrivetrain swerveDrivetrain;
  private DifferentialDrivetrain differentialDrivetrain;
  private Alliance lastAlliance = Field2d.getInstance().getAlliance();
  private Vision vision;
  private Arm arm;
  private Elevator elevator;
  private Manipulator manipulator;
  private Shooter shooter;
  private RobotVisualization visualization;

  private final LoggedNetworkNumber endgameAlert1 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);
  private final LoggedNetworkNumber endgameAlert2 =
      new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

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

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {

      switch (Constants.getRobot()) {
        case ROBOT_DEFAULT, ROBOT_COMPETITION:
          {
            createCTRESubsystems();
            break;
          }
        case ROBOT_PRACTICE:
          {
            createCTREPracticeBotSubsystems();
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
        case ROBOT_NORTHSTAR_TEST_PLATFORM:
          {
            createNorthstarTestPlatformSubsystems();
            break;
          }
        case ROBOT_XRP:
          {
            createXRPSubsystems();
            break;
          }
        default:
          break;
      }

    } else {
      swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});

      CameraConfig[] cameraConfigs = config.getCameraConfigs();
      VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
      for (int i = 0; i < visionIOs.length; i++) {
        visionIOs[i] = new VisionIO() {};
      }
      vision = new Vision(visionIOs);

      // FIXME: initialize other subsystems
      arm = new Arm(new ArmIO() {});
      elevator = new Elevator(new ElevatorIO() {});
      manipulator = new Manipulator(new ManipulatorIO() {});
      shooter = new Shooter(new ShooterIO() {});
      visualization = new RobotVisualization(elevator);
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    // register autonomous commands
    if (RobotConfig.getInstance().getDrivetrainType() == RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
      AutonomousCommandsFactory.getInstance().configureAutoCommands(differentialDrivetrain, vision);
    } else if (RobotConfig.getInstance().getDrivetrainType()
        == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
      AutonomousCommandsFactory.getInstance().configureAutoCommands(swerveDrivetrain, vision);
    }

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
      case ROBOT_PRACTICE, ROBOT_SIMBOT:
        config = new NewPracticeRobotConfig();
        break;
      case ROBOT_COMPETITION:
        config = new CalypsoRobotConfig();
        break;
      case ROBOT_PRACTICE_BOARD:
        config = new PracticeBoardConfig();
        break;
      case ROBOT_VISION_TEST_PLATFORM:
        config = new VisionTestPlatformConfig();
        break;
      case ROBOT_NORTHSTAR_TEST_PLATFORM:
        config = new NorthstarTestPlatformConfig();
        break;
      case ROBOT_XRP:
        config = new XRPRobotConfig();
        break;
      default:
        break;
    }
  }

  private void createCTRESubsystems() {
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIOCTRE());

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);

      layoutFileMissingAlert.setText(
          LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      layoutFileMissingAlert.set(true);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIOPhotonVision(cameraConfigs[i].id(), layout);
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    arm = new Arm(new ArmIOTalonFX());
    elevator = new Elevator(new ElevatorIOTalonFX());
    manipulator = new Manipulator(new ManipulatorIOTalonFX());
    shooter = new Shooter(new ShooterIOTalonFX());
    visualization = new RobotVisualization(elevator);
  }

  private void createCTREPracticeBotSubsystems() {
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIOCTRE());

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);

      layoutFileMissingAlert.setText(
          LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      layoutFileMissingAlert.set(true);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIONorthstar(layout, cameraConfigs[i]);
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    arm = new Arm(new ArmIO() {});
    elevator = new Elevator(new ElevatorIO() {});
    manipulator = new Manipulator(new ManipulatorIO() {});
    shooter = new Shooter(new ShooterIO() {});
    visualization = new RobotVisualization(elevator);
  }

  private void createCTRESimSubsystems() {
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIOCTRE());

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      layoutFileMissingAlert.setText(
          LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      layoutFileMissingAlert.set(true);
    }

    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] =
          new VisionIOSim(
              cameraConfigs[i].id(),
              layout,
              swerveDrivetrain::getPose,
              cameraConfigs[i].robotToCameraTransform());
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    arm = new Arm(new ArmIOTalonFX());
    elevator = new Elevator(new ElevatorIOTalonFX());
    manipulator = new Manipulator(new ManipulatorIOTalonFX());
    shooter = new Shooter(new ShooterIOTalonFX());
    visualization = new RobotVisualization(elevator);
  }

  private void createXRPSubsystems() {
    differentialDrivetrain = new DifferentialDrivetrain(new DifferentialDrivetrainIOXRP());
    vision = new Vision(new VisionIO[] {});

    arm = new Arm(new ArmIOXRP());
    elevator = new Elevator(new ElevatorIO() {});
    manipulator = new Manipulator(new ManipulatorIO() {});
    shooter = new Shooter(new ShooterIO() {});
    visualization = new RobotVisualization(elevator);
  }

  private void createPracticeBoardSubsystems() {
    // change the following to connect the subsystem being tested to actual hardware
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});
    vision = new Vision(new VisionIO[] {new VisionIO() {}});

    // FIXME: initialize other subsystems
    arm = new Arm(new ArmIO() {});
    elevator = new Elevator(new ElevatorIO() {});
    manipulator = new Manipulator(new ManipulatorIO() {});
    shooter = new Shooter(new ShooterIO() {});
    visualization = new RobotVisualization(elevator);
  }

  private void createVisionTestPlatformSubsystems() {
    // change the following to connect the subsystem being tested to actual hardware
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);

      layoutFileMissingAlert.setText(
          LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      layoutFileMissingAlert.set(true);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIOPhotonVision(cameraConfigs[i].id(), layout);
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    arm = new Arm(new ArmIO() {});
    elevator = new Elevator(new ElevatorIO() {});
    manipulator = new Manipulator(new ManipulatorIO() {});
    shooter = new Shooter(new ShooterIO() {});
  }

  private void createNorthstarTestPlatformSubsystems() {
    // change the following to connect the subsystem being tested to actual hardware
    swerveDrivetrain = new SwerveDrivetrain(new SwerveDrivetrainIO() {});

    CameraConfig[] cameraConfigs = config.getCameraConfigs();
    VisionIO[] visionIOs = new VisionIO[cameraConfigs.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);

      layoutFileMissingAlert.setText(
          LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      layoutFileMissingAlert.set(true);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIONorthstar(layout, cameraConfigs[i]);
    }
    vision = new Vision(visionIOs);

    // FIXME: initialize other subsystems
    arm = new Arm(new ArmIO() {});
    elevator = new Elevator(new ElevatorIO() {});
    manipulator = new Manipulator(new ManipulatorIO() {});
    shooter = new Shooter(new ShooterIO() {});
    visualization = new RobotVisualization(elevator);
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

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    configureDrivetrainCommands();
    configureVisionCommands();

    // register commands for other subsystems
    ArmCommandFactory.registerCommands(oi, arm);
    ElevatorCommandsFactory.registerCommands(oi, elevator);

    if (RobotConfig.getInstance().getDrivetrainType() == RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
      CrossSubsystemsCommandsFactory.registerCommands(oi, differentialDrivetrain, vision, arm);
    } else if (RobotConfig.getInstance().getDrivetrainType()
        == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
      CrossSubsystemsCommandsFactory.registerCommands(
          oi, swerveDrivetrain, vision, arm, elevator, manipulator, shooter);
    }

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
  }

  private void configureDrivetrainCommands() {
    if (RobotConfig.getInstance().getDrivetrainType() == RobotConfig.DRIVETRAIN_TYPE.DIFFERENTIAL) {
      DifferentialDrivetrainCommandFactory.registerCommands(oi, differentialDrivetrain);
    } else if (RobotConfig.getInstance().getDrivetrainType()
        == RobotConfig.DRIVETRAIN_TYPE.SWERVE) {
      SwerveDrivetrainCommandFactory.registerCommands(oi, swerveDrivetrain, vision);
    }
  }

  private void configureVisionCommands() {
    // enable/disable vision
    oi.getVisionIsEnabledTrigger()
        .onTrue(
            Commands.runOnce(() -> vision.enable(true))
                .ignoringDisable(true)
                .withName("enable vision"));
    oi.getVisionIsEnabledTrigger()
        .onFalse(
            Commands.runOnce(() -> vision.enable(false))
                .ignoringDisable(true)
                .withName("disable vision"));
  }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() != lastAlliance) {
      this.lastAlliance = alliance.get();
      Field2d.getInstance().updateAlliance(this.lastAlliance);
    }
  }

  public void periodic() {
    // add robot-wide periodic code here
    visualization.update();
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
