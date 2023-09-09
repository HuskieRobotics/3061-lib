// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.FieldRegionConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2Phoenix6;
import frc.lib.team3061.pneumatics.Pneumatics;
import frc.lib.team3061.pneumatics.PneumaticsIO;
import frc.lib.team3061.pneumatics.PneumaticsIORev;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFXPhoenix6;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.TeleopSwerve;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.configs.MK4IRobotConfig;
import frc.robot.configs.NovaRobotConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.subsystem.Subsystem;
import frc.robot.subsystems.subsystem.SubsystemIO;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private Alliance lastAlliance = DriverStation.Alliance.Invalid;
  private Vision vision;
  private Subsystem subsystem;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  private final Map<String, Command> autoEventMap = new HashMap<>();

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
      int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
      int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
      int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
      double[] steerOffsets = config.getSwerveSteerOffsets();
      SwerveModule flModule =
          new SwerveModule(
              new SwerveModuleIOTalonFXPhoenix6(
                  0,
                  driveMotorCANIDs[0],
                  steerMotorCANDIDs[0],
                  steerEncoderCANDIDs[0],
                  steerOffsets[0]),
              0,
              config.getRobotMaxVelocity());

      SwerveModule frModule =
          new SwerveModule(
              new SwerveModuleIOTalonFXPhoenix6(
                  1,
                  driveMotorCANIDs[1],
                  steerMotorCANDIDs[1],
                  steerEncoderCANDIDs[1],
                  steerOffsets[1]),
              1,
              config.getRobotMaxVelocity());

      SwerveModule blModule =
          new SwerveModule(
              new SwerveModuleIOTalonFXPhoenix6(
                  2,
                  driveMotorCANIDs[2],
                  steerMotorCANDIDs[2],
                  steerEncoderCANDIDs[2],
                  steerOffsets[2]),
              2,
              config.getRobotMaxVelocity());

      SwerveModule brModule =
          new SwerveModule(
              new SwerveModuleIOTalonFXPhoenix6(
                  3,
                  driveMotorCANIDs[3],
                  steerMotorCANDIDs[3],
                  steerEncoderCANDIDs[3],
                  steerOffsets[3]),
              3,
              config.getRobotMaxVelocity());
      switch (Constants.getRobot()) {
        case ROBOT_DEFAULT:
        case ROBOT_2023_NOVA:
        case ROBOT_2023_MK4I:
          {
            GyroIO gyro = new GyroIOPigeon2Phoenix6(config.getGyroCANID());

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);

            String[] cameraNames = config.getCameraNames();
            VisionIO[] visionIOs = new VisionIO[cameraNames.length];
            for (int i = 0; i < visionIOs.length; i++) {
              visionIOs[i] = new VisionIOPhotonVision(cameraNames[i]);
            }
            vision = new Vision(visionIOs);
            // subsystem = new Subsystem(new SubsystemIOTalonFX());
            subsystem = new Subsystem(new SubsystemIO() {});

            if (Constants.getRobot() == Constants.RobotType.ROBOT_DEFAULT) {
              new Pneumatics(new PneumaticsIORev());
            }
            break;
          }
        case ROBOT_SIMBOT:
          {
            GyroIO gyro = new GyroIOPigeon2Phoenix6(config.getGyroCANID());

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            new Pneumatics(new PneumaticsIO() {});
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
            subsystem = new Subsystem(new SubsystemIO() {});

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, config.getRobotMaxVelocity());

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, config.getRobotMaxVelocity());

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, config.getRobotMaxVelocity());

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, config.getRobotMaxVelocity());
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);

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
      case ROBOT_2023_NOVA:
      case ROBOT_SIMBOT:
        config = new NovaRobotConfig();
        break;
      case ROBOT_2023_MK4I:
        config = new MK4IRobotConfig();
        break;
    }
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   */
  private void constructField() {
    Field2d.getInstance()
        .setRegions(
            new Region2d[] {
              COMMUNITY_REGION_1,
              COMMUNITY_REGION_2,
              COMMUNITY_REGION_3,
              LOADING_ZONE_REGION_1,
              LOADING_ZONE_REGION_2,
              FIELD_ZONE_REGION_1,
              FIELD_ZONE_REGION_2,
              FIELD_ZONE_REGION_3,
              FIELD_ZONE_REGION_4
            });

    COMMUNITY_REGION_1.addNeighbor(COMMUNITY_REGION_2, COMMUNITY_REGION_1_2_TRANSITION_POINT);
    COMMUNITY_REGION_2.addNeighbor(COMMUNITY_REGION_1, COMMUNITY_REGION_2_1_TRANSITION_POINT);
    COMMUNITY_REGION_1.addNeighbor(COMMUNITY_REGION_3, COMMUNITY_REGION_1_3_TRANSITION_POINT);
    COMMUNITY_REGION_3.addNeighbor(COMMUNITY_REGION_1, COMMUNITY_REGION_3_1_TRANSITION_POINT);
    COMMUNITY_REGION_2.addNeighbor(FIELD_ZONE_REGION_1, COMMUNITY_2_TO_FIELD_1_TRANSITION_POINT);
    COMMUNITY_REGION_3.addNeighbor(FIELD_ZONE_REGION_2, COMMUNITY_3_TO_FIELD_2_TRANSITION_POINT);

    LOADING_ZONE_REGION_1.addNeighbor(
        LOADING_ZONE_REGION_2, LOADING_ZONE_REGION_1_2_TRANSITION_POINT);
    LOADING_ZONE_REGION_2.addNeighbor(
        LOADING_ZONE_REGION_1, LOADING_ZONE_REGION_2_1_TRANSITION_POINT);
    LOADING_ZONE_REGION_2.addNeighbor(FIELD_ZONE_REGION_4, LOADING_2_TO_FIELD_4_TRANSITION_POINT);

    FIELD_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_2, FIELD_ZONE_REGION_1_2_TRANSITION_POINT);
    FIELD_ZONE_REGION_2.addNeighbor(FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_2_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_3, FIELD_ZONE_REGION_1_3_TRANSITION_POINT);
    FIELD_ZONE_REGION_3.addNeighbor(FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_3_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_4, FIELD_ZONE_REGION_1_4_TRANSITION_POINT);
    FIELD_ZONE_REGION_4.addNeighbor(FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_4_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(COMMUNITY_REGION_2, FIELD_1_TO_COMMUNITY_2_TRANSITION_POINT);
    FIELD_ZONE_REGION_2.addNeighbor(COMMUNITY_REGION_3, FIELD_2_TO_COMMUNITY_3_TRANSITION_POINT);
    FIELD_ZONE_REGION_3.addNeighbor(LOADING_ZONE_REGION_1, FIELD_3_TO_LOADING_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_4.addNeighbor(LOADING_ZONE_REGION_2, FIELD_4_TO_LOADING_2_TRANSITION_POINT);
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
    autoEventMap.put("event1", Commands.print("passed marker 1"));
    autoEventMap.put("event2", Commands.print("passed marker 2"));

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /************ Test Path ************
     *
     * demonstration of PathPlanner path group with event markers
     *
     */
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "TestPath", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command autoTest =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(0), drivetrain, true, true),
                auto1Paths.get(0).getMarkers(),
                autoEventMap),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(1), drivetrain, false, true),
                auto1Paths.get(1).getMarkers(),
                autoEventMap));
    autoChooser.addOption("Test Path", autoTest);

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    PathPlannerTrajectory startPointPath =
        PathPlanner.loadPath(
            "StartPoint", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command startPoint =
        Commands.runOnce(
            () -> drivetrain.resetOdometry(startPointPath.getInitialState()), drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Drive Characterization ************
     *
     * useful for characterizing the drivetrain (i.e, determining kS and kV)
     *
     */
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));

    /************ Distance Test ************
     *
     * used for empirically determining the wheel diameter
     *
     */
    PathPlannerTrajectory distanceTestPath = PathPlanner.loadPath("DistanceTest", 2, 2);
    Command distanceTestPathCommand = new FollowPath(distanceTestPath, drivetrain, true, true);
    autoChooser.addOption("Distance Path", distanceTestPathCommand);

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    PathPlannerTrajectory tuningPath = PathPlanner.loadPath("Tuning", 2.0, 3.0);
    Command tuningCommand = new FollowPath(tuningPath, drivetrain, true, true);
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
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0, false, false), drivetrain))));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());

    // enable the path planner server so we can update paths without redeploying code
    if (TUNING_MODE) {
      PathPlannerServer.startServer(3061);
    }
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
        .onTrue(
            new RotateToAngle(
                drivetrain,
                oi::getTranslateX,
                oi::getTranslateY,
                () ->
                    (drivetrain.getPose().getRotation().getDegrees() > -90
                            && drivetrain.getPose().getRotation().getDegrees() < 90)
                        ? 0.0
                        : 180.0));

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
    if (DriverStation.getAlliance() != lastAlliance) {
      lastAlliance = DriverStation.getAlliance();
      vision.updateAlliance(lastAlliance);
      Field2d.getInstance().updateAlliance(lastAlliance);
    }
  }

  public void autonomousInit() {
    // when the LED subsystem is pulled in, we will change the LEDs here
  }

  public void teleopInit() {
    // when the LED subsystem is pulled in, we will change the LEDs here
  }

  public void disabledPeriodic() {
    // when the LED subsystem is pulled in, we will change the LEDs here
  }
}
