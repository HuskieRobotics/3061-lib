/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * This class models the entire Robot. It extends from LoggedRobot instead of TimedRobot as required
 * to leverage AdvantageKit's logging features.
 */
public class Robot extends LoggedRobot {

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);

  /** Create a new Robot. */
  public Robot() {
    super(Constants.LOOP_PERIOD_SECS);
  }
  /**
   * This method is executed when the code first starts running on the robot and should be used for
   * any initialization code.
   */
  @Override
  public void robotInit() {
    // DO THIS FIRST
    Pathfinding.setPathfinder(new LocalADStarAK());

    final String GIT_DIRTY = "GitDirty";

    // from AdvantageKit Robot Configuration docs
    // (https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/START-LOGGING.md#robot-configuration)

    // Set a metadata value
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata(GIT_DIRTY, "All changes committed");
        break;
      case 1:
        Logger.recordMetadata(GIT_DIRTY, "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata(GIT_DIRTY, "Unknown");
        break;
    }

    switch (Constants.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));

        // Provide log data over the network, viewable in Advantage Scope.
        Logger.addDataReceiver(new NT4Publisher());

        LoggedPowerDistribution.getInstance();
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Run as fast as possible during replay
        setUseTiming(false);

        // Prompt the user for a file path on the command line (if not open in AdvantageScope)
        String path = LogFileUtil.findReplayLog();

        // Read log file for replay
        Logger.setReplaySource(new WPILOGReader(path));

        // Save replay results to a new log with the "_sim" suffix
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    // Start logging! No more data receivers, replay sources, or metadata values may be added.
    Logger.start();

    // Alternative logging of scheduled commands
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> Logger.recordOutput("Command initialized", command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> Logger.recordOutput("Command interrupted", command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> Logger.recordOutput("Command finished", command.getName()));

    // Logging of autonomous paths
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        pose -> Logger.recordOutput("PathFollowing/currentPose", pose));

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("PathFollowing/targetPose", pose));

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("PathFollowing/activePath", poses));

    // Invoke the factory method to create the RobotContainer singleton.
    robotContainer = RobotContainer.getInstance();
  }

  /**
   * This method is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleop and test.
   *
   * <p>This runs after the mode specific periodic methods, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /*
     * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
     * running already-scheduled commands, removing finished or interrupted commands, and running
     * subsystem periodic() methods. This must be called from the robot's periodic block in order
     * for anything in the Command-based framework to work.
     */
    CommandScheduler.getInstance().run();

    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());
  }

  /** This method is invoked periodically when the robot is in the disabled state. */
  @Override
  public void disabledPeriodic() {
    // check if the operator interface (e.g., joysticks) has changed
    robotContainer.updateOI();

    // check if the alliance color has changed based on the FMS data
    robotContainer.checkAllianceColor();
  }

  /**
   * This method is invoked at the start of the autonomous period. It schedules the autonomous
   * command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // check if the alliance color has changed based on the FMS data; the current alliance color is
    // not guaranteed to be correct until the start of autonomous
    robotContainer.checkAllianceColor();

    robotContainer.autonomousInit();
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This method is invoked at the start of the teleop period. */
  @Override
  public void teleopInit() {
    /*
     * This makes sure that the autonomous stops running when teleop starts running. If you want the
     * autonomous to continue until interrupted by another command, remove this line or comment it
     * out.
     */
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // check if the alliance color has changed based on the FMS data; if the robot power cycled
    // during a match, this would be the first opportunity to check the alliance color based on FMS
    // data.
    robotContainer.checkAllianceColor();

    robotContainer.teleopInit();
  }

  /** This method is invoked at the start of the test period. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
