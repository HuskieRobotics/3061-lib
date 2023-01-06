/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

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
    final String GIT_DIRTY = "GitDirty";

    // from AdvantageKit Robot Configuration docs
    // (https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/START-LOGGING.md#robot-configuration)

    Logger logger = Logger.getInstance();

    // Set a metadata value
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata(GIT_DIRTY, "All changes committed");
        break;
      case 1:
        logger.recordMetadata(GIT_DIRTY, "Uncomitted changes");
        break;
      default:
        logger.recordMetadata(GIT_DIRTY, "Unknown");
        break;
    }

    switch (Constants.getMode()) {
      case REAL:
        logger.addDataReceiver(new WPILOGWriter("/media/sda1"));

        // Provide log data over the network, viewable in Advantage Scope.
        logger.addDataReceiver(new NT4Publisher());

        LoggedPowerDistribution.getInstance();
        break;

      case SIM:
        logger.addDataReceiver(new WPILOGWriter(""));
        logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Run as fast as possible during replay
        setUseTiming(false);

        // Prompt the user for a file path on the command line (if not open in AdvantageScope)
        String path = LogFileUtil.findReplayLog();

        // Read log file for replay
        logger.setReplaySource(new WPILOGReader(path));

        // Save replay results to a new log with the "_sim" suffix
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    // Start logging! No more data receivers, replay sources, or metadata values may be added.
    logger.start();

    // Alternative logging of scheduled commands
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> Logger.getInstance().recordOutput("Command initialized", command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> Logger.getInstance().recordOutput("Command interrupted", command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> Logger.getInstance().recordOutput("Command finished", command.getName()));

    // Invoke the factory method to create the RobotContainer singleton.
    robotContainer = RobotContainer.getInstance();
  }

  /**
   * This method is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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

    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.updateOI();
  }

  /**
   * This method is invoked at the start of the autonomous period. It schedules the autonomous
   * command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This method is invoked at the start of the teleoperated period. */
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
  }

  /** This method is invoked at the start of the test period. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
