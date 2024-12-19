/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * This class models the entire Robot. It extends from LoggedRobot instead of TimedRobot as required
 * to leverage AdvantageKit's logging features.
 */
public class Robot extends LoggedRobot {
  private static final double LOW_BATTERY_VOLTAGE = 10.0;
  private static final double LOW_BATTERY_DISABLED_TIME = 1.5;

  private RobotContainer robotContainer;
  private Command autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;

  private final Timer disabledTimer = new Timer();

  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);

  /** Create a new Robot. */
  public Robot() {
    // Record metadata
    Logger.recordMetadata("Robot", Constants.getRobot().toString());
    Logger.recordMetadata("TuningMode", Boolean.toString(Constants.TUNING_MODE));
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }
  /**
   * This method is executed when the code first starts running on the robot and should be used for
   * any initialization code.
   */
  @Override
  public void robotInit() {
    // DO THIS FIRST
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (Boolean.TRUE.equals(active) ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    // Default to blue alliance in sim
    if (Constants.getMode() == Constants.Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

    // Logging of autonomous paths
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback(
        pose -> Logger.recordOutput("PathFollowing/currentPose", pose));

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("PathFollowing/targetPose", pose));

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("PathFollowing/activePath", poses.toArray(new Pose2d[0])));

    // Start timers
    disabledTimer.reset();
    disabledTimer.start();

    // Invoke the factory method to create the RobotContainer singleton.
    robotContainer = RobotContainer.getInstance();

    // Due to the nature of how Java works, the first run of a path following command could have a
    // significantly higher delay compared with subsequent runs, as all the classes involved will
    // need to be loaded. To help alleviate this issue, you can run a warmup command in the
    // background when code starts.
    // DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER
    PathfindingCommand.warmupCommand().schedule();
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
    Threads.setCurrentThreadPriority(true, 99);

    /*
     * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
     * running already-scheduled commands, removing finished or interrupted commands, and running
     * subsystem periodic() methods. This must be called from the robot's periodic block in order
     * for anything in the Command-based framework to work.
     */
    CommandScheduler.getInstance().run();

    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    // Update low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() < LOW_BATTERY_VOLTAGE
        && disabledTimer.hasElapsed(LOW_BATTERY_DISABLED_TIME)) {
      LEDs.getInstance().requestState(LEDs.States.LOW_BATTERY);
      lowBatteryAlert.set(true);
    }

    // Print auto duration
    if (autonomousCommand != null && !autonomousCommand.isScheduled() && !autoMessagePrinted) {
      if (DriverStation.isAutonomousEnabled()) {
        System.out.println(
            String.format(
                "*** Auto finished in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
      } else {
        System.out.println(
            String.format(
                "*** Auto cancelled in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
      }
      autoMessagePrinted = true;
    }

    robotContainer.periodic();

    Threads.setCurrentThreadPriority(true, 10);
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

    autoStart = Timer.getFPGATimestamp();
    autoMessagePrinted = false;
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    robotContainer.autonomousInit();
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

    robotContainer.teleopInit();
  }

  /** This method is invoked at the start of the test period. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
