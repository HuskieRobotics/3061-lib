/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import static frc.robot.Constants.TUNING_MODE;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.SystemTimeValidReader;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutonomousCommandsFactory;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

@java.lang.SuppressWarnings({"java:S1192", "java:S106"})

/**
 * This class models the entire Robot. It extends from LoggedRobot instead of TimedRobot as required
 * to leverage AdvantageKit's logging features.
 */
public class Robot extends LoggedRobot {
  private static final double LOW_BATTERY_VOLTAGE = 10.0;
  private static final double LOW_BATTERY_DISABLED_TIME = 1.5;
  private static final double CAN_ERROR_TIME_THRESHOLD = 0.5; // Seconds to disable alert
  private static final double CANIVORE_ERROR_TIME_THRESHOLD = 0.5;

  private RobotContainer robotContainer;
  private Command autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private CANBus canivoreBus;

  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();

  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert canivoreErrorAlert =
      new Alert("CANivore error detected, robot may not be controllable.", AlertType.kError);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.kError);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.kWarning);
  private final Alert jitAlert =
      new Alert("Please wait to enable, JITing in progress.", AlertType.kWarning);

  private final Alert noAutoSelectedAlert =
      new Alert("No auto selected: please select an auto", AlertType.kWarning);
  /** Create a new Robot. */
  public Robot() {
    // start code loading LED animation
    LEDs.getInstance();

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
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1"));
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
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

    // start Elastic Dashboard server
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // DO THIS FIRST
    Pathfinding.setPathfinder(new LocalADStarAK());

    // Start system time valid reader
    SystemTimeValidReader.start();

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
      DriverStationSim.notifyNewData();
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
    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    // adjust loop overrun warning timeout
    if (!Constants.TUNING_MODE) {
      try {
        Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
        watchdogField.setAccessible(true);
        Watchdog watchdog = (Watchdog) watchdogField.get(this);
        watchdog.setTimeout(0.2);
      } catch (Exception e) {
        DriverStation.reportWarning("Failed to disable loop overrun warnings", false);
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    // create the CANivore bus object
    this.canivoreBus = RobotConfig.getInstance().getCANBus();

    // Due to the nature of how Java works, the first run of a path following command could have a
    // significantly higher delay compared with subsequent runs, as all the classes involved will
    // need to be loaded. To help alleviate this issue, you can run a warmup command in the
    // background when code starts.
    // DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    if (!TUNING_MODE) {
      Threads.setCurrentThreadPriority(true, 10);
    }
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
    // Refresh all Phoenix signals
    LoggedTracer.reset();
    Phoenix6Util.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    /*
     * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
     * running already-scheduled commands, removing finished or interrupted commands, and running
     * subsystem periodic() methods. This must be called from the robot's periodic block in order
     * for anything in the Command-based framework to work.
     */
    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");

    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    Logger.recordOutput("CANStatus/OffCount", canStatus.busOffCount);
    Logger.recordOutput("CANStatus/TxFullCount", canStatus.txFullCount);
    Logger.recordOutput("CANStatus/ReceiveErrorCount", canStatus.receiveErrorCount);
    Logger.recordOutput("CANStatus/TransmitErrorCount", canStatus.transmitErrorCount);

    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD)
            && canInitialErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD));

    // Log CANivore status
    if (Constants.getMode() == Mode.REAL) {
      var canivoreStatus = this.canivoreBus.getStatus();
      Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.Status.getName());
      Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.BusUtilization);
      Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.BusOffCount);
      Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.TxFullCount);
      Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.REC);
      Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.TEC);
      if (!canivoreStatus.Status.isOK() || canivoreStatus.REC > 0 || canivoreStatus.TEC > 0) {
        canivoreErrorTimer.restart();
      }
      canivoreErrorAlert.set(
          !canivoreErrorTimer.hasElapsed(CANIVORE_ERROR_TIME_THRESHOLD)
              && canInitialErrorTimer.hasElapsed(CAN_ERROR_TIME_THRESHOLD));
    }

    // Update low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() < LOW_BATTERY_VOLTAGE
        && disabledTimer.hasElapsed(LOW_BATTERY_DISABLED_TIME)) {
      LEDs.getInstance().requestState(LEDs.States.LOW_BATTERY);
      lowBatteryAlert.set(true);
    }

    // JIT alert
    jitAlert.set(Timer.getTimestamp() < 45.0);

    // Print auto duration
    if (autonomousCommand != null && !autonomousCommand.isScheduled() && !autoMessagePrinted) {
      if (DriverStation.isAutonomousEnabled()) {
        System.out.println(
            String.format("*** Auto finished in %.2f secs ***", Timer.getTimestamp() - autoStart));
      } else {
        System.out.println(
            String.format("*** Auto cancelled in %.2f secs ***", Timer.getTimestamp() - autoStart));
      }
      autoMessagePrinted = true;
    }

    robotContainer.periodic();
    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  /** This method is invoked periodically when the robot is in the disabled state. */
  @Override
  public void disabledPeriodic() {
    // check if the operator interface (e.g., joysticks) has changed
    robotContainer.updateOI();

    // check if the alliance color has changed based on the FMS data
    robotContainer.checkAllianceColor();

    // get the auto command and, if the default is selected, then do solid red leds
    if (AutonomousCommandsFactory.getInstance()
        .getAutonomousCommand()
        .getName()
        .equals("Do Nothing")) {
      LEDs.getInstance().requestState(LEDs.States.NO_AUTO_SELECTED);
      noAutoSelectedAlert.set(true);
    } else {
      noAutoSelectedAlert.set(false);
    }
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

    autoStart = Timer.getTimestamp();
    autoMessagePrinted = false;
    autonomousCommand = AutonomousCommandsFactory.getInstance().getAutonomousCommand();

    // schedule the autonomous command
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
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
