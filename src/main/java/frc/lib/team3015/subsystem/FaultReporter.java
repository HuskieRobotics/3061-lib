// derived from 3015's AdvancedSubsystem abstract class

package frc.lib.team3015.subsystem;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3015.subsystem.selfcheck.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * The FaultReporter class is a singleton that is used to check for and publish faults related to
 * registered subsystems and registered devices. It checks for faults only when the robot is
 * disabled at 1 second intervals. Subsystems can register their associated system check command via
 * the registerSystemCheck method. Hardware-specific classes can register their devices via the
 * registerHardware method. All faults are reported via the Alert class. The result of system checks
 * are published to NetworkTables under the "SystemStatus/[subsystemName]" key.
 *
 * <p>For each registered subsystem, the following keys are published under
 * "SystemStatus/[subsystemName]": <br>
 * - SystemCheck: The system check command that was registered with the FaultReporter <br>
 * - CheckRan: A boolean that indicates whether the system check command has been run <br>
 * - Status: The status of the subsystem, which is either OK, WARNING, or ERROR <br>
 * - SystemOK: A boolean that indicates whether the subsystem is OK
 */
public class FaultReporter {
  public enum SystemStatus {
    OK,
    WARNING,
    ERROR
  }

  private static class SubsystemFaults {
    private List<SubsystemFault> faults = new ArrayList<>();
    private List<Alert> faultAlerts = new ArrayList<>();
    private List<SelfChecking> hardware = new ArrayList<>();
  }

  private static FaultReporter instance = null;

  private static final String CHECK_RAN = "/CheckRan";
  private static final String SYSTEM_STATUS = "SystemStatus/";

  private final Map<String, SubsystemFaults> subsystemsFaults = new HashMap<>();

  private boolean startedCTRESignalLogger = false;

  private FaultReporter() {
    setupCallbacks();
    registerDashboardCommands();
  }

  /**
   * Returns the FaultReporter singleton. Invoke this method to get a reference to the FaultReporter
   *
   * @return the FaultReporter singleton
   */
  public static FaultReporter getInstance() {
    if (instance == null) {
      instance = new FaultReporter();
    }
    return instance;
  }

  /**
   * Registers a system check command with the FaultReporter for the specified subsystem. The system
   * check command will be wrapped in a command that will record the status of the system check and
   * publish the status to NetworkTables. The system check command will be scheduled when the
   * command is executed via a dashboard such as Shuffleboard or Elastic. The command is located at
   * "SystemStatus/[subsystemName]/SystemCheck" in Network Tables
   *
   * @param subsystemName the name of the subsystem that the system check command checks
   * @param systemCheckCommand the system check command to register
   * @return
   */
  public Command registerSystemCheck(String subsystemName, Command systemCheckCommand) {
    String statusTable = SYSTEM_STATUS + subsystemName;
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());

    Command wrappedSystemCheckCommand = wrapSystemCheckCommand(subsystemName, systemCheckCommand);
    wrappedSystemCheckCommand.setName(subsystemName + "Check");
    SmartDashboard.putData(statusTable + "/SystemCheck", wrappedSystemCheckCommand);
    Logger.recordOutput(statusTable + CHECK_RAN, false);

    subsystemsFaults.put(subsystemName, subsystemFaults);

    return wrappedSystemCheckCommand;
  }

  /**
   * Registers a command that clears all sticky faults for all devices registered with the
   * FaultReporter. The command is located at "SystemStatus/ClearAllFaults" in Network Tables. Also
   * registers a command that checks for faults in all registered devices even when the robot is
   * enabled. The command is located at "SystemStatus/CheckForFaults" in Network Tables. These
   * commands can be executed via a dashboard such as Shuffleboard or Elastic to help troubleshoot
   * when the robot is encountering problems during a match.
   */
  private void registerDashboardCommands() {
    SmartDashboard.putData(
        SYSTEM_STATUS + "ClearAllFaults",
        Commands.runOnce(
                () -> {
                  for (Map.Entry<String, SubsystemFaults> entry : subsystemsFaults.entrySet()) {
                    SubsystemFaults subsystemFaults = entry.getValue();
                    for (SelfChecking device : subsystemFaults.hardware) {
                      device.clearStickyFaults();
                    }

                    for (Alert alert : subsystemFaults.faultAlerts) {
                      alert.set(false);
                    }
                    subsystemFaults.faultAlerts.clear();
                    subsystemFaults.faults.clear();
                  }
                })
            .ignoringDisable(true)
            .withName("ClearAllFaults"));

    SmartDashboard.putData(
        SYSTEM_STATUS + "CheckForFaults",
        Commands.runOnce(this::checkForFaults).ignoringDisable(true).withName("check for faults"));
  }

  private Command wrapSystemCheckCommand(String subsystemName, Command systemCheckCommand) {
    String statusTable = SYSTEM_STATUS + subsystemName;
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Logger.recordOutput(statusTable + CHECK_RAN, false);
              clearFaults(subsystemName);
            }),
        systemCheckCommand,
        Commands.runOnce(
            () -> {
              SubsystemFaults subsystemFaults = subsystemsFaults.get(subsystemName);
              SystemStatus status = getSystemStatus(subsystemFaults.faults);
              Logger.recordOutput(statusTable + "/Status", status.name());
              Logger.recordOutput(statusTable + "/SystemOK", status == SystemStatus.OK);
              Logger.recordOutput(statusTable + CHECK_RAN, true);
            }));
  }

  private void setupCallbacks() {
    CommandScheduler.getInstance()
        .schedule(
            Commands.repeatingSequence(
                    Commands.runOnce(this::checkForFaultsWhenDisabled), Commands.waitSeconds(1.0))
                .ignoringDisable(true)
                .withName("check for faults when disabled"));
  }

  /**
   * Adds a fault to the FaultReporter for the specified subsystem. This method is invoked by the
   * various SelfChecking subclasses when they detect a failure. An alert will be generated for the
   * fault.
   *
   * @param subsystemName the name of the subsystem that the fault is associated with
   * @param fault the fault to add
   */
  private void addFault(String subsystemName, SubsystemFault fault) {
    SubsystemFaults subsystems = subsystemsFaults.get(subsystemName);
    List<SubsystemFault> subsystemFaults = subsystems.faults;
    List<Alert> subsystemAlerts = subsystems.faultAlerts;
    if (!subsystemFaults.contains(fault)) {
      subsystemFaults.add(fault);

      Alert alert =
          new Alert(
              subsystemName + ": " + fault.description,
              fault.isWarning ? Alert.AlertType.kWarning : Alert.AlertType.kError);
      alert.set(true);
      subsystemAlerts.add(alert);
    }
  }

  /**
   * Adds a fault to the FaultReporter for the specified subsystem. This method should be invoked by
   * the system check command for the specified subsystem when it detects a failure during the
   * system check. An alert will be generated for the fault.
   *
   * @param subsystemName the name of the subsystem that the fault is associated with
   * @param description the description of the fault
   * @param isWarning true if the fault is a warning, false if the fault is an error
   */
  public void addFault(String subsystemName, String description, boolean isWarning) {
    this.addFault(subsystemName, new SubsystemFault(description, isWarning));
  }

  /**
   * Adds a fault to the FaultReporter for the specified subsystem. This method should be invoked by
   * the system check command for the specified subsystem when it detects a failure during the
   * system check. An alert will be generated for the fault. Defaults to an error fault.
   *
   * @param subsystemName the name of the subsystem that the fault is associated with
   * @param description the description of the fault
   */
  public void addFault(String subsystemName, String description) {
    this.addFault(subsystemName, description, false);
  }

  /**
   * Returns a list of faults for the specified subsystem. This method is usually invoked within the
   * until decorator of the system check command to interrupt the command if a fault is detected.
   * Refer to the getSystemCheckCommand method in the Subsystem class for an example.
   *
   * @param subsystemName the name of the subsystem to get faults for
   * @return a list of faults for the specified subsystem
   */
  public List<SubsystemFault> getFaults(String subsystemName) {
    return subsystemsFaults.get(subsystemName).faults;
  }

  /**
   * Clears all faults for the specified subsystem. This method is invoked at the start of the
   * subsystem's wrapped system check command to clear all faults before starting the system check.
   *
   * @param subsystemName the name of the subsystem to clear faults for
   */
  private void clearFaults(String subsystemName) {
    SubsystemFaults subsystemFaults = subsystemsFaults.get(subsystemName);
    for (Alert alert : subsystemFaults.faultAlerts) {
      alert.set(false);
    }
    subsystemFaults.faultAlerts.clear();
    subsystemFaults.faults.clear();
  }

  /**
   * Returns the system status for the specified subsystem. The system status is determined by the
   * worst fault in the subsystem. If there are no faults, the status is OK. If there are only
   * warnings, the status is WARNING. If there are any errors, the status is ERROR.
   *
   * @param subsystemFaults the list of faults for the subsystem
   * @return the system status for the subsystem
   */
  private SystemStatus getSystemStatus(List<SubsystemFault> subsystemFaults) {
    SystemStatus worstStatus = SystemStatus.OK;

    for (SubsystemFault f : subsystemFaults) {
      if (f.isWarning) {
        if (worstStatus != SystemStatus.ERROR) {
          worstStatus = SystemStatus.WARNING;
        }
      } else {
        worstStatus = SystemStatus.ERROR;
      }
    }
    return worstStatus;
  }

  /**
   * Registers a hardware device with the FaultReporter. This method should be invoked by the
   * hardware-specific subsystem class to register all hardware devices associated with the
   * subsystem. Refer to the configMotor method in the Subsystem class for an example.
   *
   * @param subsystemName the name of the subsystem that the hardware device is associated with
   * @param label the label of the hardware device
   * @param phoenixMotor the hardware device to register
   */
  public void registerHardware(String subsystemName, String label, TalonFX phoenixMotor) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPhoenixMotor(label, phoenixMotor));
    subsystemsFaults.put(subsystemName, subsystemFaults);

    // The following is the recommended workaround from CTRE to ensure that the CANivore has been
    // enumerated by the root hub and therefore, hoot files will be properly generated.
    if (!this.startedCTRESignalLogger) {
      this.startedCTRESignalLogger = true;
      phoenixMotor.getVersion().waitForUpdate(0.5);
      SignalLogger.setPath("/media/sda1");
      SignalLogger.start();
    }
  }

  /**
   * Registers a hardware device with the FaultReporter. This method should be invoked by the
   * hardware-specific subsystem class to register all hardware devices associated with the
   * subsystem. Refer to the configMotor method in the Subsystem class for an example.
   *
   * @param subsystemName the name of the subsystem that the hardware device is associated with
   * @param label the label of the hardware device
   * @param pwmMotor the hardware device to register
   */
  public void registerHardware(String subsystemName, String label, PWMMotorController pwmMotor) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPWMMotor(label, pwmMotor));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Registers a hardware device with the FaultReporter. This method should be invoked by the
   * hardware-specific subsystem class to register all hardware devices associated with the
   * subsystem. Refer to the configMotor method in the Subsystem class for an example.
   *
   * @param subsystemName the name of the subsystem that the hardware device is associated with
   * @param label the label of the hardware device
   * @param spark the hardware device to register
   */
  public void registerHardware(String subsystemName, String label, SparkMax spark) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingSparkMax(label, spark));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Registers a hardware device with the FaultReporter. This method should be invoked by the
   * hardware-specific subsystem class to register all hardware devices associated with the
   * subsystem. Refer to the configMotor method in the Subsystem class for an example.
   *
   * @param subsystemName the name of the subsystem that the hardware device is associated with
   * @param label the label of the hardware device
   * @param pigeon2 the hardware device to register
   */
  public void registerHardware(String subsystemName, String label, Pigeon2 pigeon2) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPigeon2(label, pigeon2));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Registers a hardware device with the FaultReporter. This method should be invoked by the
   * hardware-specific subsystem class to register all hardware devices associated with the
   * subsystem. Refer to the configMotor method in the Subsystem class for an example.
   *
   * @param subsystemName the name of the subsystem that the hardware device is associated with
   * @param label the label of the hardware device
   * @param canCoder the hardware device to register
   */
  public void registerHardware(String subsystemName, String label, CANcoder canCoder) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingCANCoder(label, canCoder));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  /**
   * Checks for faults in all registered devices when the robot is disabled. This method is
   * scheduled to run at 1 second intervals when the robot is disabled. It checks for faults in all
   * registered devices.
   */
  private void checkForFaultsWhenDisabled() {
    if (DriverStation.isDisabled()) {
      checkForFaults();
    }
  }

  /**
   * Checks for faults in all registered devices. This method is invoked by the
   * checkForFaultsWhenDisabled method when the robot is disabled. It iterates through all
   * registered devices and checks for faults.
   */
  public void checkForFaults() {
    for (Map.Entry<String, SubsystemFaults> entry : subsystemsFaults.entrySet()) {
      String subsystemName = entry.getKey();
      SubsystemFaults subsystemFaults = entry.getValue();
      for (SelfChecking device : subsystemFaults.hardware) {
        for (SubsystemFault fault : device.checkForFaults()) {
          addFault(subsystemName, fault);
        }
      }
    }
  }
}
