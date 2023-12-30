package frc.lib.team3015.subsystem;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3015.subsystem.selfcheck.*;
import frc.lib.team6328.util.Alert;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

// derived from 3015's AdvancedSubsystem abstract class

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
  private final boolean checkErrors;

  private FaultReporter() {
    this.checkErrors = RobotBase.isReal();
    setupCallbacks();
  }

  public static FaultReporter getInstance() {
    if (instance == null) {
      instance = new FaultReporter();
    }
    return instance;
  }

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

  private Command wrapSystemCheckCommand(String subsystemName, Command systemCheckCommand) {
    String statusTable = SYSTEM_STATUS + subsystemName;
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              Logger.recordOutput(statusTable + CHECK_RAN, false);
              clearFaults(subsystemName);
              publishStatus();
            }),
        systemCheckCommand,
        Commands.runOnce(
            () -> {
              publishStatus();
              Logger.recordOutput(statusTable + CHECK_RAN, true);
            }));
  }

  private void setupCallbacks() {
    CommandScheduler.getInstance()
        .schedule(
            Commands.repeatingSequence(
                    Commands.runOnce(this::checkForFaults), Commands.waitSeconds(0.25))
                .ignoringDisable(true));

    CommandScheduler.getInstance()
        .schedule(
            Commands.repeatingSequence(
                    Commands.runOnce(this::publishStatus), Commands.waitSeconds(1.0))
                .ignoringDisable(true));
  }

  private void publishStatus() {
    for (Map.Entry<String, SubsystemFaults> entry : subsystemsFaults.entrySet()) {
      String subsystemName = entry.getKey();
      SubsystemFaults subsystemFaults = entry.getValue();

      SystemStatus status = getSystemStatus(subsystemFaults.faults);

      String statusTable = SYSTEM_STATUS + subsystemName;
      Logger.recordOutput(statusTable + "/Status", status.name());
      Logger.recordOutput(statusTable + "/SystemOK", status == SystemStatus.OK);

      String[] faultStrings = new String[subsystemFaults.faults.size()];
      for (int i = 0; i < subsystemFaults.faults.size(); i++) {
        SubsystemFault fault = subsystemFaults.faults.get(i);
        faultStrings[i] = String.format("[%.2f] %s", fault.timestamp, fault.description);
      }
      Logger.recordOutput(statusTable + "/Faults", faultStrings);

      if (faultStrings.length > 0) {
        Logger.recordOutput(statusTable + "/LastFault", faultStrings[faultStrings.length - 1]);
      } else {
        Logger.recordOutput(statusTable + "/LastFault", "");
      }
    }
  }

  public void addFault(String subsystemName, SubsystemFault fault) {
    SubsystemFaults subsystems = subsystemsFaults.get(subsystemName);
    List<SubsystemFault> subsystemFaults = subsystems.faults;
    List<Alert> subsystemAlerts = subsystems.faultAlerts;
    if (!subsystemFaults.contains(fault)) {
      subsystemFaults.add(fault);

      Alert alert =
          new Alert(
              subsystemName + ": " + fault.description,
              fault.isWarning ? Alert.AlertType.WARNING : Alert.AlertType.ERROR);
      alert.set(true);
      subsystemAlerts.add(alert);
    }
  }

  public void addFault(String subsystemName, String description, boolean isWarning) {
    this.addFault(subsystemName, new SubsystemFault(description, isWarning));
  }

  public void addFault(
      String subsystemName, String description, boolean isWarning, boolean sticky) {
    this.addFault(subsystemName, new SubsystemFault(description, isWarning, sticky));
  }

  public void addFault(String subsystemName, String description) {
    this.addFault(subsystemName, description, false);
  }

  public List<SubsystemFault> getFaults(String subsystemName) {
    return subsystemsFaults.get(subsystemName).faults;
  }

  public void clearFaults(String subsystemName) {
    subsystemsFaults.get(subsystemName).faults.clear();
  }

  private SystemStatus getSystemStatus(List<SubsystemFault> subsystemFaults) {
    SystemStatus worstStatus = SystemStatus.OK;

    for (SubsystemFault f : subsystemFaults) {
      if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
        if (f.isWarning) {
          if (worstStatus != SystemStatus.ERROR) {
            worstStatus = SystemStatus.WARNING;
          }
        } else {
          worstStatus = SystemStatus.ERROR;
        }
      }
    }
    return worstStatus;
  }

  public void registerHardware(String subsystemName, String label, TalonFX phoenixMotor) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPhoenixMotor(label, phoenixMotor));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  public void registerHardware(String subsystemName, String label, PWMMotorController pwmMotor) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPWMMotor(label, pwmMotor));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  public void registerHardware(String subsystemName, String label, CANSparkMax spark) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingSparkMax(label, spark));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  public void registerHardware(String subsystemName, String label, Pigeon2 pigeon2) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingPigeon2(label, pigeon2));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  public void registerHardware(String subsystemName, String label, CANcoder canCoder) {
    SubsystemFaults subsystemFaults =
        subsystemsFaults.getOrDefault(subsystemName, new SubsystemFaults());
    subsystemFaults.hardware.add(new SelfCheckingCANCoder(label, canCoder));
    subsystemsFaults.put(subsystemName, subsystemFaults);
  }

  // Method to check for faults while the robot is operating normally
  private void checkForFaults() {
    if (checkErrors) {
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
}
