// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

/**
 * Utility class for selecting the appropriate OI implementations based on the connected joysticks.
 */
public class OISelector {
  private static String[] lastJoystickNames = new String[] {null, null, null, null, null, null};
  private static final Alert noOperatorInterfaceWarning =
      new Alert("No operator controller(s) connected.", AlertType.WARNING);
  private static final Alert nonCompetitionOperatorInterfaceWarning =
      new Alert("Non-competition operator controller connected.", AlertType.WARNING);

  private OISelector() {}

  /**
   * Returns whether the connected joysticks have changed since the last time this method was
   * called.
   */
  public static boolean didJoysticksChange() {
    boolean joysticksChanged = false;
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      String name = DriverStation.getJoystickName(port);
      if (!name.equals(lastJoystickNames[port])) {
        lastJoystickNames[port] = name;
        joysticksChanged = true;
      }
    }
    return joysticksChanged;
  }

  /**
   * Instantiates and returns an appropriate OperatorInterface object based on the connected
   * joysticks.
   */
  public static OperatorInterface findOperatorInterface() {
    Integer firstPort = null;
    Integer secondPort = null;
    Integer xBoxPort = null;
    for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
      if (DriverStation.getJoystickName(port).toLowerCase().contains("xbox")) {
        if (xBoxPort == null) {
          xBoxPort = port;
        }
      } else if (!DriverStation.getJoystickName(port).equals("")) {
        if (firstPort == null) {
          firstPort = port;
        } else if (secondPort == null) {
          secondPort = port;
        }
      }
    }

    if (firstPort != null && secondPort != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(false);
      return new DualJoysticksOI(firstPort, secondPort);
    } else if (xBoxPort != null) {
      noOperatorInterfaceWarning.set(false);
      nonCompetitionOperatorInterfaceWarning.set(true);
      return new SingleHandheldOI(xBoxPort);
    } else {
      noOperatorInterfaceWarning.set(true);
      nonCompetitionOperatorInterfaceWarning.set(false);
      return new OperatorInterface() {};
    }
  }
}
