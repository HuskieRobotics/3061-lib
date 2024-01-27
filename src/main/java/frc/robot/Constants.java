// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>Subsystem-specific constants should be defined in the subsystem's own constant class.
 * Constants that vary from robot to robot should be defined in the config classes.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // set to true in order to change all Tunable values via Shuffleboard
  public static final boolean TUNING_MODE = false;

  private static final RobotType ROBOT = RobotType.ROBOT_SIMBOT;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  // FIXME: update for various robots
  public enum RobotType {
    ROBOT_2023_NOVA_CTRE,
    ROBOT_2023_NOVA_CTRE_FOC,
    ROBOT_2023_NOVA,
    ROBOT_DEFAULT,
    ROBOT_SIMBOT,
    ROBOT_SIMBOT_CTRE,
    ROBOT_PRACTICE
  }

  // FIXME: update for various robots
  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT
          || ROBOT == RobotType.ROBOT_SIMBOT_CTRE) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_DEFAULT;
      } else {
        return ROBOT;
      }
    } else {
      return ROBOT;
    }
  }

  // FIXME: update for various robots
  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_DEFAULT:
      case ROBOT_2023_NOVA_CTRE:
      case ROBOT_2023_NOVA_CTRE_FOC:
      case ROBOT_2023_NOVA:
      case ROBOT_PRACTICE:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
      case ROBOT_SIMBOT_CTRE:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  public static final double LOOP_PERIOD_SECS = 0.02;
}
