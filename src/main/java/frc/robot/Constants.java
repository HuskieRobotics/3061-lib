// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

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

  // set to true in order to change all Tunable values via AdvantageScope
  public static final boolean TUNING_MODE = true;
  public static final boolean DEMO_MODE = false;

  private static final RobotType ROBOT = RobotType.ROBOT_SIMBOT;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError);

  // FIXME: update for various robots
  public enum RobotType {
    ROBOT_DEFAULT,
    ROBOT_SIMBOT,
    ROBOT_XRP,
    ROBOT_PRACTICE,
    ROBOT_COMPETITION,
    ROBOT_PRACTICE_BOARD,
    ROBOT_VISION_TEST_PLATFORM,
    ROBOT_NORTHSTAR_TEST_PLATFORM
  }

  // FIXME: update for various robots
  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (ROBOT == RobotType.ROBOT_SIMBOT
          || ROBOT == RobotType.ROBOT_XRP) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_COMPETITION;
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
      case ROBOT_DEFAULT, ROBOT_PRACTICE, ROBOT_PRACTICE_BOARD, ROBOT_COMPETITION:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT, ROBOT_VISION_TEST_PLATFORM, ROBOT_NORTHSTAR_TEST_PLATFORM, ROBOT_XRP:
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
