// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for controlling the robot with two joysticks, 1 Xbox controller, and 1 operator button
 * panel.
 */
public class FullOperatorConsoleOI extends OperatorDashboard {
  private final CommandJoystick translateJoystick;
  private final Trigger[] translateJoystickButtons;

  private final CommandJoystick rotateJoystick;
  private final Trigger[] rotateJoystickButtons;

  private final CommandXboxController operatorController;

  public FullOperatorConsoleOI(int translatePort, int rotatePort, int operatorControllerPort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);
    operatorController = new CommandXboxController(operatorControllerPort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
  }

  // Translation Joystick
  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public Trigger getLock180Button() {
    return translateJoystickButtons[2];
  }

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getSnakeDriveButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger getInterruptAll() {
    return translateJoystickButtons[5];
  }

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[8];
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[9];
  }

  // Rotation Joystick
  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public Trigger getXStanceButton() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger getResetPoseToVisionButton() {
    return rotateJoystickButtons[5];
  }

  @Override
  public Trigger getCurrentPoseButton() {
    return rotateJoystickButtons[6];
  }

  // Operator Controller
  @Override
  public Trigger getDrivetrainSystemTest() {
    return operatorController.back().and(operatorController.b());
  }

  @Override
  public Trigger getShooterSystemTest() {
    return operatorController.back().and(operatorController.start());
  }

  @Override
  public Trigger getIntakeSystemTest() {
    return operatorController.back().and(operatorController.y());
  }

  @Override
  public Trigger getHopperSystemTest() {
    return operatorController.back().and(operatorController.x());
  }

  @Override
  public Trigger getClearAllFaults() {
    return operatorController.leftBumper().and(operatorController.rightBumper());
  }

  @Override
  public Trigger getCheckForFaults() {
    return operatorController.rightBumper();
  }
}
