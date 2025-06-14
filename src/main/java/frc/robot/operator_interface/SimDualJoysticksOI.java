// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two joysticks. */
public class SimDualJoysticksOI extends OperatorDashboard {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;

  public SimDualJoysticksOI(int translatePort, int rotatePort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
  }

  // translation joystick

  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public Trigger getDriveToPoseButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[2];
  }

  @Override
  public Trigger getResetPoseToVisionButton() {
    return translateJoystickButtons[3];
  }

  // rotation joystick

  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public Trigger getMoveArmMiddlePositionTrigger() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getMoveArmHighPositionTrigger() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getInterruptAll() {
    return rotateJoystickButtons[4];
  }
}
