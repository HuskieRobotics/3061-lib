// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI extends OperatorDashboard {
  private final CommandXboxController controller;

  public SingleHandheldOI(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getTranslateX() {
    return -controller.getLeftY();
  }

  @Override
  public double getTranslateY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRotate() {
    return -controller.getRightX();
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return controller.b();
  }

  @Override
  public Trigger getResetGyroButton() {
    return controller.start();
  }

  @Override
  public Trigger getXStanceButton() {
    return controller.y();
  }

  @Override
  public Trigger getTranslationSlowModeButton() {
    return controller.leftBumper();
  }

  @Override
  public Trigger getRotationSlowModeButton() {
    return controller.rightBumper();
  }

  @Override
  public Trigger getLock180Button() {
    return controller.a();
  }

  @Override
  public Trigger getSysIdDynamicForward() {
    return controller.back().and(controller.y());
  }

  @Override
  public Trigger getSysIdDynamicReverse() {
    return controller.back().and(controller.x());
  }

  @Override
  public Trigger getSysIdQuasistaticForward() {
    return controller.start().and(controller.y());
  }

  @Override
  public Trigger getSysIdQuasistaticReverse() {
    return controller.start().and(controller.x());
  }
}
