// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import org.wpilib.command2.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  // drivetrain, generic

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseToVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdDynamicReverse() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticForward() {
    return new Trigger(() -> false);
  }

  public default Trigger getSysIdQuasistaticReverse() {
    return new Trigger(() -> false);
  }

  // DRIVER TRIGGERS, mostly game-specific

  public default Trigger getDriveToPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getOverrideDriveToPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLimitAccelerationAndVelocityButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getSnakeDriveButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCurrentPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getInterruptAll() {
    return new Trigger(() -> false);
  }

  // OPERATOR TRIGGERS, mostly game-specific
  public default Trigger getEnablePrimaryIRSensorsTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getEnableAutoScoringTrigger() {
    return new Trigger(() -> false);
  }

  // XRP EXAMPLE TRIGGERS
  public default Trigger getMoveArmMiddlePositionTrigger() {
    return new Trigger(() -> false);
  }

  public default Trigger getMoveArmHighPositionTrigger() {
    return new Trigger(() -> false);
  }

  // ELEVATOR EXAMPLE TRIGGERS
  public default Trigger getRaiseElevatorSlowButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLowerElevatorSlowButton() {
    return new Trigger(() -> false);
  }

  // only for at home
  public default Trigger getSimulateCollisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAutoSnapsEnabledTrigger() {
    return new Trigger(() -> false);
  }

  // System Tests

  public default Trigger getDrivetrainSystemTest() {
    return new Trigger(() -> false);
  }

  public default Trigger getArmSystemTest() {
    return new Trigger(() -> false);
  }

  public default Trigger getElevatorSystemTest() {
    return new Trigger(() -> false);
  }

  public default Trigger getManipulatorSystemTest() {
    return new Trigger(() -> false);
  }

  public default Trigger getShooterSystemTest() {
    return new Trigger(() -> false);
  }

  // Faults

  public default Trigger getClearAllFaults() {
    return new Trigger(() -> false);
  }

  public default Trigger getCheckForFaults() {
    return new Trigger(() -> false);
  }
}
