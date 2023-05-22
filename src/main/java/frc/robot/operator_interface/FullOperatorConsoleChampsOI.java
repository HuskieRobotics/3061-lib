// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class FullOperatorConsoleChampsOI implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final Trigger[] translateJoystickButtons;

  private final CommandJoystick rotateJoystick;
  private final Trigger[] rotateJoystickButtons;

  private final XboxController operatorController;

  private final CommandJoystick operatorPanel;
  private final Trigger[] operatorPanelButtons;

  public FullOperatorConsoleChampsOI(
      int translatePort, int rotatePort, int operatorControllerPort, int operatorPanelPort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);
    operatorController = new XboxController(operatorControllerPort);
    operatorPanel = new CommandJoystick(operatorPanelPort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];
    this.operatorPanelButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
    for (int i = 1; i < operatorPanelButtons.length; i++) {
      operatorPanelButtons[i] = operatorPanel.button(i);
    }
  }

  // Translate Joystick
  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public Trigger getToggleIntakeButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getIntakeShootButton() {
    return translateJoystickButtons[2];
  }

  @Override
  public Trigger getTurn180Button() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger getInterruptAll() {
    return translateJoystickButtons[5];
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[9];
  }

  @Override
  public Trigger getAutoBalanceTestNOTGRID() {
    return translateJoystickButtons[10];
  }

  // Rotate Joystick

  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public Trigger getReleaseTriggerButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getFinishExtensionButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getIntakeShelfGridSideButton() {
    return rotateJoystickButtons[3];
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
  public Trigger getAutoZeroExtensionButton() {
    return rotateJoystickButtons[9];
  }

  @Override
  public Trigger getAutoBalanceTest() {
    return rotateJoystickButtons[10];
  }

  @Override
  public Trigger getSpoolRecoverButton() {
    return rotateJoystickButtons[11];
  }

  // Operator Controller

  @Override
  public Trigger getIntakeDeployButton() {
    return new Trigger(() -> getControllerLeftTrigger());
  }

  @Override
  public Trigger getIntakeRetractButton() {
    return new Trigger(() -> getControllerRightTrigger());
  }

  public boolean getControllerLeftTrigger() {
    return operatorController.getLeftTriggerAxis() > 0.5;
  }

  public boolean getControllerRightTrigger() {
    return operatorController.getRightTriggerAxis() > 0.5;
  }

  @Override
  public Trigger getDisableArmBackupButton() {
    return new Trigger(operatorController::getLeftBumper);
  }

  @Override
  public Trigger getMoveArmToShelfButton() {
    return new Trigger(operatorController::getRightBumper);
  }

  @Override
  public Trigger getMoveArmToLowButton() {
    return new Trigger(operatorController::getAButton);
  }

  @Override
  public Trigger getMoveArmToStorageButton() {
    return new Trigger(operatorController::getBButton);
  }

  @Override
  public Trigger getMoveArmToMidButton() {
    return new Trigger(operatorController::getXButton);
  }

  @Override
  public Trigger getMoveArmToHighButton() {
    return new Trigger(operatorController::getYButton);
  }

  @Override
  public double getRotateArm() {
    return -operatorController.getRightY();
  }

  @Override
  public double getMoveElevator() {
    return -operatorController.getLeftY();
  }

  // not implemented
  @Override
  public Trigger getToggleIntakeRollerButton() {
    return new Trigger(operatorController::getLeftStickButton);
  }

  @Override
  public Trigger getToggleManipulatorOpenCloseButton() {
    return new Trigger(operatorController::getRightStickButton);
  }

  @Override
  public boolean getManualManipulatorClose() {
    return operatorController.getBackButtonPressed();
  }

  @Override
  public Trigger getEnableManualElevatorControlButton() {
    return new Trigger(() -> operatorController.getPOV() == 0);
  }

  @Override
  public Trigger getDisableManualElevatorControlButton() {
    return new Trigger(() -> operatorController.getPOV() == 180);
  }

  @Override
  public Trigger getEnableManualElevatorPresetButton() {
    return new Trigger(() -> operatorController.getPOV() == 270);
  }

  @Override
  public Trigger getDisableManualElevatorPresetButton() {
    return new Trigger(() -> operatorController.getPOV() == 90);
  }

  // Operator Panel

  @Override
  public Trigger getConeCubeLEDTriggerButton() {
    return operatorPanelButtons[1];
  }

  @Override
  public Trigger getDisableArmButton() {
    return operatorPanelButtons[2];
  }

  @Override
  public Trigger getMoveArmToStorageBackupButton() {
    return operatorPanelButtons[5];
  }

  @Override
  public Trigger getIntakeShelfWallSideBackupButton() {
    return operatorPanelButtons[6];
  }

  @Override
  public Trigger getIntakeShelfGridSideBackupButton() {
    return operatorPanelButtons[7];
  }

  @Override
  public Trigger getIntakeGroundConeButton() {
    return operatorPanelButtons[8];
  }

  @Override
  public Trigger getMoveToGridButton() {
    return operatorPanelButtons[9];
  }

  @Override
  public Trigger getVisionIsEnabledSwitch() {
    return operatorPanelButtons[10];
  }

  @Override
  public Trigger getMoveToGridEnabledSwitch() {
    return operatorPanelButtons[11];
  }

  @Override
  public Trigger getToggleManipulatorSensorButton() {
    return operatorPanelButtons[12];
  }

  private double getScoringGridSwitchValue() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      if (operatorPanelButtons[3].getAsBoolean()) {
        return 1;
      } else if (operatorPanelButtons[4].getAsBoolean()) {
        return -1;
      } else {
        return 0;
      }
    } else {
      if (operatorPanelButtons[3].getAsBoolean()) {
        return -1;
      } else if (operatorPanelButtons[4].getAsBoolean()) {
        return 1;
      } else {
        return 0;
      }
    }
  }

  private double getScoringColumnSwitchValue() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return (Math.round(operatorPanel.getX()) * -1);
    } else {
      return Math.round(operatorPanel.getX());
    }
  }

  private double getScoringLevelSwitchValue() {
    return Math.round(operatorPanel.getY());
  }

  @Override
  public GridRow getGridRow() {
    if (this.getScoringLevelSwitchValue() == 1) {
      return GridRow.BOTTOM;
    } else if (this.getScoringLevelSwitchValue() == 0) {
      return GridRow.MIDDLE;
    } else {
      return GridRow.TOP;
    }
  }

  @Override
  public Node getNode() {
    if (this.getScoringGridSwitchValue() == -1) {
      if (this.getScoringColumnSwitchValue() == -1) {
        return Node.NODE_1;
      } else if (this.getScoringColumnSwitchValue() == 0) {
        return Node.NODE_2;
      } else {
        return Node.NODE_3;
      }
    } else if (this.getScoringGridSwitchValue() == 0) {
      if (this.getScoringColumnSwitchValue() == -1) {
        return Node.NODE_4;
      } else if (this.getScoringColumnSwitchValue() == 0) {
        return Node.NODE_5;
      } else {
        return Node.NODE_6;
      }
    } else {
      if (this.getScoringColumnSwitchValue() == -1) {
        return Node.NODE_7;
      } else if (this.getScoringColumnSwitchValue() == 0) {
        return Node.NODE_8;
      } else {
        return Node.NODE_9;
      }
    }
  }
}
