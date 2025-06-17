package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommandsFactory {

  private ElevatorCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Elevator elevator) {

    oi.getRaiseElevatorSlowButton()
        .onTrue(
            Commands.runOnce(elevator::raiseElevatorSlow, elevator)
                .withName("raise elevator slow"));

    oi.getRaiseElevatorSlowButton()
        .onFalse(Commands.runOnce(elevator::stop, elevator).withName("stop elevator"));

    oi.getLowerElevatorSlowButton()
        .onTrue(
            Commands.runOnce(elevator::lowerElevatorSlow, elevator)
                .withName("lower elevator slow"));
    oi.getLowerElevatorSlowButton()
        .onFalse(
            Commands.sequence(
                    Commands.runOnce(elevator::stop, elevator),
                    Commands.runOnce(elevator::zero, elevator))
                .withName("stop and zero elevator"));
  }
}
