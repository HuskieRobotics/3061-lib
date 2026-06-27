package first.robot.commands;

import first.lib.team3015.subsystem.FaultReporter;
import first.robot.operator_interface.OperatorInterface;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.elevator.ElevatorConstants;
import org.wpilib.command2.Commands;

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

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
    FaultReporter.getInstance()
        .registerSystemCheck(
            ElevatorConstants.SUBSYSTEM_NAME,
            elevator.getElevatorSystemCheckCommand(),
            oi.getElevatorSystemTest());
  }
}
