package first.robot.commands;

import first.lib.team3015.subsystem.FaultReporter;
import first.robot.operator_interface.OperatorInterface;
import first.robot.subsystems.manipulator.Manipulator;
import first.robot.subsystems.manipulator.ManipulatorConstants;

public class ManipulatorCommandsFactory {
  private ManipulatorCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Manipulator manipulator) {

    FaultReporter.getInstance()
        .registerSystemCheck(
            ManipulatorConstants.SUBSYSTEM_NAME,
            manipulator.getSystemCheckCommand(),
            oi.getManipulatorSystemTest());
  }
}
