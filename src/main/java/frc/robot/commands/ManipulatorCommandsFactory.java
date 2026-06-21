package frc.robot.commands;

import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorConstants;

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
