package first.robot.commands;

import first.lib.team3015.subsystem.FaultReporter;
import first.robot.operator_interface.OperatorInterface;
import first.robot.subsystems.shooter.Shooter;
import first.robot.subsystems.shooter.ShooterConstants;

public class ShooterCommandsFactory {

  private ShooterCommandsFactory() {}

  public static void registerCommands(OperatorInterface oi, Shooter shooter) {

    FaultReporter.getInstance()
        .registerSystemCheck(
            ShooterConstants.SUBSYSTEM_NAME,
            shooter.getSystemCheckCommand(),
            oi.getShooterSystemTest());
  }
}
