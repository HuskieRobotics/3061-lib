package frc.robot.commands;

import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

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
