package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;

public class ArmCommandFactory {

  private ArmCommandFactory() {}

  public static void registerCommands(OperatorInterface oi, Arm arm) {

    oi.getMoveArmMiddlePositionTrigger()
        .onTrue(new InstantCommand(() -> arm.goToAngle(45.0), arm))
        .onFalse(new InstantCommand(() -> arm.goToAngle(0.0), arm));

    oi.getMoveArmHighPositionTrigger()
        .onTrue(new InstantCommand(() -> arm.goToAngle(90.0), arm))
        .onFalse(new InstantCommand(() -> arm.goToAngle(0.0), arm));
  }
}
