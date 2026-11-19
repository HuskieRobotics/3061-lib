package frc.robot.commands;

import frc.lib.team3015.subsystem.FaultReporter;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;

public class ArmCommandFactory {

  private ArmCommandFactory() {}

  public static void registerCommands(OperatorInterface oi, Arm arm) {

    oi.getMoveArmStowPositionTrigger()
        .onTrue(
            arm.getMoveArmToCommand(ArmConstants.ARM_STOWED_POSITION)
                .withName("Move Arm to Stow Position"));

    oi.getMoveArmMiddlePositionTrigger()
        .onTrue(
            arm.getMoveArmToCommand(ArmConstants.ARM_MIDDLE_SETPOINT)
                .withName("Move Arm to Middle Position"));

    oi.getMoveArmHighPositionTrigger()
        .onTrue(
            arm.getMoveArmToCommand(ArmConstants.ARM_HIGHER_SETPOINT)
                .withName("Move Arm to High Position"));

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
    FaultReporter.getInstance()
        .registerSystemCheck(
            ArmConstants.SUBSYSTEM_NAME, arm.getSystemCheckCommand(), oi.getArmSystemTest());
  }
}
