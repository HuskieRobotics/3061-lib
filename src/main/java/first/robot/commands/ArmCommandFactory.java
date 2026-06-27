package first.robot.commands;

import first.lib.team3015.subsystem.FaultReporter;
import first.robot.operator_interface.OperatorInterface;
import first.robot.subsystems.arm.Arm;
import first.robot.subsystems.arm.ArmConstants;
import org.wpilib.command2.InstantCommand;
import org.wpilib.math.util.Units;

public class ArmCommandFactory {

  private ArmCommandFactory() {}

  public static void registerCommands(OperatorInterface oi, Arm arm) {

    oi.getMoveArmMiddlePositionTrigger()
        .onTrue(
            new InstantCommand(() -> arm.setAngleRotations(Units.degreesToRotations(45.0)), arm)
                .withName("Move Arm to Middle Position"));
    oi.getMoveArmMiddlePositionTrigger()
        .onFalse(
            new InstantCommand(() -> arm.setAngleRotations(Units.degreesToRotations(0.0)), arm)
                .withName("Reset Arm to Zero Position"));

    oi.getMoveArmHighPositionTrigger()
        .onTrue(
            new InstantCommand(() -> arm.setAngleRotations(Units.degreesToRotations(90.0)), arm)
                .withName("Move Arm to High Position"));
    oi.getMoveArmHighPositionTrigger()
        .onFalse(
            new InstantCommand(() -> arm.setAngleRotations(Units.degreesToRotations(0.0)), arm)
                .withName("Reset Arm to Zero Position"));

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
    FaultReporter.getInstance()
        .registerSystemCheck(
            ArmConstants.SUBSYSTEM_NAME, arm.getSystemCheckCommand(), oi.getArmSystemTest());
  }
}
