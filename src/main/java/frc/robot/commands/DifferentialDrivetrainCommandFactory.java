package frc.robot.commands;

import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import frc.robot.operator_interface.OperatorInterface;

public class DifferentialDrivetrainCommandFactory {

  private DifferentialDrivetrainCommandFactory() {}

  public static void registerCommands(
      OperatorInterface oi, DifferentialDrivetrain differentialDrivetrain) {
    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    differentialDrivetrain.setDefaultCommand(
        new ArcadeDrive(differentialDrivetrain, oi::getTranslateX, oi::getRotate));
  }
}
