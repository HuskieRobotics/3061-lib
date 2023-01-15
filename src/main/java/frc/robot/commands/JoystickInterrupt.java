package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.function.DoubleSupplier;

public class JoystickInterrupt extends CommandBase {
  DoubleSupplier[] joystickSuppliers;
  double[] initialValues;
  boolean finished;

  JoystickInterrupt(DoubleSupplier[] joystickSuppliers) {
    this.finished = false;
    this.joystickSuppliers = joystickSuppliers;
    initialValues = new double[joystickSuppliers.length];
  }

  @Override
  public void initialize() {
    for (int i = 0; i < joystickSuppliers.length; i++) {
      initialValues[i] = joystickSuppliers[i].getAsDouble();
    }
  }

  @Override
  public void execute() {
    for (int i = 0; i < joystickSuppliers.length; i++) {
      if (Math.abs(joystickSuppliers[i].getAsDouble() - initialValues[i])
          > DrivetrainConstants.DEADBAND) {
        finished = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
