// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.drivetrain.Drivetrain;

public class Demo extends Command {
  /** Creates a new Demo. */
  Drivetrain drivetrain;

  public Demo(Drivetrain train) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(train);
    drivetrain = train;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0, 0, 0.5, true, drivetrain.getFieldRelative());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0,false,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
