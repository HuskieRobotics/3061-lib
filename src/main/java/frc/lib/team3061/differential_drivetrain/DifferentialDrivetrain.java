// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team3061.differential_drivetrain;

import static frc.lib.team3061.differential_drivetrain.DifferentialDrivetrainConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class DifferentialDrivetrain extends SubsystemBase {
  private final DifferentialDrivetrainIO io;
  private final DifferentialDrivetrainIOInputsAutoLogged inputs =
      new DifferentialDrivetrainIOInputsAutoLogged();

  /** Creates a new Drivetrain. */
  public DifferentialDrivetrain(DifferentialDrivetrainIO io) {
    this.io = io;
  }

  public void arcadeDrive(double xVelocity, double rotationalVelocity) {

    io.driveRobotRelative(xVelocity, rotationalVelocity, true);
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    Logger.processInputs(SUBSYSTEM_NAME, this.inputs);
  }
}
