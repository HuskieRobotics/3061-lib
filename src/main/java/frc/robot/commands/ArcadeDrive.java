// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.differential_drivetrain.DifferentialDrivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ArcadeDrive extends Command {
  private final DifferentialDrivetrain drivetrain;
  private final Supplier<Double> xVelocitySupplier;
  private final Supplier<Double> rotationVelocitySupplier;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xVelocityMPSSupplier Lambda supplier of forward/backward speed [-1.0, 1.0]
   * @param rotationVelocityRPSSupplier Lambda supplier of rotational speed [-1.0, 1.0]
   */
  public ArcadeDrive(
      DifferentialDrivetrain drivetrain,
      Supplier<Double> xVelocityMPSSupplier,
      Supplier<Double> rotationVelocityRPSSupplier) {
    this.drivetrain = drivetrain;
    this.xVelocitySupplier = xVelocityMPSSupplier;
    this.rotationVelocitySupplier = rotationVelocityRPSSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xVelocityMPS = xVelocitySupplier.get();
    double rotationalVelocityRPS = rotationVelocitySupplier.get();

    drivetrain.arcadeDrive(
        RobotConfig.getInstance().getRobotMaxVelocityMPS() * xVelocityMPS,
        RobotConfig.getInstance().getRobotMaxAngularVelocityRPS() * rotationalVelocityRPS);

    Logger.recordOutput(
        "ArcadeDrive/xVelocity", RobotConfig.getInstance().getRobotMaxVelocityMPS() * xVelocityMPS);
    Logger.recordOutput(
        "ArcadeDrive/rotationalVelocity",
        RobotConfig.getInstance().getRobotMaxAngularVelocityRPS() * rotationalVelocityRPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
