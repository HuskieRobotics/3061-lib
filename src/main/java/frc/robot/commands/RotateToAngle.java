// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.FieldConstants;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RotateToAngle extends CommandBase {
  private final Drivetrain drivetrain;
  private double angle;
  private boolean running = false;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final boolean determineShelfOrGrid;

  protected static final TunableNumber thetaKp = new TunableNumber("RotateToAngle/ThetaKp", 2);
  protected static final TunableNumber thetaKd = new TunableNumber("RotateToAngle/ThetaKd", 0.1);
  protected static final TunableNumber thetaKi =
      new TunableNumber("RotateToAngle/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());
  protected static final TunableNumber thetaMaxVelocity =
      new TunableNumber("RotateToAngle/ThetaMaxVelocity", 8);
  protected static final TunableNumber thetaMaxAcceleration =
      new TunableNumber("RotateToAngle/ThetaMaxAcceleration", 100);
  protected static final TunableNumber thetaTolerance =
      new TunableNumber(
          "RotateToAngle/ThetaTolerance", RobotConfig.getInstance().getDriveToPoseThetaTolerance());

  protected final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
          LOOP_PERIOD_SECS);

  /** Drives to the specified pose under full software control. */
  public RotateToAngle(Drivetrain drivetrain, double desiredAngle) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.translationXSupplier = () -> 0.0;
    this.translationYSupplier = () -> 0.0;
    this.angle = desiredAngle;
    this.determineShelfOrGrid = false;
  }

  public RotateToAngle(
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.determineShelfOrGrid = true;
    this.angle = 0.0;
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/RotateToAngle", true);

    // Reset all controllers
    Pose2d currentPose = drivetrain.getPose();
    thetaController.reset(currentPose.getRotation().getRadians());
    thetaController.setTolerance(thetaTolerance.get());
    if (determineShelfOrGrid) {
      if (drivetrain.getPose().getX() > (FieldConstants.fieldLength / 2)) {
        this.angle = 0;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

      } else {
        this.angle = 180;
        thetaController.enableContinuousInput(0, 2 * Math.PI);
      }
    }

    Logger.getInstance().recordOutput("RotateToAngle/angle", angle);
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (thetaKp.hasChanged()
        || thetaKd.hasChanged()
        || thetaKi.hasChanged()
        || thetaMaxVelocity.hasChanged()
        || thetaMaxAcceleration.hasChanged()) {
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
      thetaController.setI(thetaKi.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Get current and target pose
    Pose2d currentPose = drivetrain.getPose();

    // Command speeds
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), Units.degreesToRadians(this.angle));
    if (thetaController.atGoal()) {
      thetaVelocity = 0.0;
    }
    double xPercentage = modifyAxis(translationXSupplier.getAsDouble(), 2.0);
    double yPercentage = modifyAxis(translationYSupplier.getAsDouble(), 2.0);

    double xVelocity = xPercentage * RobotConfig.getInstance().getRobotMaxVelocity();
    double yVelocity = yPercentage * RobotConfig.getInstance().getRobotMaxVelocity();

    drivetrain.drive(xVelocity, yVelocity, thetaVelocity, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    Logger.getInstance().recordOutput("ActiveCommands/RotateToAngle", false);
  }

  @Override
  public boolean isFinished() {
    Logger.getInstance().recordOutput("RotateToAngle/tErr", thetaController.atGoal());

    return (running && thetaController.atGoal());
  }

  private static double modifyAxis(double value, double power) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(Math.pow(value, power), value);

    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
