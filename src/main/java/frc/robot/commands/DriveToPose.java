// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Field2d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class DriveToPose extends Command {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private Pose2d targetPose;

  private boolean running = false;
  private Timer timer;

  private static final LoggedTunableNumber driveKp =
      new LoggedTunableNumber(
          "DriveToPose/DriveKp", RobotConfig.getInstance().getDriveToPoseDriveKP());
  private static final LoggedTunableNumber driveKd =
      new LoggedTunableNumber(
          "DriveToPose/DriveKd", RobotConfig.getInstance().getDriveToPoseDriveKD());
  private static final LoggedTunableNumber driveKi =
      new LoggedTunableNumber("DriveToPose/DriveKi", 0);
  private static final LoggedTunableNumber thetaKp =
      new LoggedTunableNumber(
          "DriveToPose/ThetaKp", RobotConfig.getInstance().getDriveToPoseThetaKP());
  private static final LoggedTunableNumber thetaKd =
      new LoggedTunableNumber(
          "DriveToPose/ThetaKd", RobotConfig.getInstance().getDriveToPoseThetaKD());
  private static final LoggedTunableNumber thetaKi =
      new LoggedTunableNumber(
          "DriveToPose/ThetaKi", RobotConfig.getInstance().getDriveToPoseThetaKI());
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber(
          "DriveToPose/DriveMaxVelocityMetersPerSecond",
          RobotConfig.getInstance().getDriveToPoseDriveMaxVelocity().in(MetersPerSecond));
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber(
          "DriveToPose/DriveMaxAccelerationMetersPerSecondPerSecond",
          RobotConfig.getInstance()
              .getDriveToPoseDriveMaxAcceleration()
              .in(MetersPerSecondPerSecond));
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber(
          "DriveToPose/ThetaMaxVelocityRadiansPerSecond",
          RobotConfig.getInstance().getDriveToPoseTurnMaxVelocity().in(RadiansPerSecond));
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber(
          "DriveToPose/ThetaMaxAccelerationRadiansPerSecondPerSecond",
          RobotConfig.getInstance()
              .getDriveToPoseTurnMaxAcceleration()
              .in(RadiansPerSecondPerSecond));
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber(
          "DriveToPose/DriveToleranceMeters",
          RobotConfig.getInstance().getDriveToPoseDriveTolerance().in(Meters));
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber(
          "DriveToPose/ThetaToleranceRadians",
          RobotConfig.getInstance().getDriveToPoseThetaTolerance().in(Radians));
  private static final LoggedTunableNumber timeout =
      new LoggedTunableNumber("DriveToPose/timeout", 5.0);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          driveKp.get(),
          driveKi.get(),
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          driveKp.get(),
          driveKi.get(),
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
          LOOP_PERIOD_SECS);

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToPose(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier) {
    this.drivetrain = drivetrain;
    this.poseSupplier = poseSupplier;
    this.timer = new Timer();
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * This method is invoked once when this command is scheduled. It resets all the PID controllers
   * and initializes the current and target poses. It is critical that this initialization occurs in
   * this method and not the constructor as this object is constructed well before the command is
   * scheduled and the robot's pose will definitely have changed and the target pose may not be
   * known until this command is scheduled.
   */
  @Override
  public void initialize() {
    // Reset all controllers
    Pose2d currentPose = drivetrain.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
    xController.setTolerance(driveTolerance.get());
    yController.setTolerance(driveTolerance.get());
    thetaController.setTolerance(thetaTolerance.get());
    this.targetPose = poseSupplier.get();

    Logger.recordOutput("DriveToPose/targetPose", targetPose);

    this.timer.restart();
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    // set running to true in this method to capture that the calculate method has been invoked on
    // the PID controllers. This is important since these controllers will return true for atGoal if
    // the calculate method has not yet been invoked.
    running = true;

    // Update from tunable numbers
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          xController.setPID(pid[0], pid[1], pid[2]);
          yController.setPID(pid[0], pid[1], pid[2]);
        },
        driveKp,
        driveKi,
        driveKd);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        max -> {
          xController.setConstraints(new TrapezoidProfile.Constraints(max[0], max[1]));
          yController.setConstraints(new TrapezoidProfile.Constraints(max[0], max[1]));
        },
        driveMaxVelocity,
        driveMaxAcceleration);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        tolerance -> {
          xController.setTolerance(tolerance[0]);
          yController.setTolerance(tolerance[0]);
        },
        driveTolerance);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> thetaController.setPID(pid[0], pid[1], pid[2]),
        thetaKp,
        thetaKi,
        thetaKd);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        max -> thetaController.setConstraints(new TrapezoidProfile.Constraints(max[0], max[1])),
        thetaMaxVelocity,
        thetaMaxAcceleration);
    LoggedTunableNumber.ifChanged(
        hashCode(), tolerance -> thetaController.setTolerance(tolerance[0]), thetaTolerance);

    Pose2d currentPose = drivetrain.getPose();

    double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());
    if (xController.atGoal()) xVelocity = 0.0;
    if (yController.atGoal()) yVelocity = 0.0;
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    int allianceMultiplier = Field2d.getInstance().getAlliance() == Alliance.Blue ? 1 : -1;

    drivetrain.drive(
        allianceMultiplier * xVelocity, allianceMultiplier * yVelocity, thetaVelocity, true, true);
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). This command is considered finished if the
   * move-to-pose feature is disabled on the drivetrain subsystem or if the timeout has elapsed or
   * if all the PID controllers are at their goal.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    Logger.recordOutput("DriveToPose/xErr", xController.atGoal());
    Logger.recordOutput("DriveToPose/yErr", yController.atGoal());
    Logger.recordOutput("DriveToPose/tErr", thetaController.atGoal());

    // check that running is true (i.e., the calculate method has been invoked on the PID
    // controllers) and that each of the controllers is at their goal. This is important since these
    // controllers will return true for atGoal if the calculate method has not yet been invoked.
    return !drivetrain.isMoveToPoseEnabled()
        || this.timer.hasElapsed(timeout.get())
        || (running && xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    running = false;
  }
}
