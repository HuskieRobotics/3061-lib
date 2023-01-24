// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.swerve;

import static frc.lib.team3061.swerve.SwerveModuleConstants.*;
import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.team6328.util.TunableNumber;

/**
 * A simulated version of the SwerveModuleIO interface.
 *
 * <p>The swerve module is simulated as a flywheel connected to the drive motor and another flywheel
 * connected ot the turn motor.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {

  private final TunableNumber driveKp = new TunableNumber("Drive/DriveKp", SIM_DRIVE_KP);
  private final TunableNumber driveKi = new TunableNumber("Drive/DriveKi", SIM_DRIVE_KI);
  private final TunableNumber driveKd = new TunableNumber("Drive/DriveKd", SIM_DRIVE_KD);
  private final TunableNumber turnKp = new TunableNumber("Drive/TurnKp", SIM_ANGLE_KP);
  private final TunableNumber turnKi = new TunableNumber("Drive/TurnKi", SIM_ANGLE_KI);
  private final TunableNumber turnKd = new TunableNumber("Drive/TurnKd", SIM_ANGLE_KD);

  /* Simulated Angle Motor PID Values */
  private static final double SIM_ANGLE_KP = 12.0;
  private static final double SIM_ANGLE_KI = 0.0;
  private static final double SIM_ANGLE_KD = 0.0;

  /* Simulated Drive Motor PID Values */
  private static final double SIM_DRIVE_KP = 0.8;
  private static final double SIM_DRIVE_KI = 0.0;
  private static final double SIM_DRIVE_KD = 0.0;

  /* Simulated Drive Motor Characterization Values */
  private static final double SIM_DRIVE_KS = 0.116970;
  private static final double SIM_DRIVE_KV = 0.133240;
  private static final double SIM_DRIVE_KA = 0.0;

  private FlywheelSim driveSim =
      new FlywheelSim(DCMotor.getFalcon500(1), MK4_L2_DRIVE_GEAR_RATIO, 0.025);
  private FlywheelSim turnSim =
      new FlywheelSim(DCMotor.getFalcon500(1), MK4_L2_ANGLE_GEAR_RATIO, 0.004096955);

  private double turnRelativePositionRad = 0.0;
  private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private boolean isDriveOpenLoop = true;
  private double driveSetpointMPS = 0.0;
  private double angleSetpointDeg = 0.0;

  private SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(SIM_DRIVE_KS, SIM_DRIVE_KV, SIM_DRIVE_KA);
  private PIDController driveController =
      new PIDController(driveKp.get(), driveKi.get(), driveKd.get());
  private PIDController turnController =
      new PIDController(turnKp.get(), turnKi.get(), turnKd.get());

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // update the models
    driveSim.update(LOOP_PERIOD_SECS);
    turnSim.update(LOOP_PERIOD_SECS);

    // update the inputs that will be logged
    double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * LOOP_PERIOD_SECS;
    turnRelativePositionRad += angleDiffRad;
    turnAbsolutePositionRad += angleDiffRad;
    while (turnAbsolutePositionRad < 0) {
      turnAbsolutePositionRad += 2.0 * Math.PI;
    }
    while (turnAbsolutePositionRad > 2.0 * Math.PI) {
      turnAbsolutePositionRad -= 2.0 * Math.PI;
    }

    inputs.drivePositionDeg =
        inputs.drivePositionDeg
            + (driveSim.getAngularVelocityRadPerSec() * LOOP_PERIOD_SECS * (180.0 / Math.PI));

    inputs.driveDistanceMeters =
        inputs.driveDistanceMeters
            + (driveSim.getAngularVelocityRadPerSec()
                * LOOP_PERIOD_SECS
                * (MK4_L2_WHEEL_CIRCUMFERENCE / (2.0 * Math.PI)));

    inputs.driveVelocityMetersPerSec =
        driveSim.getAngularVelocityRadPerSec() * (MK4_L2_WHEEL_CIRCUMFERENCE / (2.0 * Math.PI));

    inputs.driveAppliedPercentage = driveAppliedVolts / 12.0;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};
    inputs.driveTempCelsius = new double[] {};

    inputs.angleAbsolutePositionDeg = turnAbsolutePositionRad * (180.0 / Math.PI);
    inputs.anglePositionDeg = turnRelativePositionRad * (180.0 / Math.PI);
    inputs.angleVelocityRevPerMin =
        turnSim.getAngularVelocityRadPerSec() * (60.0 / (2.0 * Math.PI));

    inputs.angleAppliedPercentage = turnAppliedVolts / 12.0;
    inputs.angleCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};
    inputs.angleTempCelsius = new double[] {};

    // update the tunable PID constants
    if (driveKp.hasChanged() || driveKi.hasChanged() || driveKd.hasChanged()) {
      driveController.setPID(driveKp.get(), driveKi.get(), driveKd.get());
    }
    if (turnKp.hasChanged() || turnKi.hasChanged() || turnKd.hasChanged()) {
      turnController.setPID(turnKp.get(), turnKi.get(), turnKd.get());
    }

    // calculate and apply the "on-board" controllers for the turn and drive motors
    turnAppliedVolts =
        turnController.calculate(turnRelativePositionRad, angleSetpointDeg * (Math.PI / 180.0));
    turnAppliedVolts = MathUtil.clamp(turnAppliedVolts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);

    if (!isDriveOpenLoop) {
      double velocityRadPerSec = driveSetpointMPS * (2.0 * Math.PI) / (MK4_L2_WHEEL_CIRCUMFERENCE);
      driveAppliedVolts =
          feedForward.calculate(velocityRadPerSec)
              + driveController.calculate(inputs.driveVelocityMetersPerSec, velocityRadPerSec);
      driveAppliedVolts = MathUtil.clamp(driveAppliedVolts, -12.0, 12.0);
      driveSim.setInputVoltage(driveAppliedVolts);
    }
  }

  /** Run the drive motor at the specified percentage of full power. */
  @Override
  public void setDriveMotorPercentage(double percentage) {
    isDriveOpenLoop = true;
    driveController.reset();
    driveAppliedVolts = MathUtil.clamp(percentage * 12.0, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  /** Run the drive motor at the specified velocity. */
  @Override
  public void setDriveVelocity(double velocity) {
    isDriveOpenLoop = false;
    driveSetpointMPS = velocity;
  }

  /** Run the turn motor to the specified angle. */
  @Override
  public void setAnglePosition(double degrees) {

    angleSetpointDeg = degrees;
  }
}
