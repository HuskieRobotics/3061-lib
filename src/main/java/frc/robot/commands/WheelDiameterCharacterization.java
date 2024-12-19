// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class WheelDiameterCharacterization extends Command {
  private static final TunableNumber characterizationSpeed =
      new TunableNumber("WheelDiameterCharacterization/SpeedRadsPerSec", 0.5);
  private static final double DRIVE_RADIUS =
      Math.hypot(
          RobotConfig.getInstance().getTrackwidth() / 2.0,
          RobotConfig.getInstance().getWheelbase() / 2.0);
  private static final DoubleSupplier gyroYawRadsSupplier =
      () -> RobotOdometry.getInstance().getEstimatedPose().getRotation().getRadians();

  private final Drivetrain drivetrain;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumulatedGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelDiameterCharacterization(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumulatedGyroYawRads = 0.0;

    startWheelPositions = drivetrain.getWheelDiameterCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drivetrain.runWheelDiameterCharacterization(
        omegaLimiter.calculate(characterizationSpeed.get()));

    // Get yaw and wheel positions
    accumulatedGyroYawRads +=
        MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositions = drivetrain.getWheelDiameterCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumulatedGyroYawRads * DRIVE_RADIUS) / averageWheelPosition;
    Logger.recordOutput("WheelDiameterCharacterization/DrivePosition", averageWheelPosition);
    Logger.recordOutput(
        "WheelDiameterCharacterization/AccumulatedGyroYawRads", accumulatedGyroYawRads);
    Logger.recordOutput(
        "WheelDiameterCharacterization/CurrentWheelDiameterMeters",
        currentEffectiveWheelRadius * 2.0);
  }

  @Override
  public void end(boolean interrupted) {
    if (accumulatedGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Diameter: " + currentEffectiveWheelRadius * 2.0 + " meters");
    }
  }
}
