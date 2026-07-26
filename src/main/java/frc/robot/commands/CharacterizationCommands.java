// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import org.littletonrobotics.junction.Logger;

@java.lang.SuppressWarnings({"java:S106"})
public class CharacterizationCommands {
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.5; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  // Minimum average wheel travel (radians) before the running estimate is logged. Until the
  // wheels have actually moved, wheelDelta is ~0 and the computed radius is Infinity/NaN.
  private static final double WHEEL_DELTA_LOGGING_THRESHOLD = 1e-6;

  private static final double DRIVE_RADIUS =
      Math.hypot(
          RobotConfig.getInstance().getTrackwidthMeters() / 2.0,
          RobotConfig.getInstance().getWheelbaseMeters() / 2.0);

  private CharacterizationCommands() {}

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(SwerveDrivetrain drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.drive(0.0, 0.0, speed, true, false);
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPosition();
                  state.lastAngle = Rotation2d.fromDegrees(drive.getYawDeg());
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta and log the running estimate
            Commands.run(
                    () -> {
                      var rotation = Rotation2d.fromDegrees(drive.getYawDeg());
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;

                      WheelRadiusResult result =
                          computeResult(
                              state.positions,
                              drive.getWheelRadiusCharacterizationPosition(),
                              state.gyroDelta);

                      // Skip logging until the wheels have moved, to avoid publishing
                      // Infinity/NaN on the first loops (wheelDelta ~ 0).
                      if (result.wheelDelta() > WHEEL_DELTA_LOGGING_THRESHOLD) {
                        Logger.recordOutput(
                            "WheelRadiusCharacterization/WheelDelta", result.wheelDelta());
                        Logger.recordOutput(
                            "WheelRadiusCharacterization/WheelRadiusMeters",
                            result.wheelRadiusMeters());
                        Logger.recordOutput(
                            "WheelRadiusCharacterization/WheelRadiusInches",
                            Units.metersToInches(result.wheelRadiusMeters()));
                      }
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      WheelRadiusResult result =
                          computeResult(
                              state.positions,
                              drive.getWheelRadiusCharacterizationPosition(),
                              state.gyroDelta);

                      NumberFormat formatter = new DecimalFormat("#0.000000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(result.wheelDelta()) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(result.gyroDelta()) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(result.wheelRadiusMeters())
                              + " meters, "
                              + formatter.format(Units.metersToInches(result.wheelRadiusMeters()))
                              + " inches");
                    })));
  }

  /**
   * Computes the average wheel travel (radians) since the starting positions and the resulting
   * wheel radius (meters) implied by the accumulated gyro rotation.
   */
  private static WheelRadiusResult computeResult(
      double[] startPositions, double[] endPositions, double gyroDelta) {
    double wheelDelta = 0.0;
    for (int i = 0; i < 4; i++) {
      wheelDelta += Math.abs(endPositions[i] - startPositions[i]) / 4.0;
    }
    double wheelRadiusMeters = (gyroDelta * DRIVE_RADIUS) / wheelDelta;
    return new WheelRadiusResult(wheelDelta, gyroDelta, wheelRadiusMeters);
  }

  private record WheelRadiusResult(double wheelDelta, double gyroDelta, double wheelRadiusMeters) {}

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
