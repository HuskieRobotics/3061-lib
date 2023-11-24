package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class DrivetrainIOInputs {}

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DrivetrainIOInputs inputs) {}
}
