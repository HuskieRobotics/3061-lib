/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import org.littletonrobotics.junction.AutoLog;

@java.lang.SuppressWarnings({"java:S1104"})

/**
 * Gyro hardware abstraction interface
 *
 * <p>The Gyro is modeled not as a subsystem but rather a shared resource that may be used by
 * multiple subsystems. Since it doesn't have a periodic method like a subsystem, it is important
 * that its updateInputs method is called just once via another periodic method. (This requires some
 * coordination, and, in this library, it is invoked via the drivetrain subsystem's periodic
 * mehtod.)
 *
 * <p>There is not a simulated version of this interface. Instead, the drivetrain supports
 * determining the robot's rotation from the gryo when connected and via the swwerve module
 * positions when not connected.
 */
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double positionDeg = 0.0;
    public double velocityDegPerSec = 0.0;
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(GyroIOInputs inputs) {}
}
