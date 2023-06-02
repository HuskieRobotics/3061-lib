/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import org.littletonrobotics.junction.AutoLog;

@java.lang.SuppressWarnings({"java:S1104"})

/**
 * Gyro hardware abstraction interface
 *
 * <p>The coordinate system for the gyro interface is the same as the Pigeon 2.0's coordinate
 * system: https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
 *
 * <p>The Gyro is modeled not as a subsystem but rather a shared resource that may be used by
 * multiple subsystems. Since it doesn't have a periodic method like a subsystem, it is important
 * that its updateInputs method is called just once via another periodic method. (This requires some
 * coordination, and, in this library, it is invoked via the drivetrain subsystem's periodic
 * method.)
 *
 * <p>There is not a simulated version of this interface. Instead, the drivetrain supports
 * determining the robot's rotation from the gyro when connected and via the swerve module positions
 * when not connected.
 */
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawDeg = 0.0;
    public double yawDegPerSec = 0.0;
    public double pitchDeg = 0.0;
    public double pitchDegPerSec = 0.0;
    public double rollDeg = 0.0;
    public double rollDegPerSec = 0.0;
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(GyroIOInputs inputs) {}
}
