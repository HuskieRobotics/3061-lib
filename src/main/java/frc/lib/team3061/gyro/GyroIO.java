/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
 * coordination.)
 */
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Angle yaw = Rotations.of(0.0);
    public AngularVelocity yawVelocity = RotationsPerSecond.of(0.0);
    public Angle pitch = Rotations.of(0.0);
    public AngularVelocity pitchVelocity = RotationsPerSecond.of(0.0);
    public Angle roll = Rotations.of(0.0);
    public AngularVelocity rollVelocity = RotationsPerSecond.of(0.0);
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(GyroIOInputs inputs) {}
}
