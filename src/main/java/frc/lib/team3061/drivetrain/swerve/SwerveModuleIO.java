package frc.lib.team3061.drivetrain.swerve;

import com.ctre.phoenix6.StatusSignal;
import frc.lib.team3061.drivetrain.DrivetrainIO.SwerveIOInputs;
import java.util.ArrayList;
import java.util.List;

/** Swerve module hardware abstraction interface. */
public interface SwerveModuleIO {

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveIOInputs inputs) {}

  /** Run the drive motor at the specified percentage of full power. */
  public default void setDriveMotorVoltage(double voltage) {}

  /** Run the angle motor at the specified percentage of full power. */
  public default void setAngleMotorVoltage(double voltage) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocity) {}

  /** Run the turn motor to the specified angle. */
  public default void setAnglePosition(double degrees) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setAngleBrakeMode(boolean enable) {}

  /**
   * Returns a list of status signals for the swerve module related to odometry. This can be used to
   * synchronize the gyro and swerve modules to improve the accuracy of pose estimation.
   *
   * @return the status signals for the swerve module
   */
  public default List<StatusSignal<Double>> getOdometryStatusSignals() {
    return new ArrayList<>();
  }
}
