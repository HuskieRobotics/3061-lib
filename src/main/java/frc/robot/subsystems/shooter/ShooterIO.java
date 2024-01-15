package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ShooterIOInputs {
    double appliedVoltage = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public default void setMotorPower(double power) {}

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  public default void setMotorCurrent(double current) {}

  /**
   * Set the applied voltage to the specified value in volts
   *
   * @param voltage
   */
  public default void setAppliedVoltage(double volts) {}

  public default void setRightMotor(double volts) {}

  public default void setLeftMotor(double volts) {}

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public default void setMotorPosition(double position, double arbitraryFeedForward) {}
}
