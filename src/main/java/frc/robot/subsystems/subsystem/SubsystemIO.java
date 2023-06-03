package frc.robot.subsystems.subsystem;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface SubsystemIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class SubsystemIOInputs {
    double positionDeg = 0.0;
    double velocityRPM = 0.0;
    double closedLoopError = 0.0;
    double appliedVoltage = 0.0;
    double setpoint = 0.0;
    double power = 0.0;
    String controlMode = "";
    double[] statorCurrentAmps = new double[] {};
    double[] tempCelsius = new double[] {};
    double[] supplyCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SubsystemIOInputs inputs) {}

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
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public default void setMotorPosition(double position, double arbitraryFeedForward) {}
}
