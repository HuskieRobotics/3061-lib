package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ElevatorIO {

  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference for each device. The first property is always
  // `connected` and logs if each device is reachable. Due to logging limitations, properties cannot
  // be a subtype of Measure. Therefore all properties are suffix with their unit to mitigate bugs
  // due to unit mismatches.
  @AutoLog
  public static class ElevatorIOInputs {

    boolean connectedLead = false;
    boolean connectedFollower = false;

    double voltageSuppliedLead = 0.0;
    double voltageSuppliedFollower = 0.0;

    double statorCurrentLead = 0.0;
    double statorCurrentFollower = 0.0;
    double supplyCurrentLead = 0.0;
    double supplyCurrentFollower = 0.0;

    double velocityRPS = 0.0;

    double closedLoopErrorRotations = 0.0;
    double closedLoopReferenceRotations = 0.0;

    double linearPositionMeters = 0.0;
    double angularPositionRotations = 0.0;

    double leadTemp = 0.0;
    double followerTemp = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public default void setPositionMeters(double positionMeters) {}

  public default void zeroPosition() {}

  public default void setMotorVoltage(double volts) {}
}
