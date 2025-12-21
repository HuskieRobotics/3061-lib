package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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

    Voltage voltageSuppliedLead = Volts.of(0.0);
    Voltage voltageSuppliedFollower = Volts.of(0.0);

    Current statorCurrentLead = Amps.of(0.0);
    Current statorCurrentFollower = Amps.of(0.0);
    Current supplyCurrentLead = Amps.of(0.0);
    Current supplyCurrentFollower = Amps.of(0.0);

    AngularVelocity velocity = RotationsPerSecond.of(0.0);

    Angle closedLoopError = Rotations.of(0.0);
    Angle closedLoopReference = Rotations.of(0.0);

    Distance linearPosition = Meters.of(0.0);
    Angle angularPosition = Rotations.of(0.0);

    Temperature leadTemp = Celsius.of(0.0);
    Temperature followerTemp = Celsius.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public default void setPosition(Distance position) {}

  public default void zeroPosition() {}

  public default void setMotorVoltage(Voltage volts) {}
}
