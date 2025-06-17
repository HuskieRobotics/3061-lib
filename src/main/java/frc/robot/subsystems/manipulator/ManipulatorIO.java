package frc.robot.subsystems.manipulator;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ManipulatorIO {
  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference). The first property is always `connected` and logs if
  // each device is reachable. Due to logging limitations, properties cannot be a subtype of
  // Measure. Therefore all properties are suffix with their unit to mitigate bugs due to unit
  // mismatches.
  @AutoLog
  public static class ManipulatorIOInputs {

    boolean manipulatorConnected = false;
    boolean isManipulatorPrimaryIRBlocked = false;
    boolean isManipulatorSecondaryIRBlocked = false;
    double manipulatorStatorCurrentAmps = 0;
    double manipulatorSupplyCurrentAmps = 0;
    double manipulatorTempCelsius = 0;
    double manipulatorMotorVoltage = 0;
    double manipulatorVelocityRPS = 0;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setManipulatorVoltage(Voltage volts) {}
}
