package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference). The first property is always `connected` and logs if
  // each device is reachable. Due to logging limitations, properties cannot be a subtype of
  // Measure. Therefore all properties are suffix with their unit to mitigate bugs due to unit
  // mismatches.
  @AutoLog
  public static class ArmIOInputs {
    boolean connected = false;
    Current angleMotorStatorCurrent = Amps.of(0.0);
    Current angleMotorSupplyCurrent = Amps.of(0.0);
    Temperature angleMotorTemperature = Celsius.of(0.0);
    Voltage angleMotorVoltage = Volts.of(0.0);
    Angle position = Rotations.of(0.0);
    Angle angleMotorReferenceAngle = Rotations.of(0.0);
    Angle angleMotorClosedLoopReferenceAngle = Rotations.of(0.0);
    Angle angleMotorClosedLoopErrorAngle = Rotations.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setAngleMotorVoltage(Voltage voltage) {}

  public default void setAngle(Angle angle) {}
}
