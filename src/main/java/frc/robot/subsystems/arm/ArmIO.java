package frc.robot.subsystems.arm;

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
    double angleMotorStatorCurrent = 0.0;
    double angleMotorSupplyCurrent = 0.0;
    double angleMotorTemperature = 0.0;
    double angleMotorVoltage = 0.0;
    double positionRotations = 0.0;
    double angleMotorReferenceAngleRotations = 0.0;
    double angleMotorClosedLoopReferenceAngleRotations = 0.0;
    double angleMotorClosedLoopErrorAngleRotations = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setAngleMotorVoltage(double voltage) {}

  public default void setAngleRotations(double angle) {}
}
