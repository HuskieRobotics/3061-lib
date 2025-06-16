package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference).
  @AutoLog
  public static class ArmIOInputs {
    double angleMotorStatorCurrentAmps = 0.0;
    double angleMotorSupplyCurrentAmps = 0.0;
    double angleMotorTemperatureCelsius = 0.0;
    double angleMotorVoltage = 0.0;
    double angleDegrees = 0.0;
    double angleMotorReferenceAngleDegrees = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setAngleMotorVoltage(double voltage) {}

  public default void setAngle(double angle) {}
}
