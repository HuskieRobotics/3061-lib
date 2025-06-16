package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    double angleMotorStatorCurrentAmps = 0.0;
    double angleMotorSupplyCurrentAmps = 0.0;
    double angleMotorVoltage = 0.0;
    double angleMotorReferenceAngleDegrees = 0.0;
    double angleMotorClosedLoopReferenceDegrees = 0.0;
    double angleEncoderAngleDegrees = 0.0;
    double angleMotorTemperatureCelsius = 0.0;
    double angleMotorClosedLoopReferenceSlope = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setAngleMotorVoltage(double voltage) {}

  public default void setAngle(double angle) {}
}
