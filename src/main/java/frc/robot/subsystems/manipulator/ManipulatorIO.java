package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ManipulatorIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ManipulatorIOInputs {

    boolean isManipulatorIRBlocked = false;
    boolean manipulatorConnected = false;
    double manipulatorStatorCurrentAmps = 0;
    double manipulatorSupplyCurrentAmps = 0;
    double manipulatorTempCelsius = 0;
    double manipulatorMotorVoltage = 0;
    double manipulatorVelocityRPS = 0;
  }

  public default void updateInputs(ManipulatorIOInputs inputs) {}

  public default void setManipulatorVoltage(double volts) {}
}
