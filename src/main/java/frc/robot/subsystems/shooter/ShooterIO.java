package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  // The inputs class for a subsystem usually contains the stator and supply currents, temperature,
  // voltage (or current), and, depending on the control mode, additional fields related to position
  // or velocity (both measured and reference). The first property is always `connected` and logs if
  // each device is reachable. Due to logging limitations, properties cannot be a subtype of
  // Measure. Therefore all properties are suffix with their unit to mitigate bugs due to unit
  // mismatches.
  @AutoLog
  public static class ShooterIOInputs {

    // Top Shooter Motor Inputs
    boolean shootMotorTopConnected = false;
    double shootMotorTopStatorCurrent = 0.0;
    double shootMotorTopSupplyCurrent = 0.0;
    double shootMotorTopVelocityRPS = 0.0;
    double shootMotorTopReferenceVelocityRPS = 0.0;
    double shootMotorTopClosedLoopReferenceVelocityRPS = 0.0;
    double shootMotorTopClosedLoopErrorVelocityRPS = 0.0;
    double shootMotorTopTemperature = 0.0;
    double shootMotorTopVoltage = 0.0;

    // Bottom Shooter Motor Inputs
    boolean shootMotorBottomConnected = false;
    double shootMotorBottomStatorCurrent = 0.0;
    double shootMotorBottomSupplyCurrent = 0.0;
    double shootMotorBottomVelocityRPS = 0.0;
    double shootMotorBottomReferenceVelocityRPS = 0.0;
    double shootMotorBottomClosedLoopReferenceVelocityRPS = 0.0;
    double shootMotorBottomClosedLoopErrorVelocityRPS = 0.0;
    double shootMotorBottomTemperature = 0.0;
    double shootMotorBottomVoltage = 0.0;

    // Game Piece Detection
    boolean sensorConnected = false;
    boolean hasGamePiece = false;
    double distanceToGamePieceMeters = 0.0;
    double signalStrength = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterWheelTopVelocityRPS(double velocity) {}

  public default void setShooterWheelBottomVelocityRPS(double velocity) {}

  public default void setShooterWheelTopCurrent(double amps) {}

  public default void setShooterWheelBottomCurrent(double amps) {}
}
