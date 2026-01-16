package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
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
    Current shootMotorTopStatorCurrent = Amps.of(0.0);
    Current shootMotorTopSupplyCurrent = Amps.of(0.0);
    AngularVelocity shootMotorTopVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity shootMotorTopReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity shootMotorTopClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity shootMotorTopClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);
    Temperature shootMotorTopTemperature = Celsius.of(0.0);
    Voltage shootMotorTopVoltage = Volts.of(0.0);

    // Bottom Shooter Motor Inputs
    boolean shootMotorBottomConnected = false;
    Current shootMotorBottomStatorCurrent = Amps.of(0.0);
    Current shootMotorBottomSupplyCurrent = Amps.of(0.0);
    AngularVelocity shootMotorBottomVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity shootMotorBottomReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity shootMotorBottomClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity shootMotorBottomClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);
    Temperature shootMotorBottomTemperature = Celsius.of(0.0);
    Voltage shootMotorBottomVoltage = Volts.of(0.0);

    // Game Piece Detection
    boolean sensorConnected = false;
    boolean hasGamePiece = false;
    Distance distanceToGamePiece = Meters.of(0.0);
    double signalStrength = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterWheelTopVelocity(AngularVelocity velocity) {}

  public default void setShooterWheelBottomVelocity(AngularVelocity velocity) {}

  public default void setShooterWheelTopCurrent(Current amps) {}

  public default void setShooterWheelBottomCurrent(Current amps) {}
}
