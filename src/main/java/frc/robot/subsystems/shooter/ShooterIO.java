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
    boolean leadConnected = false;
    Current leadMotorStatorCurrent = Amps.of(0.0);
    Current leadMotorSupplyCurrent = Amps.of(0.0);
    AngularVelocity leadMotorVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity leadMotorReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity leadMotorClosedLoopReferenceVelocity = RotationsPerSecond.of(0.0);
    AngularVelocity leadMotorClosedLoopErrorVelocity = RotationsPerSecond.of(0.0);
    Temperature leadMotorTemp = Celsius.of(0.0);
    Voltage leadMotorVoltage = Volts.of(0.0);

    // Bottom Shooter Motor Inputs
    boolean followerConnected = false;
    Current followerMotorStatorCurrent = Amps.of(0.0);
    Current followerMotorSupplyCurrent = Amps.of(0.0);
    Voltage followerMotorVoltage = Volts.of(0.0);
    Temperature followerMotorTemp = Celsius.of(0.0);

    // Game Piece Detection
    boolean sensorConnected = false;
    boolean hasGamePiece = false;
    Distance distanceToGamePiece = Meters.of(0.0);
    double signalStrength = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVelocity(AngularVelocity velocity) {}

  public default void setShooterCurrent(Current amps) {}
}
