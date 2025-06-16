package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {

    // Top Shooter Motor Inputs
    double shootMotorTopStatorCurrentAmps = 0.0;
    double shootMotorTopSupplyCurrentAmps = 0.0;
    double shootMotorTopVelocityRPS = 0.0;
    double shootMotorTopReferenceVelocityRPS = 0.0;
    double shootMotorTopClosedLoopReferenceRPS = 0.0;
    double shootMotorTopTemperatureCelsius = 0.0;
    double shootMotorTopVoltage = 0.0;

    // Bottom Shooter Motor Inputs
    double shootMotorBottomStatorCurrentAmps = 0.0;
    double shootMotorBottomSupplyCurrentAmps = 0.0;
    double shootMotorBottomVelocityRPS = 0.0;
    double shootMotorBottomReferenceVelocityRPS = 0.0;
    double shootMotorBottomClosedLoopReferenceRPS = 0.0;
    double shootMotorBottomTemperatureCelsius = 0.0;
    double shootMotorBottomVoltage = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterWheelTopVelocity(double rps) {}

  public default void setShooterWheelBottomVelocity(double rps) {}
}
