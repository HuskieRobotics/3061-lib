package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputs inputs;

  private TunableNumber appliedVoltsTunable = new TunableNumber("Shooter/AppliedVolts", 0.0);

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    this.setAppliedVoltage(appliedVoltsTunable.get());
    io.updateInputs(inputs);
  }

  public void setAppliedVoltage(double volts) {
    io.setAppliedVoltage(volts);
  }
}
