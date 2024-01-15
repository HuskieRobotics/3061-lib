package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputs inputs;
  private boolean controlBoth = false;

  private TunableNumber appliedVoltsTunable = new TunableNumber("Shooter/AppliedVolts", 0.0);
  private TunableNumber appliedVoltsRightTunable =
      new TunableNumber("Shooter/AppliedVoltsRight", 0.0);
  private TunableNumber appliedVoltsLeftTunable =
      new TunableNumber("Shooter/AppliedVoltsLeft", 0.0);

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputs();

    // ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    // // shooterTab.addNumber("Applied Volts", () -> appliedVoltsTunable.get());
    // shooterTab.add("Applied Volts", appliedVoltsTunable).getEntry();
  }

  @Override
  public void periodic() {
    if (controlBoth) {
      this.setAppliedVoltage(appliedVoltsTunable.get());
      io.updateInputs(inputs);
    } else {
      this.setRightMotor(appliedVoltsRightTunable.get());
      this.setLeftMotor(appliedVoltsLeftTunable.get());
    }
  }

  public void setAppliedVoltage(double volts) {
    io.setAppliedVoltage(volts);
  }

  public void setRightMotor(double volts) {
    io.setRightMotor(volts);
  }

  public void setLeftMotor(double volts) {
    io.setLeftMotor(volts);
  }
}
