package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputs inputs;
  private boolean controlBoth = false;

  private TunableNumber velocityTunable = new TunableNumber("Shooter/Velocity", 0.0);
  private TunableNumber velocityRightTunable = new TunableNumber("Shooter/VelocityTop", 0.0);
  private TunableNumber velocityLeftTunable = new TunableNumber("Shooter/VelocityBottom", 0.0);
  private TunableNumber 

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
      this.setVelocity(velocityTunable.get());
      io.updateInputs(inputs);
    } else {
      this.setRightMotor(velocityRightTunable.get());
      this.setLeftMotor(velocityLeftTunable.get());
    }
  }

  public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void setKickerVelocity(double velocity) {
    io.setKickerMotor(velocity);
  }

  public void setRightMotor(double velocity) {
    io.setTopMotor(velocity);
  }

  public void setLeftMotor(double velocity) {
    io.setBottomMotor(velocity);
  }
}
