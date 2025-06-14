package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase {
  private ArmIO io;

  private final ArmIOInputs inputs = new ArmIOInputs();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void goToAngle(double angleDeg) {
    io.setAngle(angleDeg);
  }

  public double getAngle() {
    return inputs.positionDeg;
  }
}
