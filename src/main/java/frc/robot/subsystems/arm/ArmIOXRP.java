package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.xrp.XRPServo;

public class ArmIOXRP implements ArmIO {

  private final XRPServo armMotor = new XRPServo(4);

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionDeg = armMotor.getAngle();
  }

  @Override
  public void setAngle(double angleDeg) {
    armMotor.setAngle(angleDeg);
  }
}
