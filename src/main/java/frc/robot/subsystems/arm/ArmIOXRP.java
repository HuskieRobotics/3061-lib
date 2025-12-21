package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.xrp.XRPServo;

public class ArmIOXRP implements ArmIO {

  private final XRPServo armMotor = new XRPServo(4);

  private Angle referenceAngle = Degrees.of(0.0);

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.position = Degrees.of(armMotor.getAngle());
    inputs.angleMotorReferenceAngle = referenceAngle;
  }

  @Override
  public void setAngle(Angle angle) {
    armMotor.setAngle(angle.in(Degrees));
  }
}
