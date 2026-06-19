package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.xrp.XRPServo;

public class ArmIOXRP implements ArmIO {

  private final XRPServo armMotor = new XRPServo(4);

  private double referenceAngleRotations = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.positionRotations = Units.degreesToRotations(armMotor.getAngle());
    inputs.angleMotorReferenceAngleRotations = Units.degreesToRotations(referenceAngleRotations);
  }

  @Override
  public void setAngleRotations(double angleRotations) {
    armMotor.setAngle(Units.rotationsToDegrees(angleRotations));
  }
}
