package frc.robot.subsystems.arm;

public interface ArmIO {

  public class ArmIOInputs {
    // boolean connected = false;
    public double angleDegrees = 0.0;
    public double servoMotorPosition = 0.0;
  }

  public void setAngle(double angleDegrees);

  public void setPosition(double position);

  public void updateInputs(ArmIOInputs inputs);
}
