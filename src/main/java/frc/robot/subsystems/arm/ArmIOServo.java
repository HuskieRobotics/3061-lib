package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.xrp.XRPServo;

public class ArmIOServo implements ArmIO
{
    XRPServo armMotor;
    
    public ArmIOServo(int deviceNum)
    {
        armMotor = new XRPServo(deviceNum);
    }

    public void setAngle(double angleDegrees)
    {
        armMotor.setAngle(angleDegrees);
    }

    public void setPosition(double position)
    {
        armMotor.setPosition(position);
    } 

    public void updateInputs(ArmIOInputs inputs)
    {
        ar
    }
}