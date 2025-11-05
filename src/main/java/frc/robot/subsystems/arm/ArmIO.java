package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.xrp.XRPServo;

public interface ArmIO {
    
    public class ArmIOInputs
    {
        boolean connected = false;
        double angleDegrees = 0.0;
        double servoMotorPosition = 0.0;
    }

    public void setAngle(double angleDegrees);

    public void setPosition(double position);

    public void updateInputs(ArmIOInputs inputs);

}
