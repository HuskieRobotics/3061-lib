package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.xrp.XRPServo;

public interface ArmIO {
    
    @AutoLog
    public class ArmIOInputs
    {
        boolean connected = false;
        double angleDegrees = 0.0;
        double servoMotorPosition = 0.0;
    }

    public void setAngle(double angleDegrees);

    public double getAngle();

    public void setPosition(double position);

    public double getPosition();

}
