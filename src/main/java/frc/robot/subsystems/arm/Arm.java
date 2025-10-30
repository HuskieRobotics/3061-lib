package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.xrp.XRPServo;

public class Arm {
    private ArmIOServo io;
    //private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public Arm(ArmIOServo armIO)
    {
        this.io = armIO;
    }

    // @Override
    // public void periodic()
    // {
    //     io.updateInputs(inputs);
    //     Logger.processInputs(SUBSYSTEM_NAME, inputs);

    // }

    public void setAngle(double angleDegrees)
    {
        io.setAngle(angleDegrees);
    }

    public double getAngle()
    {
        return io.getAngle();
    }

    public double getPosition()
    {
        return io.getPosition();
    }

    public void setPosition(double position)
    {
        io.setPosition(position);
    }

}   
