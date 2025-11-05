package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.subsystems.arm.ArmIO;

public class Arm extends SubsystemBase {
    private ArmIOServo io;
    private final ArmIOInputs inputs = new ArmIOInputs();

    public Arm(ArmIOServo armIO)
    {
        this.io = armIO;
    }

    @Override
    public void periodic()
    {
        io.updateInputs(inputs);
        //Logger.processInputs(SUBSYSTEM_NAME, inputs);

    }

    public double getAngle()
    {
        return inputs.angleDegrees;
    }

    public double getPosition()
    {
        return inputs.servoMotorPosition;
    }

    public void goToPosition(double position)
    {
        inputs.servoMotorPosition = position;
    }

    public void goToAngle(double angle)
    {
        inputs.angleDegrees = angle;
    }

}   
