package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.xrp.XRPServo;

public class ArmIOServo implements ArmIO {
    private XRPServo m_armServo;

     /** Creates a new Arm. */
    public ArmIOXRP() {
        m_armServo = new XRPServo(ArmConstants.DEVICE_NUMBER);
    }

}