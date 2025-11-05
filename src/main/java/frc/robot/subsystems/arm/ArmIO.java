package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    
    public static class ArmIOInputs {
        double servoAngleDegrees = 0.0;

    }

    public default void updateInputs(ArmIOInputs inputs) {}

    /**
    * Set the current angle of the arm (0 - 180 degrees).
    *
    * @param angleDeg Desired arm angle in degrees
    */
    public default void setAngle(double angleDeg) {}


}