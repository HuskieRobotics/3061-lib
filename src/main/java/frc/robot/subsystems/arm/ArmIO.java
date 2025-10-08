package frc.robot.subsystems.arm;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    public default void setAngle(double angle) {}

    public default void setPosition(Distance position) {}
}
