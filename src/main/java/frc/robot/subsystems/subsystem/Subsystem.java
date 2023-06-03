package frc.robot.subsystems.subsystem;

import static frc.robot.subsystems.subsystem.SubsystemConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Models a generic subsystem for a rotational mechanism. The other subsystems defined in this
 * library aren't good examples for typical robot subsystems. This class can serve as an example or
 * be used for quick prototyping.
 */
public class Subsystem extends SubsystemBase {

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  private final TunableNumber motorPower = new TunableNumber("Subsystem/power", 0.0);
  private final TunableNumber motorCurrent = new TunableNumber("Subsystem/current", 0.0);
  private final TunableNumber motorPosition = new TunableNumber("Subsystem/position", 0.0);

  private final SubsystemIOInputsAutoLogged inputs = new SubsystemIOInputsAutoLogged();
  private SubsystemIO io;

  /**
   * Create a new subsystem with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */
  public Subsystem(SubsystemIO io) {

    this.io = io;

    // Create a Shuffleboard tab for this subsystem if testing is enabled. Add additional indicators
    // and controls as needed.
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }
  }

  /**
   * The subsystem's periodic method needs to update and process the inputs from the hardware
   * interface object.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Subsystem", inputs);

    // when testing, set the motor power, current, or position based on the Tunables (if non-zero)
    if (TESTING) {
      if (motorPower.get() != 0) {
        this.setMotorPower(motorPower.get());
      }

      if (motorCurrent.get() != 0) {
        this.setMotorCurrent(motorCurrent.get());
      }

      if (motorPosition.get() != 0) {
        this.setMotorPosition(motorPosition.get());
      }
    }
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public void setMotorPower(double power) {
    io.setMotorPower(power);
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  public void setMotorCurrent(double power) {
    io.setMotorCurrent(power);
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public void setMotorPosition(double position) {
    io.setMotorPosition(position, POSITION_FEEDFORWARD);
  }
}
