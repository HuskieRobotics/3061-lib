package frc.robot.subsystems.subsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.subsystem.SubsystemConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Models a generic subsystem for a rotational mechanism. The other subsystems defined in this
 * library aren't good examples for typical robot subsystems. This class can serve as an example or
 * be used for quick prototyping.
 */
public class Subsystem extends SubsystemBase {

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Subsystem/TestingMode", 0);
  private final LoggedTunableNumber motorPower = new LoggedTunableNumber("Subsystem/power", 0.0);
  private final LoggedTunableNumber motorCurrent =
      new LoggedTunableNumber("Subsystem/current", 0.0);
  private final LoggedTunableNumber motorPosition =
      new LoggedTunableNumber("Subsystem/position", 0.0);

  private final SubsystemIOInputsAutoLogged inputs = new SubsystemIOInputsAutoLogged();
  private SubsystemIO io;
  private State state = State.A;
  private State lastState = State.UNINITIALIZED;

  /* SysId routine for characterizing the subsystem. This is used to find FF/PID gains for the motor. */
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              null, // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> setMotorVoltage(output.in(Volts)), null, this));

  /**
   * Few subsystems require the complexity of a state machine. A simpler command-based approach is
   * usually better. However, there are times when diagraming and implementing a formal state
   * machine is a reasonable approach. This code is designed to facilitate mapping from a formal
   * state machine diagram to code.
   *
   * <p>The state machine is defined as an enum with each state having its own execute, onEnter, and
   * onExit methods. The execute method is called every iteration of the periodic method. The
   * onEnter and onExit methods are called when the state is entered and exited, respectively.
   * Transitions between states are defined in the execute methods. It is critical that the setState
   * method is only invoked within a state's execute method. Otherwise, it is possible for a state
   * transition to be missed.
   *
   * <p>This approach is modeled after this ChiefDelphi post:
   * https://www.chiefdelphi.com/t/enums-and-subsytem-states/463974/6
   */
  private enum State {
    UNINITIALIZED {
      @Override
      void execute(Subsystem subsystem) {
        /* no-op */
      }

      @Override
      void onEnter(Subsystem subsystem) {
        /* no-op */
      }

      @Override
      void onExit(Subsystem subsystem) {
        /* no-op */
      }
    },
    A {
      @Override
      void onEnter(Subsystem subsystem) {}

      @Override
      void execute(Subsystem subsystem) {
        if (
        /* some condition is */ true) {
          subsystem.setState(State.B);
        }
      }

      @Override
      void onExit(Subsystem subsystem) {}
    },
    B {
      @Override
      void execute(Subsystem subsystem) {
        if (
        /* some condition is */ true) {
          subsystem.setState(State.A);
        } else if (
        /* some other condition is */ true) {
          subsystem.setState(State.C);
        }
      }

      @Override
      void onEnter(Subsystem subsystem) {
        /* no-op */
      }

      @Override
      void onExit(Subsystem subsystem) {
        /* no-op */
      }
    },
    C {
      @Override
      void execute(Subsystem subsystem) {
        if (
        /* some condition is */ true) {
          subsystem.setState(State.A);
        }
      }

      @Override
      void onEnter(Subsystem subsystem) {
        /* no-op */
      }

      @Override
      void onExit(Subsystem subsystem) {
        /* no-op */
      }
    };

    abstract void execute(Subsystem subsystem);

    abstract void onEnter(Subsystem subsystem);

    abstract void onExit(Subsystem subsystem);
  }

  /**
   * Create a new subsystem with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */
  public Subsystem(SubsystemIO io) {

    this.io = io;

    SysIdRoutineChooser.getInstance().addOption("Subsystem Voltage", sysIdRoutine);

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  /**
   * The subsystem's periodic method needs to update and process the inputs from the hardware
   * interface object.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Subsystem", inputs);

    // when testing, set the motor power, current, or position based on the Tunables (if non-zero)
    if (testingMode.get() != 0) {
      if (motorPower.get() != 0) {
        this.setMotorVoltage(motorPower.get());
      }

      if (motorCurrent.get() != 0) {
        this.setMotorCurrent(motorCurrent.get());
      }

      if (motorPosition.get() != 0) {
        this.setMotorPosition(motorPosition.get());
      }
    } else {
      runStateMachine();
    }
  }

  private void setState(State state) {
    this.state = state;
  }

  private void runStateMachine() {
    if (state != lastState) {
      lastState.onExit(this);
      lastState = state;
      state.onEnter(this);
    }

    state.execute(this);
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public void setMotorVoltage(double volts) {
    io.setMotorVoltage(volts);
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  public void setMotorCurrent(double current) {
    io.setMotorCurrent(current);
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public void setMotorPosition(double position) {
    io.setMotorPosition(position, POSITION_FEEDFORWARD);
  }

  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> FaultReporter.getInstance().clearFaults(SUBSYSTEM_NAME)),
            Commands.run(() -> io.setMotorVoltage(3.6)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityRPM < 2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Subsystem motor not moving as fast as expected",
                            false,
                            true);
                  }
                }),
            Commands.run(() -> io.setMotorVoltage(-2.4)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityRPM > -2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Subsystem motor moving too slow or in the wrong direction",
                            false,
                            true);
                  }
                }))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> io.setMotorVoltage(0.0)));
  }
}
