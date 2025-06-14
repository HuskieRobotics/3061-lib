package frc.robot.subsystems.manipulator;

import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.States;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Models a generic subsystem for a rotational mechanism. The other subsystems defined in this
 * library aren't good examples for typical robot subsystems. This class can serve as an example or
 * be used for quick prototyping.
 */
public class Manipulator extends SubsystemBase {

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Manipulator/TestingMode", 0);

  private final LoggedTunableNumber manipulatorMotorVoltage =
      new LoggedTunableNumber("Manipulator/MotorVoltage", 0);

  public final LoggedTunableNumber manipulatorCollectionVoltage =
      new LoggedTunableNumber("Manipulator/CollectionVoltage", MANIPULATOR_COLLECTION_VOLTAGE);

  public final LoggedTunableNumber manipulatorReleaseVoltage =
      new LoggedTunableNumber("Manipulator/ReleaseVoltage", MANIPULATOR_RELEASE_VOLTAGE);

  public final LoggedTunableNumber manipulatorEjectVoltage =
      new LoggedTunableNumber("Manipulator/Indexer/EjectVoltage", MANIPULATOR_EJECT_VOLTAGE);

  private ManipulatorIO io;
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();
  private State state = State.WAITING_FOR_GAME_PIECE;
  private State lastState = State.UNINITIALIZED;

  // create a timer to track how long is spent in this stage
  Timer inIndexingState = new Timer();
  Timer ejectingTimer = new Timer();

  // the first value is the time constant, the characteristic timescale of the
  // filter's impulse response, and the second value is the time-period, how often
  // the calculate() method will be called
  private LinearFilter currentInAmps = LinearFilter.singlePoleIIR(0.1, 0.02);

  private boolean releaseButtonPressed = false;

  /**
   * Create a new subsystem with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */
  public Manipulator(ManipulatorIO io) {

    this.io = io;

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

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
   * <p>Our best practice is set the voltage/current/velocity/position of each device in the onEnter
   * method of each state. This simplifies needing to keep track of which states could have been the
   * previous states and the associated states of these devices.
   *
   * <p>This approach is modeled after this ChiefDelphi post:
   * https://www.chiefdelphi.com/t/enums-and-subsytem-states/463974/6
   */
  private enum State {
    WAITING_FOR_GAME_PIECE {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(subsystem.manipulatorCollectionVoltage.get());
      }

      @Override
      void execute(Manipulator subsystem) {

        LEDs.getInstance().requestState(States.WAITING_FOR_GAME_PIECE);

        if (DriverStation.isDisabled() && subsystem.inputs.isManipulatorIRBlocked) {
          subsystem.setState(State.GAME_PIECE_IN_MANIPULATOR);
        } else if (subsystem.inputs.isManipulatorIRBlocked) {
          subsystem.setState(State.INDEXING_GAME_PIECE_IN_MANIPULATOR);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    INDEXING_GAME_PIECE_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(subsystem.manipulatorCollectionVoltage.get());
        subsystem.inIndexingState.restart(); // restart timer
        subsystem.currentInAmps
            .reset(); // reset the linear filter thats used to detect a current spike
      }

      @Override
      void execute(Manipulator subsystem) {

        LEDs.getInstance().requestState(States.INDEXING_GAME_PIECE);

        // the currentInAmps filters out the current in the noise and getting the lastValue gets the
        // last value of the current, and if that last value is greater than some constant, then
        // current spike has been detected
        if (subsystem.inputs.isManipulatorIRBlocked
            && subsystem.currentInAmps.lastValue() > COLLECTION_CURRENT_SPIKE_THRESHOLD) {
          subsystem.setState(State.GAME_PIECE_IN_MANIPULATOR);
        } else if (subsystem.inIndexingState.hasElapsed(COLLECTION_TIME_OUT)) {
          subsystem.setState(GAME_PIECE_STUCK);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    GAME_PIECE_STUCK {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(subsystem.manipulatorEjectVoltage.get());
        subsystem.ejectingTimer.restart();
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.EJECTING_GAME_PIECE);

        if (subsystem.inputs.isManipulatorIRBlocked) {
          subsystem.setState(State.INDEXING_GAME_PIECE_IN_MANIPULATOR);
        } else if (!subsystem.inputs.isManipulatorIRBlocked
            && subsystem.ejectingTimer.hasElapsed(FINAL_EJECT_DURATION_SECONDS)) {
          subsystem.setState(State.WAITING_FOR_GAME_PIECE);
        } else if (subsystem.ejectingTimer.hasElapsed(SECOND_INTAKE_SECONDS)) {
          subsystem.setManipulatorMotorVoltage(subsystem.manipulatorEjectVoltage.get());
        } else if (subsystem.ejectingTimer.hasElapsed(FIRST_EJECT_DURATION_SECONDS)) {
          subsystem.setManipulatorMotorVoltage(subsystem.manipulatorCollectionVoltage.get());
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },
    GAME_PIECE_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {}

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.HAS_GAME_PIECE);

        if (subsystem.releaseButtonPressed) {
          subsystem.setState(State.RELEASE_GAME_PIECE);
          subsystem.releaseButtonPressed = false;
        } else if (!subsystem.inputs.isManipulatorIRBlocked) {
          subsystem.setState(State.WAITING_FOR_GAME_PIECE);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {
        /*NO-OP */
      }
    },
    RELEASE_GAME_PIECE {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(subsystem.manipulatorReleaseVoltage.get());
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.RELEASING_GAME_PIECE);

        if (!subsystem.inputs.isManipulatorIRBlocked) {
          subsystem.setState(State.WAITING_FOR_GAME_PIECE);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    UNINITIALIZED {

      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(0);
      }

      @Override
      void execute(Manipulator subsystem) {
        subsystem.setState(
            State.WAITING_FOR_GAME_PIECE); // default state to WAITING_FOR_CORAL_IN_FUNNEL state
      }

      @Override
      void onExit(Manipulator subsystem) {
        /*NO-OP */
      }
    };

    abstract void execute(Manipulator subsystem);

    abstract void onEnter(Manipulator subsystem);

    abstract void onExit(Manipulator subsystem);
  }

  /**
   * The subsystem's periodic method needs to update and process the inputs from the hardware
   * interface object.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Manipulator", inputs);
    Logger.recordOutput(SUBSYSTEM_NAME + "/State", this.state);

    currentInAmps.calculate(inputs.manipulatorStatorCurrentAmps);

    // when testing, set the indexer motor power, current, or position based on the Tunables (if
    // non-zero)
    if (testingMode.get() == 1) {
      if (manipulatorMotorVoltage.get() != 0) {
        setManipulatorMotorVoltage(manipulatorMotorVoltage.get());
      }
    } else {
      runStateMachine();
    }

    // Record cycle time
    LoggedTracer.record("Manipulator");
  }

  private void setState(State state) {
    this.state = state;
  }

  public void resetStateMachine() {
    this.state = State.WAITING_FOR_GAME_PIECE;
  }

  private void runStateMachine() {
    if (state != lastState) {
      lastState.onExit(this);
      lastState = state;
      state.onEnter(this);
    }

    state.execute(this);
  }

  private void setManipulatorMotorVoltage(double volts) {
    io.setManipulatorVoltage(volts);
  }

  // Whichever line of code does something with the motors, i replaced it with 2 lines that do the
  // same exact thing but for the funnel and indexer motor, unsure if this is correct
  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> FaultReporter.getInstance().clearFaults(SUBSYSTEM_NAME)),
            Commands.run(() -> io.setManipulatorVoltage(3.6)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.manipulatorVelocityRPS < 2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Manipulator motor not moving as fast as expected",
                            false,
                            true);
                  }
                }),
            Commands.run(() -> io.setManipulatorVoltage(-2.4)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.manipulatorVelocityRPS > -2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Manipulator motor moving too slow or in the wrong direction",
                            false,
                            true);
                  }
                }))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> io.setManipulatorVoltage(0.0)));
  }

  public void releaseGamePiece() {
    releaseButtonPressed = true;
  }

  public boolean indexingGamePiece() {
    return state == State.INDEXING_GAME_PIECE_IN_MANIPULATOR;
  }

  public boolean hasIndexedGamePiece() {
    return state == State.GAME_PIECE_IN_MANIPULATOR;
  }
}
