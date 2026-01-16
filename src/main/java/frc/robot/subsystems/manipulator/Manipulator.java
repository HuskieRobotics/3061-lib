package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.States;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OISelector;
import org.littletonrobotics.junction.Logger;

/**
 * Example subsystem for controlling an intake or manipulator mechanism.
 *
 * <p>WARNING: This code is for example purposes only. It will not work with a physical manipulator
 * mechanism without changes. While it is derived from Huskie Robotics 2025 manipulator, it has been
 * simplified to highlight select best practices.
 *
 * <p>This example illustrates the following features:
 *
 * <ul>
 *   <li>Use of a state machine to model a sophisticated mechanism
 *   <li>Use of a sensor, with redundancy, to detect the presence of a game piece
 *   <li>AdvantageKit support for logging and replay
 *   <li>Use of a current filters to determine if the game piece has stalled against the hard stop
 *   <li>Use of logged tunable numbers for manual control and testing
 *   <li>Use of a system check command to verify the manipulator's functionality
 *   <li>Use of a fault reporter to report issues with the manipulator's motor
 * </ul>
 */
public class Manipulator extends SubsystemBase {

  // all subsystems receive a reference to their IO implementation when constructed
  private ManipulatorIO io;

  // all subsystems create the AutoLogged version of their IO inputs class
  private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

  // When initially testing a mechanism, it is best to manually provide a voltage or current to
  // verify the mechanical functionality. At times, this can be done via Phoenix Tuner. However,
  // when multiple motors are involved, that is not possible. Using a tunables to enable testing
  // mode and, for the manipulator, specifying voltage is convenient. This feature is also an
  // efficient approach when, for example, empirically tuning the voltage to optimize performance
  // when collecting a game piece.
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

  // Initialize the last state to the uninitialized state and the current state to the desired
  // initial state to ensure that the onEnter method is invoked when the subsystem in constructed.
  private State state = State.WAITING_FOR_GAME_PIECE;
  private State lastState = State.UNINITIALIZED;

  // Some state transitions are triggered by a timeout. Use Timer objects for that purpose.
  Timer inIndexingState = new Timer();
  Timer ejectingTimer = new Timer();

  private CurrentSpikeDetector currentSpikeDetector =
      new CurrentSpikeDetector(
          COLLECTION_CURRENT_SPIKE_THRESHOLD_AMPS, COLLECTION_CURRENT_TIME_THRESHOLD_SECONDS);

  // Some state transitions are triggered by the driver or operator via a button press. We don't
  // want those commands to directly change the state as that can result in a missed state
  // transition. Instead, those commands will change a variable which is monitored within the state
  // machine.
  private boolean releaseButtonPressed = false;

  public Manipulator(ManipulatorIO io) {

    this.io = io;

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
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
   * <p>This example state machine models a manipulator that collects a game piece. The game piece
   * is first detected by a sensor. However, it is not considered indexed (i.e., fully collected)
   * until the game piece stalls against the hard stop. This example also models detecting if the
   * game piece becomes jammed while collecting and attempts to unjam the game piece or eject it.
   * The game piece is released in response to a button press.
   *
   * <p>This approach is modeled after this ChiefDelphi post:
   * https://www.chiefdelphi.com/t/enums-and-subsytem-states/463974/6
   */
  private enum State {
    WAITING_FOR_GAME_PIECE {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(
            Volts.of(subsystem.manipulatorCollectionVoltage.get()));
      }

      @Override
      void execute(Manipulator subsystem) {

        LEDs.getInstance().requestState(States.WAITING_FOR_GAME_PIECE);

        // Often preloading a game piece requires a special case state transition.
        if (DriverStation.isDisabled() && subsystem.isManipulatorIRBlocked()) {
          subsystem.setState(State.GAME_PIECE_IN_MANIPULATOR);
        }
        // check if the game piece is detected by the manipulator
        else if (subsystem.isManipulatorIRBlocked()) {
          subsystem.setState(State.INDEXING_GAME_PIECE_IN_MANIPULATOR);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    INDEXING_GAME_PIECE_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(
            Volts.of(subsystem.manipulatorCollectionVoltage.get()));

        // If a state has a timeout, the timer must be restarted in the onEnter method.
        subsystem.inIndexingState.restart();
      }

      @Override
      void execute(Manipulator subsystem) {

        LEDs.getInstance().requestState(States.INDEXING_GAME_PIECE);

        // check if the game piece has stalled against the hard stop
        if (subsystem.isManipulatorIRBlocked() && subsystem.currentSpikeDetector.getAsBoolean()) {
          subsystem.setState(State.GAME_PIECE_IN_MANIPULATOR);
        }
        // check if the timeout has elapsed which indicates that the game piece may be stuck
        else if (subsystem.inIndexingState.hasElapsed(COLLECTION_TIME_OUT)) {
          subsystem.setState(GAME_PIECE_STUCK);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    GAME_PIECE_STUCK {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(Volts.of(subsystem.manipulatorEjectVoltage.get()));

        // If a state has a timeout, the timer must be restarted in the onEnter method.
        subsystem.ejectingTimer.restart();
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.EJECTING_GAME_PIECE);

        // wait for the specified duration before transitioning back to the waiting for game piece
        // state to ensure that the game piece has been ejected
        if (subsystem.ejectingTimer.hasElapsed(EJECT_DURATION_SECONDS)) {
          subsystem.setState(State.WAITING_FOR_GAME_PIECE);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    GAME_PIECE_IN_MANIPULATOR {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(Volts.of(0.0));
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.HAS_GAME_PIECE);

        // check if the release button has been pressed
        if (subsystem.releaseButtonPressed) {
          subsystem.setState(State.RELEASE_GAME_PIECE);
          subsystem.releaseButtonPressed = false;
        }
        // check if the game piece is no longer detected by the manipulator; this could occur if
        // it has dropped or knocked out; we don't want to be stuck in this state
        else if (!subsystem.isManipulatorIRBlocked()) {
          subsystem.setState(State.WAITING_FOR_GAME_PIECE);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    RELEASE_GAME_PIECE {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(Volts.of(subsystem.manipulatorReleaseVoltage.get()));
      }

      @Override
      void execute(Manipulator subsystem) {
        LEDs.getInstance().requestState(States.RELEASING_GAME_PIECE);

        // wait until the game piece is no longer detected by the manipulator before transitioning
        // back to the waiting for game piece state
        if (!subsystem.isManipulatorIRBlocked()) {
          subsystem.setState(State.WAITING_FOR_GAME_PIECE);
        }
      }

      @Override
      void onExit(Manipulator subsystem) {}
    },

    UNINITIALIZED {
      @Override
      void onEnter(Manipulator subsystem) {
        subsystem.setManipulatorMotorVoltage(Volts.of(0.0));
      }

      @Override
      void execute(Manipulator subsystem) {
        subsystem.setState(
            State.WAITING_FOR_GAME_PIECE); // default state to WAITING_FOR_CORAL_IN_FUNNEL state
      }

      @Override
      void onExit(Manipulator subsystem) {}
    };

    abstract void execute(Manipulator subsystem);

    abstract void onEnter(Manipulator subsystem);

    abstract void onExit(Manipulator subsystem);
  }

  @Override
  public void periodic() {
    // the first step in periodic is to update the inputs from the IO implementation.
    io.updateInputs(inputs);

    // the next step is to log the inputs to the AdvantageKit logger.
    Logger.processInputs("Manipulator", inputs);

    // Subsystems may need to log additional information that is not part of the inputs. This is
    // done for convenience as additional values can always be logged when replaying a log file.
    // Logging the state is very useful.
    Logger.recordOutput(SUBSYSTEM_NAME + "/State", this.state);

    // If a filter is used, it must be updated every periodic call.
    currentSpikeDetector.update(inputs.manipulatorStatorCurrent.in(Amps));

    // If the testing mode is enabled, apply the specified voltage (if not zero). Only run the state
    // machine if testing mode is not enabled. Otherwise, the state machine will "fight" the
    // specified testing value. Similarly, if testing the mechanism using Phoenix Tuner, enable
    // testing mode to ensure that the state machine won't "fight" Phoenix Tuner.
    if (testingMode.get() == 1) {
      if (manipulatorMotorVoltage.get() != 0) {
        setManipulatorMotorVoltage(Volts.of(manipulatorMotorVoltage.get()));
      }
    } else {
      runStateMachine();
    }

    // Log how long this subsystem takes to execute its periodic method.
    // This is useful for debugging performance issues.
    LoggedTracer.record("Manipulator");
  }

  public void resetStateMachine() {
    this.state = State.WAITING_FOR_GAME_PIECE;
  }

  public void releaseGamePiece() {
    releaseButtonPressed = true;
  }

  public boolean isIndexingGamePiece() {
    return state == State.INDEXING_GAME_PIECE_IN_MANIPULATOR;
  }

  public boolean hasIndexedGamePiece() {
    return state == State.GAME_PIECE_IN_MANIPULATOR;
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

  private void setManipulatorMotorVoltage(Voltage volts) {
    io.setManipulatorVoltage(volts);
  }

  // The inputs class contains the state of the primary and secondary IR sensors. It is useful to
  // have both logged when checking for sensor reliability across matches. Which sensors are used
  // are determined based on the dashboard button.
  private boolean isManipulatorIRBlocked() {
    if (OISelector.getOperatorInterface().getEnablePrimaryIRSensorsTrigger().getAsBoolean()) {
      return inputs.isManipulatorPrimaryIRBlocked;
    } else {
      return inputs.isManipulatorSecondaryIRBlocked;
    }
  }

  // A subsystem's system check command is used to verify the functionality of the subsystem. It
  // should perform a sequence of commands (usually encapsulated in another method). The command
  // should always be decorated with an `until` condition that checks for faults in the subsystem
  // and an `andThen` condition that sets the subsystem to a safe state. This ensures that if any
  // faults are detected, the test will stop and the subsystem is always left in a safe state.
  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> io.setManipulatorVoltage(Volts.of(3.6))),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.manipulatorVelocity.lt(RotationsPerSecond.of(2.0))) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Manipulator motor not moving as fast as expected",
                            false);
                  }
                }),
            Commands.runOnce(() -> io.setManipulatorVoltage(Volts.of(-2.4))),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.manipulatorVelocity.gt(RotationsPerSecond.of(-2.0))) {
                    FaultReporter.getInstance()
                        .addFault(
                            SUBSYSTEM_NAME,
                            "[System Check] Manipulator motor moving too slow or in the wrong direction",
                            false);
                  }
                }))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> io.setManipulatorVoltage(Volts.of(0.0))));
  }
}
