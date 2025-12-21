package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team254.CurrentSpikeDetector;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.elevator.ElevatorConstants.Positions;
import org.littletonrobotics.junction.Logger;

/**
 * Example subsystem for controlling an elevator.
 *
 * <p>WARNING: This code is for example purposes only. It will not work with a physical elevator
 * mechanism without changes. While it is derived from Huskie Robotics 2025 elevator, it has been
 * simplified to highlight select best practices.
 *
 * <p>This example illustrates the following features:
 *
 * <ul>
 *   <li>Configuring a lead and follower motor
 *   <li>AdvantageKit support for logging and replay
 *   <li>Use of logged tunable numbers for manual control and testing
 *   <li>Use of filtering current values to detect if the elevator is jammed
 *   <li>Use of a simulation class to model the elevator's behavior in simulation and provide
 *       visualization
 *   <li>Use of a SysIdRoutine to perform system identification
 *   <li>Use of a system check command to verify the elevator's functionality
 *   <li>Use of a fault reporter to report issues with the elevator's motors
 * </ul>
 */
public class Elevator extends SubsystemBase {
  // all subsystems receive a reference to their IO implementation when constructed
  private ElevatorIO elevatorIO;

  // all subsystems create the AutoLogged version of their IO inputs class
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private Positions targetPosition = Positions.BOTTOM;

  private Alert jammedAlert =
      new Alert("Elevator jam detected. Use manual control.", AlertType.kError);

  private CurrentSpikeDetector jamDetector =
      new CurrentSpikeDetector(JAMMED_CURRENT_AMPS, JAMMED_TIME_THRESHOLD_SECONDS);

  private final Debouncer atSetpointDebouncer = new Debouncer(0.1);

  // When initially testing a mechanism, it is best to manually provide a voltage or current to
  // verify the mechanical functionality. At times, this can be done via Phoenix Tuner. However,
  // when multiple motors are involved, that is not possible. Using a tunables to enable testing
  // mode and, for the elevator, specifying voltage or position is convenient. This feature is also
  // an efficient approach when, for example, empirically tuning the height for different scenarios
  // when scoring a game piece.
  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Elevator/TestingMode", 0);
  private final LoggedTunableNumber elevatorVoltage =
      new LoggedTunableNumber("Elevator/Voltage", 0);
  private final LoggedTunableNumber elevatorHeightInches =
      new LoggedTunableNumber("Elevator/Height(Inches)", 0);

  // The SysId routine is used to characterize the mechanism. The ramp rate and step voltage
  // specified in the configuration often needs to be adjusted based on the physical mechanism
  // (e.g., an elevator can only go up so far) to ensure the mechanism can be characterized within
  // its range of motion.
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2.0).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // Use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> elevatorIO.setMotorVoltage(output), null, this));

  public Elevator(ElevatorIO io) {

    this.elevatorIO = io;

    io.zeroPosition();

    // Register this subsystem's SysId routine with the SysIdRoutineChooser. This allows
    // the routine to be selected and executed from the dashboard.
    SysIdRoutineChooser.getInstance().addOption("Elevator Voltage", sysIdRoutine);

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
    FaultReporter.getInstance()
        .registerSystemCheck(SUBSYSTEM_NAME, getElevatorSystemCheckCommand());
  }

  @Override
  public void periodic() {
    // The first step in periodic is to update the inputs from the IO implementation.
    elevatorIO.updateInputs(inputs);

    // The next step is to log the inputs to the AdvantageKit logger.
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    // Subsystems may need to log additional information that is not part of the inputs. This is
    // done for convenience as additional values can always be logged when replaying a log file.
    Logger.recordOutput(SUBSYSTEM_NAME + "/targetPosition", targetPosition);

    // The elevator is considered jammed if the current consistently exceeds a threshold. We feed
    // the current through a linear filter to exclude current spikes that would certainly be present
    // when the elevator initiates motion. If a jam is detected, we schedule a command to stop the
    // elevator, request a jammed state on the LEDs, and generate an alert. We don't directly stop
    // the elevator using the io object as that won't interrupt commands that are currently using
    // the elevator.
    if (jamDetector.update(Math.abs(inputs.statorCurrentLead.in(Amps)))) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.sequence(
                      Commands.runOnce(() -> elevatorIO.setMotorVoltage(Volts.of(0.0)), this),
                      Commands.run(
                              () -> LEDs.getInstance().requestState(LEDs.States.ELEVATOR_JAMMED))
                          .withTimeout(1.0))
                  .withName("stop elevator jammed"));
      jammedAlert.set(true);
    } else {
      jammedAlert.set(false);
    }

    // If the testing mode is enabled, set either the position (if not zero) or apply the
    // specified voltage (if not zero).
    if (testingMode.get() == 1) {
      if (elevatorVoltage.get() != 0) {
        elevatorIO.setMotorVoltage(Volts.of(elevatorVoltage.get()));
      } else if (elevatorHeightInches.get() != 0) {
        elevatorIO.setPosition(Inches.of(elevatorHeightInches.get()));
      }
    }

    // Log how long this subsystem takes to execute its periodic method.
    // This is useful for debugging performance issues.
    LoggedTracer.record("Elevator");
  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., Distance) throughout the rest of the code to mitigate bugs due to
  // unit mismatches.
  private Distance positionToDistance(Positions reefBranch) {
    Distance height;

    switch (reefBranch) {
      case BOTTOM:
        height = BOTTOM_HEIGHT;
        break;

      case MIDDLE:
        height = MIDDLE_HEIGHT;
        break;

      case TOP:
        height = TOP_HEIGHT;
        break;

      default:
        height = MIN_HEIGHT;
        break;
    }
    return height;
  }

  public boolean isAtPosition(Positions position) {
    return atSetpointDebouncer.calculate(
        getPosition().isNear(positionToDistance(position), LINEAR_POSITION_TOLERANCE));
  }

  public Command getElevatorSystemCheckCommand() {
    // A subsystem's system check command is used to verify the functionality of the subsystem. It
    // should perform a sequence of commands (usually encapsulated in another method). The command
    // should always be decorated with an `until` condition that checks for faults in the subsystem
    // and an `andThen` condition that sets the subsystem to a safe state. This ensures that if any
    // faults are detected, the test will stop and the subsystem is always left in a safe state.
    return Commands.sequence(
            getTestPositionCommand(Positions.BOTTOM),
            getTestPositionCommand(Positions.MIDDLE),
            getTestPositionCommand(Positions.TOP))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> goToPosition(Positions.BOTTOM)));
  }

  private Command getTestPositionCommand(Positions position) {
    return Commands.sequence(
        Commands.runOnce(() -> goToPosition(position), this),
        Commands.waitUntil(() -> isAtPosition(position)).withTimeout(1.0),
        Commands.runOnce(() -> checkPosition(position), this));
  }

  private void checkPosition(Positions position) {
    if (!isAtPosition(position)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "Elevator position not at "
                  + position
                  + " as expected. Should be: "
                  + positionToDistance(position)
                  + " but is: "
                  + getPosition());
    }
  }

  public void goToPosition(Positions position) {
    targetPosition = position;
    elevatorIO.setPosition(positionToDistance(position));
  }

  public Distance getPosition() {
    return inputs.linearPosition;
  }

  public void raiseElevatorSlow() {
    elevatorIO.setMotorVoltage(ELEVATOR_RAISE_SLOW_VOLTAGE);
  }

  public void lowerElevatorSlow() {
    elevatorIO.setMotorVoltage(ELEVATOR_LOWERING_SLOW_VOLTAGE);
  }

  public void stop() {
    elevatorIO.setMotorVoltage(Volts.of(0.0));
  }

  public void zero() {
    elevatorIO.zeroPosition();
  }
}
