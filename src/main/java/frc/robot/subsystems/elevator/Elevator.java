package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.subsystems.elevator.ElevatorConstants.Positions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private Positions targetPosition = Positions.BOTTOM;

  private Alert jammedAlert =
      new Alert("Elevator jam detected. Use manual control.", AlertType.kError);

  // the first value is the time constant, the characteristic timescale of the
  // filter's impulse response, and the second value is the time-period, how often
  // the calculate() method will be called
  private LinearFilter current = LinearFilter.singlePoleIIR(0.1, 0.02);

  private LinearFilter jamFilter = LinearFilter.singlePoleIIR(0.4, 0.02);

  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Elevator/TestingMode", 0);
  private final LoggedTunableNumber elevatorVoltage =
      new LoggedTunableNumber("Elevator/Voltage", 0);
  private final LoggedTunableNumber elevatorHeightInches =
      new LoggedTunableNumber("Elevator/Height(Inches)", 0);

  public Elevator(ElevatorIO io) {

    this.elevatorIO = io;

    io.zeroPosition();

    SysIdRoutineChooser.getInstance().addOption("Elevator Voltage", sysIdRoutine);

    FaultReporter.getInstance()
        .registerSystemCheck(SUBSYSTEM_NAME, getElevatorSystemCheckCommand());
  }

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(2.0).per(Second), // Use default ramp rate (1 V/s)
              Volts.of(2.0), // Use default step voltage (7 V)
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> elevatorIO.setMotorVoltage(output.in(Volts)), null, this));

  @Override
  public void periodic() {
    elevatorIO.updateInputs(inputs);
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    Logger.recordOutput(SUBSYSTEM_NAME + "/targetPosition", targetPosition);

    current.calculate(Math.abs(inputs.statorCurrentAmpsLead));

    if (jamFilter.calculate(Math.abs(inputs.statorCurrentAmpsLead)) > JAMMED_CURRENT) {
      CommandScheduler.getInstance()
          .schedule(
              Commands.sequence(
                      Commands.runOnce(() -> elevatorIO.setMotorVoltage(0), this),
                      Commands.run(
                              () -> LEDs.getInstance().requestState(LEDs.States.ELEVATOR_JAMMED))
                          .withTimeout(1.0))
                  .withName("stop elevator jammed"));
      jammedAlert.set(true);
    } else {
      jammedAlert.set(false);
    }

    if (testingMode.get() == 1) {
      if (elevatorVoltage.get() != 0) {
        elevatorIO.setMotorVoltage(elevatorVoltage.get());
      } else if (elevatorHeightInches.get() != 0) {
        elevatorIO.setPosition(Inches.of(elevatorHeightInches.get()));
      }
    }

    // Record cycle time
    LoggedTracer.record("Elevator");
  }

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

    return getPosition().minus(positionToDistance(position)).abs(Inches) < TOLERANCE_INCHES;
  }

  public Command getElevatorSystemCheckCommand() {
    return Commands.sequence(
            getTestPositionCommand(Positions.BOTTOM),
            getTestPositionCommand(Positions.MIDDLE),
            getTestPositionCommand(Positions.TOP))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(getElevatorLowerAndResetCommand())
        .withName(SUBSYSTEM_NAME + "SystemCheck");
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
    return Inches.of(inputs.positionInches);
  }

  public void raiseElevatorSlow() {
    elevatorIO.setMotorVoltage(ELEVATOR_RAISE_SLOW_VOLTAGE);
  }

  public void lowerElevatorSlow() {
    elevatorIO.setMotorVoltage(ELEVATOR_LOWERING_SLOW_VOLTAGE);
  }

  public void stop() {
    elevatorIO.setMotorVoltage(0);
  }

  public void zero() {
    elevatorIO.zeroPosition();
  }

  public Command getElevatorLowerAndResetCommand() {
    return Commands.runOnce(() -> goToPosition(Positions.BOTTOM));
  }
}
