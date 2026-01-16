package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Example subsystem for controlling the rotation of an arm mechanism.
 *
 * <p>WARNING: This code is for example purposes only. It will not work with a physical arm
 * mechanism without changes. While it is derived from Huskie Robotics 2024 shooter pivot, it has
 * been simplified to highlight select best practices.
 *
 * <p>This example illustrates the following features:
 *
 * <ul>
 *   <li>Use of a remote sensor (CANCoder) to measure the angle of the arm
 *   <li>AdvantageKit support for logging and replay
 *   <li>Use of a debouncer to determine if the arm is at a setpoint
 *   <li>Use of logged tunable numbers for manual control and testing
 *   <li>Use of a simulation class to model the arm's behavior in simulation and provide
 *       visualization
 *   <li>Use of a SysIdRoutine to perform system identification
 *   <li>Use of a system check command to verify the arm's functionality
 *   <li>Use of a fault reporter to report issues with the arm's angle
 * </ul>
 */
public class Arm extends SubsystemBase {
  // all subsystems receive a reference to their IO implementation when constructed
  private ArmIO io;

  // all subsystems create the AutoLogged version of their IO inputs class
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  // When initially testing a mechanism, it is best to manually provide a voltage or current to
  // verify the mechanical functionality. At times, this can be done via Phoenix Tuner. However,
  // when multiple motors are involved, that is not possible. Using a tunables to enable testing
  // mode and, for the arm, specifying voltage or positions is convenient. This feature is also an
  // efficient approach when, for example, empirically tuning the angle for different distances when
  // shooting a game piece.
  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Arm/TestingMode", 0);
  private final LoggedTunableNumber angleManualControlVoltage =
      new LoggedTunableNumber("Arm/ManualControlVoltage", ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE);
  private final LoggedTunableNumber pivotAngleDegrees =
      new LoggedTunableNumber("Arm/AngleDegrees", LOWER_ANGLE_LIMIT.in(Degrees));

  private final Debouncer atSetpointDebouncer = new Debouncer(0.1);

  // The SysId routine is used to characterize the mechanism. The ramp rate and step voltage
  // specified in the configuration often needs to be adjusted based on the physical mechanism
  // (e.g., an arm can only pivot so far) to ensure the mechanism can be characterized within
  // its range of motion.
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(0.5).per(Second), // override default ramp rate (1 V/s)
              Volts.of(2.0), // override default step voltage (7 V)
              null, // use default timeout (10 s)
              state -> SignalLogger.writeString("SysId_State", state.toString())),
          new SysIdRoutine.Mechanism(output -> io.setAngleMotorVoltage(output), null, this));

  public Arm(ArmIO io) {
    this.io = io;

    // Register this subsystem's SysId routine with the SysIdRoutineChooser. This allows
    // the routine to be selected and executed from the dashboard.
    SysIdRoutineChooser.getInstance().addOption("Arm Voltage", sysIdRoutine);

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  @Override
  public void periodic() {
    // the first step in periodic is to update the inputs from the IO implementation.
    io.updateInputs(inputs);

    // the next step is to log the inputs to the AdvantageKit logger.
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    // If the testing mode is enabled, set either the pivot angle (if not zero) or apply the
    // specified voltage (if not zero).
    if (testingMode.get() == 1) {
      if (pivotAngleDegrees.get() != 0) {
        io.setAngle(Degrees.of(pivotAngleDegrees.get()));
      } else if (angleManualControlVoltage.get() != 0) {
        io.setAngleMotorVoltage(Volts.of(angleManualControlVoltage.get()));
      }
    }

    // Log how long this subsystem takes to execute its periodic method.
    // This is useful for debugging performance issues.
    LoggedTracer.record("Arm");
  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., Angle) throughout the rest of the code to mitigate bugs due to unit
  // mismatches.
  public void setAngle(Angle angle) {
    io.setAngle(angle);
  }

  public boolean isAngleAtSetpoint() {
    // This method uses a debouncer to determine if the arm is at the setpoint angle.
    // The angle is considered at the setpoint if the angle is within tolerance for the period
    // specified when constructing the debouncer (e.g., 0.1 seconds or 5 loop iterations).
    return atSetpointDebouncer.calculate(
        inputs.angleMotorReferenceAngle.isNear(inputs.position, ANGLE_TOLERANCE));
  }

  private Command getSystemCheckCommand() {
    // A subsystem's system check command is used to verify the functionality of the subsystem. It
    // should perform a sequence of commands (usually encapsulated in another method). The command
    // should always be decorated with an `until` condition that checks for faults in the subsystem
    // and an `andThen` condition that sets the subsystem to a safe state. This ensures that if any
    // faults are detected, the test will stop and the subsystem is always left in a safe state.
    return Commands.sequence(
            getPresetCheckCommand(Degrees.of(20.0)),
            getPresetCheckCommand(Degrees.of(40.0)),
            getPresetCheckCommand(Degrees.of(50.0)),
            getPresetCheckCommand(Degrees.of(70.0)))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> io.setAngle(LOWER_ANGLE_LIMIT)));
  }

  private Command getPresetCheckCommand(Angle targetPosition) {
    return Commands.sequence(
        Commands.runOnce(() -> this.setAngle(targetPosition)),
        Commands.waitSeconds(2.0),
        Commands.runOnce(() -> this.checkAngle(targetPosition)));
  }

  private void checkAngle(Angle targetPosition) {
    if (this.inputs.position.isNear(targetPosition, ANGLE_TOLERANCE)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "Shooter angle is out of tolerance, should be "
                  + targetPosition
                  + " but is "
                  + this.inputs.position);
    }
  }
}
