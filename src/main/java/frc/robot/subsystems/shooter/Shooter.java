package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
 * Example subsystem for controlling the velocity of an shooter mechanism with independently
 * controlled top and bottom shooter wheels.
 *
 * <p>WARNING: This code is for example purposes only. It will not work with a physical shooter
 * mechanism without changes. While it is derived from Huskie Robotics 2024 shooter, it has been
 * simplified to highlight select best practices.
 *
 * <p>This example illustrates the following features:
 *
 * <ul>
 *   <li>AdvantageKit support for logging and replay
 *   <li>Use of an InterpolatingDoubleTreeMap to map distances to velocities
 *   <li>Use of a debouncer to determine if the wheels are at a setpoint
 *   <li>Use of logged tunable numbers for manual control and testing
 *   <li>Use of a simulation class to model the wheels' behavior in simulation
 *   <li>Use of a SysIdRoutine to perform system identification
 *   <li>Use of a system check command to verify the shooter's functionality
 *   <li>Use of a fault reporter to report issues with the shooter's devices
 * </ul>
 */
public class Shooter extends SubsystemBase {
  // all subsystems receive a reference to their IO implementation when constructed
  private ShooterIO io;

  // all subsystems create the AutoLogged version of their IO inputs class
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  // When initially testing a mechanism, it is best to manually provide a voltage or current to
  // verify the mechanical functionality. At times, this can be done via Phoenix Tuner. However,
  // when multiple motors are involved, that is not possible. Using a tunables to enable testing
  // mode and, for the shooter, specifying current or velocity is convenient. This feature is also
  // an efficient approach when, for example, empirically tuning the velocity for different
  // distances when shooting a game piece.
  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Shooter/TestingMode", 0);
  private final LoggedTunableNumber topWheelVelocityRPS =
      new LoggedTunableNumber("Shooter/Top Wheel Velocity (RPS)", 0);
  private final LoggedTunableNumber bottomWheelVelocityRPS =
      new LoggedTunableNumber("Shooter/Bottom Wheel Velocity (RPS)", 0);
  private final LoggedTunableNumber topWheelCurrent =
      new LoggedTunableNumber("Shooter/Top Wheel Current", 0);
  private final LoggedTunableNumber bottomWheelCurrent =
      new LoggedTunableNumber("Shooter/Bottom Wheel Current", 0);

  // As an alternative to determining a mathematical function to map distances to velocities,
  // we can use an InterpolatingDoubleTreeMap to store the distances and their corresponding
  // velocities. The InterpolatingDoubleTreeMap will linearly interpolate the velocity
  // for any distance that is not explicitly defined in the map.
  private InterpolatingDoubleTreeMap shootingMap = new InterpolatingDoubleTreeMap();
  private static final double FIELD_MEASUREMENT_OFFSET = 0.0;
  private final double[] shootingPopulationDistances = {7.329, 9.649, 11.336};
  private final double[] shootingPopulationRealVelocities = {40.0, 48.0, 55.0};

  private final Debouncer topAtSetpointDebouncer = new Debouncer(0.1);
  private final Debouncer bottomAtSetpointDebouncer = new Debouncer(0.1);

  // The SysId routine is used to characterize the mechanism. While the SysId routine is intended to
  // be used for voltage control, we can apply a current instead and reinterpret the units when
  // performing the analysis in SysId.
  private final SysIdRoutine shooterWheelTopSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
              Volts.of(10), // will actually be a step to 10 A
              Seconds.of(5), // override default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> io.setShooterWheelTopCurrent(Amps.of(output.in(Volts))),
              null,
              this)); // treat volts as amps
  private final SysIdRoutine shooterWheelBottomSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(5).per(Second), // will actually be a ramp rate of 5 A/s
              Volts.of(10), // will actually be a step to 10 A
              Seconds.of(5), // override default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> io.setShooterWheelBottomCurrent(Amps.of(output.in(Volts))),
              null,
              this)); // treat volts as amps

  public Shooter(ShooterIO io) {
    this.io = io;

    populateShootingMap();

    // Register this subsystem's SysId routine with the SysIdRoutineChooser. This allows
    // the routine to be selected and executed from the dashboard.
    SysIdRoutineChooser.getInstance()
        .addOption("Shooter Wheel Top Current", shooterWheelTopSysIdRoutine);
    SysIdRoutineChooser.getInstance()
        .addOption("Shooter Wheel Bottom Current", shooterWheelBottomSysIdRoutine);

    // Register this subsystem's system check command with the fault reporter. The system check
    // command can be added to the Elastic Dashboard to execute the system test.
    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  @Override
  public void periodic() {
    // the first step in periodic is to update the inputs from the IO implementation.
    io.updateInputs(shooterInputs);

    // the next step is to log the inputs to the AdvantageKit logger.
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);

    // If the testing mode is enabled, set either the velocity (if not zero) or apply the
    // specified current (if not zero).
    if (testingMode.get() == 1) {
      if (topWheelVelocityRPS.get() != 0) {
        io.setShooterWheelTopVelocity(RotationsPerSecond.of(topWheelVelocityRPS.get()));
      } else if (topWheelCurrent.get() != 0) {
        io.setShooterWheelTopCurrent(Amps.of(topWheelCurrent.get()));
      }
      if (bottomWheelVelocityRPS.get() != 0) {
        io.setShooterWheelBottomVelocity(RotationsPerSecond.of(bottomWheelVelocityRPS.get()));
      } else if (bottomWheelCurrent.get() != 0) {
        io.setShooterWheelBottomCurrent(Amps.of(bottomWheelCurrent.get()));
      }
    }

    // Log how long this subsystem takes to execute its periodic method.
    // This is useful for debugging performance issues.
    LoggedTracer.record("Shooter");
  }

  public void setIdleVelocity() {
    io.setShooterWheelBottomVelocity(SHOOTER_IDLE_VELOCITY);
    io.setShooterWheelTopVelocity(SHOOTER_IDLE_VELOCITY);
  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., Distance) throughout the rest of the code to mitigate bugs due to
  // unit mismatches.
  public void setVelocity(Distance distance) {
    // Subsystems may need to log additional information that is not part of the inputs. This is
    // done for convenience as additional values can always be logged when replaying a log file.
    // While this is often done in the periodic method, at times values are only logged when they
    // are changed.
    Logger.recordOutput("Shooter/distance", distance);

    io.setShooterWheelTopVelocity(RotationsPerSecond.of(shootingMap.get(distance.in(Meters))));
    io.setShooterWheelBottomVelocity(RotationsPerSecond.of(shootingMap.get(distance.in(Meters))));
  }

  public boolean isTopShooterAtSetpoint() {
    // This method uses a debouncer to determine if the shooter wheels are at the setpoint velocity.
    // The velocity is considered at the setpoint if the velocity is within tolerance for the period
    // specified when constructing the debouncer (e.g., 0.1 seconds or 5 loop iterations).
    return topAtSetpointDebouncer.calculate(
        shooterInputs.shootMotorTopVelocity.isNear(
            shooterInputs.shootMotorTopReferenceVelocity, VELOCITY_TOLERANCE));
  }

  public boolean isBottomShooterAtSetpoint() {
    return bottomAtSetpointDebouncer.calculate(
        shooterInputs.shootMotorBottomVelocity.isNear(
            shooterInputs.shootMotorBottomReferenceVelocity, VELOCITY_TOLERANCE));
  }

  private void populateShootingMap() {
    for (int i = 0; i < shootingPopulationDistances.length; i++) {
      shootingMap.put(
          shootingPopulationDistances[i] + FIELD_MEASUREMENT_OFFSET,
          shootingPopulationRealVelocities[i]);
    }
  }

  private Command getSystemCheckCommand() {
    // A subsystem's system check command is used to verify the functionality of the subsystem. It
    // should perform a sequence of commands (usually encapsulated in another method). The command
    // should always be decorated with an `until` condition that checks for faults in the subsystem
    // and an `andThen` condition that sets the subsystem to a safe state. This ensures that if any
    // faults are detected, the test will stop and the subsystem is always left in a safe state.
    return Commands.sequence(
            getPresetCheckCommand(Meters.of(7.0)),
            getPresetCheckCommand(Meters.of(9.0)),
            getPresetCheckCommand(Meters.of(11.0)),
            getPresetCheckCommand(Meters.of(13.0)))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(
            Commands.runOnce(
                () -> {
                  io.setShooterWheelBottomVelocity(RotationsPerSecond.of(0.0));
                  io.setShooterWheelTopVelocity(RotationsPerSecond.of(0.0));
                }));
  }

  private Command getPresetCheckCommand(Distance distance) {
    return Commands.sequence(
        Commands.runOnce(() -> this.setVelocity(distance)),
        Commands.waitSeconds(2.0),
        Commands.runOnce(
            () ->
                this.checkVelocity(
                    RotationsPerSecond.of(shootingMap.get(distance.in(Meters))),
                    RotationsPerSecond.of(shootingMap.get(distance.in(Meters))))));
  }

  private void checkVelocity(AngularVelocity topVelocity, AngularVelocity bottomVelocity) {
    // check bottom motor
    if (!this.shooterInputs.shootMotorBottomVelocity.isNear(bottomVelocity, VELOCITY_TOLERANCE)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "Bottom shooter wheel velocity out of tolerance, should be "
                  + bottomVelocity
                  + " but is "
                  + this.shooterInputs.shootMotorBottomVelocity);
    }
    // check top motor
    if (!this.shooterInputs.shootMotorTopVelocity.isNear(topVelocity, VELOCITY_TOLERANCE)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "Top shooter wheel velocity out of tolerance, should be "
                  + topVelocity
                  + " but is "
                  + this.shooterInputs.shootMotorTopVelocity);
    }
  }
}
