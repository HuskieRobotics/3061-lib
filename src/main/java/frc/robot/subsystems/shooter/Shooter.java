package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private InterpolatingDoubleTreeMap shootingMap = new InterpolatingDoubleTreeMap();

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Shooter/TestingMode", 0);

  private final LoggedTunableNumber topWheelVelocity =
      new LoggedTunableNumber("Shooter/Top Wheel Velocity", 0);
  private final LoggedTunableNumber bottomWheelVelocity =
      new LoggedTunableNumber("Shooter/Bottom Wheel Velocity", 0);

  // FIXME: tune on the competition field
  private static final double FIELD_MEASUREMENT_OFFSET = 0.0;
  private final double[] shootingPopulationDistances = {7.329, 9.649, 11.336};
  private final double[] shootingPopulationRealVelocities = {40.0, 48.0, 55.0};

  private final Debouncer topAtSetpointDebouncer = new Debouncer(0.1);
  private final Debouncer bottomAtSetpointDebouncer = new Debouncer(0.1);

  private static final String BUT_IS = " but is ";

  public Shooter(ShooterIO io) {
    this.io = io;

    populateShootingMap();

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);

    if (testingMode.get() == 1) {
      io.setShooterWheelBottomVelocity(bottomWheelVelocity.get());
      io.setShooterWheelTopVelocity(topWheelVelocity.get());
    }
  }

  public void setIdleVelocity() {
    io.setShooterWheelBottomVelocity(SHOOTER_IDLE_VELOCITY);
    io.setShooterWheelTopVelocity(SHOOTER_IDLE_VELOCITY);
  }

  public void setVelocity(double distance) {
    Logger.recordOutput("Shooter/distance", distance);

    io.setShooterWheelTopVelocity(shootingMap.get(distance));
    io.setShooterWheelBottomVelocity(shootingMap.get(distance));
  }

  public boolean isTopShootAtSetpoint() {
    return topAtSetpointDebouncer.calculate(
        Math.abs(
                shooterInputs.shootMotorTopVelocityRPS
                    - shooterInputs.shootMotorTopReferenceVelocityRPS)
            < VELOCITY_TOLERANCE);
  }

  public boolean isBottomShootAtSetpoint() {
    return bottomAtSetpointDebouncer.calculate(
        Math.abs(
                shooterInputs.shootMotorBottomVelocityRPS
                    - shooterInputs.shootMotorBottomReferenceVelocityRPS)
            < VELOCITY_TOLERANCE);
  }

  private void populateShootingMap() {
    for (int i = 0; i < shootingPopulationDistances.length; i++) {
      shootingMap.put(
          shootingPopulationDistances[i] + FIELD_MEASUREMENT_OFFSET,
          shootingPopulationRealVelocities[i]);
    }
  }

  private Command getSystemCheckCommand() {
    return Commands.sequence(
        getPresetCheckCommand(7.0),
        getPresetCheckCommand(9.0),
        getPresetCheckCommand(11.0),
        getPresetCheckCommand(13.0)
            .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
            .andThen(
                Commands.runOnce(
                    () -> {
                      io.setShooterWheelBottomVelocity(0.0);
                      io.setShooterWheelTopVelocity(0.0);
                    }))
            .withName(SUBSYSTEM_NAME + "SystemCheck"));
  }

  private Command getPresetCheckCommand(double distance) {
    return Commands.sequence(
        Commands.runOnce(() -> this.setVelocity(distance)),
        Commands.waitSeconds(2.0),
        Commands.runOnce(
            () -> this.checkVelocity(shootingMap.get(distance), shootingMap.get(distance))));
  }

  private void checkVelocity(double topVelocity, double bottomVelocity) {
    // check bottom motor
    if (Math.abs(this.shooterInputs.shootMotorBottomVelocityRPS - bottomVelocity)
        > VELOCITY_TOLERANCE) {
      if (Math.abs(this.shooterInputs.shootMotorBottomVelocityRPS) - Math.abs(bottomVelocity) < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Bottom shooter wheel velocity is too low, should be "
                    + bottomVelocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorBottomVelocityRPS);
      } else {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Bottom shooter wheel velocity is too high, should be "
                    + bottomVelocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorBottomVelocityRPS);
      }
    }
    // check top motor
    if (Math.abs(this.shooterInputs.shootMotorTopVelocityRPS - topVelocity) > VELOCITY_TOLERANCE) {
      if (Math.abs(this.shooterInputs.shootMotorTopVelocityRPS) - Math.abs(topVelocity) < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Top shooter wheel velocity is too low, should be "
                    + topVelocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorTopVelocityRPS);
      } else {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Top shooter wheel velocity is too high, should be "
                    + topVelocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorTopVelocityRPS);
      }
    }
  }
}
