package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private ArmIO io;
  private InterpolatingDoubleTreeMap angleMap;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final LoggedTunableNumber testingMode = new LoggedTunableNumber("Arm/TestingMode", 0);

  private final LoggedTunableNumber angleManualControlVoltage =
      new LoggedTunableNumber("Arm/ManualControlVoltage", ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE);
  private final LoggedTunableNumber pivotAngle = new LoggedTunableNumber("Arm/Angle", 10.0);

  // FIXME: tune on the competition field
  private static final double FIELD_MEASUREMENT_OFFSET = 0.0;
  private final double[] populationRealAngles = {
    66, 59, 54.5, 46.5, 43.5, 42, 38, 36, 35, 33, 32, 31, 29.5, 29.5, 29, 28, 27.5
  };
  private final double[] populationDistances = {
    1.33, 1.63, 1.947, 2.196, 2.47, 2.77, 3.02, 3.32, 3.6, 3.936, 4.206, 4.495, 4.785, 5.083, 5.39,
    5.72, 6.04
  };

  private final Debouncer atSetpointDebouncer = new Debouncer(0.1);

  private static final String BUT_IS = " but is ";

  public Arm(ArmIO io) {
    this.io = io;
    this.angleMap = new InterpolatingDoubleTreeMap();
    populateAngleMap();

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  private void populateAngleMap() {
    for (int i = 0; i < populationRealAngles.length; i++) {
      angleMap.put(populationDistances[i] + FIELD_MEASUREMENT_OFFSET, populationRealAngles[i]);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    if (testingMode.get() == 1) {
      io.setAngle(pivotAngle.get());
    }
  }

  private double getAngleForDistance(double distance) {
    return angleMap.get(distance);
  }

  private void adjustAngle(double distance) {
    io.setAngle(getAngleForDistance(distance));
  }

  public boolean isAngleAtSetpoint() {
    return atSetpointDebouncer.calculate(
        Math.abs(inputs.angleMotorReferenceAngleDegrees - inputs.angleEncoderAngleDegrees)
            < ANGLE_TOLERANCE_DEGREES);
  }

  public void setAngleMotorVoltage(double voltage) {
    io.setAngleMotorVoltage(voltage * angleManualControlVoltage.get());
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
                      io.setAngle(LOWER_ANGLE_LIMIT);
                    }))
            .withName(SUBSYSTEM_NAME + "SystemCheck"));
  }

  private Command getPresetCheckCommand(double distance) {
    return Commands.sequence(
        Commands.runOnce(() -> this.adjustAngle(distance)),
        Commands.waitSeconds(2.0),
        Commands.runOnce(() -> this.checkAngle(getAngleForDistance(distance))));
  }

  private void checkAngle(double degrees) {
    if (Math.abs(this.inputs.angleEncoderAngleDegrees - degrees) > ANGLE_TOLERANCE_DEGREES) {
      if (Math.abs(degrees) - Math.abs(this.inputs.angleEncoderAngleDegrees) > 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Shooter angle is too low, should be "
                    + degrees
                    + BUT_IS
                    + this.inputs.angleEncoderAngleDegrees);
      } else {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Shooter angle is too high, should be "
                    + degrees
                    + BUT_IS
                    + this.inputs.angleEncoderAngleDegrees);
      }
    }
  }
}
