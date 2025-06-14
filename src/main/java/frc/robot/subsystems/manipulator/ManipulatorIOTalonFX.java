package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput; // imported this class for the sensors
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OISelector;

/** TalonFX implementation of the generic SubsystemIO */
public class ManipulatorIOTalonFX implements ManipulatorIO {
  private TalonFX manipulatorMotor;

  // ir sensors
  private DigitalInput manipulatorIRSensor;
  private DigitalInput backupManipulatorIRSensor;

  private VoltageOut indexerVoltageRequest;

  private Alert manipulatorConfigAlert =
      new Alert("Failed to apply configuration for manipulator.", AlertType.kError);

  private final LoggedTunableNumber manipulatorKp =
      new LoggedTunableNumber("Manipulator/kP", MANIPULATOR_KP);
  private final LoggedTunableNumber manipulatorKi =
      new LoggedTunableNumber("Manipulator/kI", MANIPULATOR_KI);
  private final LoggedTunableNumber manipulatorKd =
      new LoggedTunableNumber("Manipulator/kD", MANIPULATOR_KD);
  private final LoggedTunableNumber manipulatorKs =
      new LoggedTunableNumber("Manipulator/kS", MANIPULATOR_KS);
  private final LoggedTunableNumber manipulatorKv =
      new LoggedTunableNumber("Manipulator/kV", MANIPULATOR_KV);
  private final LoggedTunableNumber manipulatorKa =
      new LoggedTunableNumber("Manipulator/kA", MANIPULATOR_KA);

  // Create StatusSignal objects for each loggable input from the ManipulatorIO class in the
  // updateInputs method

  private StatusSignal<Current> manipulatorMotorStatorCurrent;
  private StatusSignal<Current> manipulatorMotorSupplyCurrent;
  private StatusSignal<Temperature> manipulatorMotorTemp;
  private StatusSignal<Voltage> manipulatorMotorVoltage;
  private StatusSignal<AngularVelocity> manipulatorMotorVelocity;

  private final Debouncer manipulatorConnectedDebouncer = new Debouncer(0.5);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ManipulatorIOTalonFX() {

    manipulatorMotor = new TalonFX(MANIPULATOR_MOTOR_ID);
    manipulatorIRSensor = new DigitalInput(MANIPULATOR_IR_SENSOR_ID);
    backupManipulatorIRSensor = new DigitalInput(MANIPULATOR_IR_BACKUP_SENSOR_ID);

    indexerVoltageRequest = new VoltageOut(0.0);

    manipulatorMotorStatorCurrent = manipulatorMotor.getStatorCurrent();
    manipulatorMotorSupplyCurrent = manipulatorMotor.getSupplyCurrent();
    manipulatorMotorTemp = manipulatorMotor.getDeviceTemp();
    manipulatorMotorVoltage = manipulatorMotor.getMotorVoltage();
    manipulatorMotorVelocity = manipulatorMotor.getVelocity();

    Phoenix6Util.registerSignals(
        false,
        manipulatorMotorStatorCurrent,
        manipulatorMotorSupplyCurrent,
        manipulatorMotorTemp,
        manipulatorMotorVoltage,
        manipulatorMotorVelocity);

    configManipulatorMotor(manipulatorMotor);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {

    inputs.manipulatorConnected =
        manipulatorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                manipulatorMotorSupplyCurrent,
                manipulatorMotorStatorCurrent,
                manipulatorMotorTemp,
                manipulatorMotorVoltage,
                manipulatorMotorVelocity));

    inputs.manipulatorStatorCurrentAmps = manipulatorMotorStatorCurrent.getValueAsDouble();
    inputs.manipulatorSupplyCurrentAmps = manipulatorMotorSupplyCurrent.getValueAsDouble();
    inputs.manipulatorTempCelsius = manipulatorMotorTemp.getValueAsDouble();
    inputs.manipulatorVelocityRPS = manipulatorMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.manipulatorMotorVoltage = manipulatorMotorVoltage.getValueAsDouble();

    if (OISelector.getOperatorInterface().getEnablePrimaryIRSensorsTrigger().getAsBoolean()) {
      inputs.isManipulatorIRBlocked = !manipulatorIRSensor.get();
    } else {
      inputs.isManipulatorIRBlocked = !backupManipulatorIRSensor.get();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.manipulatorMotor.getConfigurator().refresh(config);
          config.Slot0.kP = pid[0];
          config.Slot0.kI = pid[1];
          config.Slot0.kD = pid[2];
          config.Slot0.kS = pid[3];
          config.Slot0.kV = pid[4];
          config.Slot0.kA = pid[5];
          this.manipulatorMotor.getConfigurator().apply(config);
        },
        manipulatorKp,
        manipulatorKi,
        manipulatorKd,
        manipulatorKs,
        manipulatorKv,
        manipulatorKa);
  }

  @Override
  public void setManipulatorVoltage(double volts) {
    this.manipulatorMotor.setControl(
        indexerVoltageRequest.withLimitForwardMotion(false).withOutput(volts));
  }

  /*
   * This method configures the Indexer motor.
   */
  private void configManipulatorMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = MANIPULATOR_GEAR_RATIO;

    config.MotorOutput.Inverted =
        MANIPULATOR_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = manipulatorKp.get();
    config.Slot0.kI = manipulatorKi.get();
    config.Slot0.kD = manipulatorKd.get();
    config.Slot0.kS = manipulatorKs.get();
    config.Slot0.kV = manipulatorKv.get();
    config.Slot0.kA = manipulatorKa.get();

    Phoenix6Util.applyAndCheckConfiguration(motor, config, manipulatorConfigAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "manipulator motor", motor);
  }
}
