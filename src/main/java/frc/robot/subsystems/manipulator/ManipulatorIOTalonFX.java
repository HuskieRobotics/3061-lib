package frc.robot.subsystems.manipulator;

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

public class ManipulatorIOTalonFX implements ManipulatorIO {
  // This mechanism has no close loop control; we just set the voltage directly.
  private VoltageOut indexerVoltageRequest;

  private TalonFX manipulatorMotor;
  private DigitalInput manipulatorIRSensor;
  private DigitalInput backupManipulatorIRSensor;

  private Alert manipulatorConfigAlert =
      new Alert("Failed to apply configuration for manipulator.", AlertType.kError);

  private StatusSignal<Current> manipulatorMotorStatorCurrent;
  private StatusSignal<Current> manipulatorMotorSupplyCurrent;
  private StatusSignal<Temperature> manipulatorMotorTemp;
  private StatusSignal<Voltage> manipulatorMotorVoltage;
  private StatusSignal<AngularVelocity> manipulatorMotorVelocity;

  private final Debouncer manipulatorConnectedDebouncer = new Debouncer(0.5);

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

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
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
    // Determine if the motor for the manipulator is still connected (i.e., reachable on the CAN
    // bus). We do this by verifying that none of the status signals for the device report an error.
    inputs.manipulatorConnected =
        manipulatorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                manipulatorMotorSupplyCurrent,
                manipulatorMotorStatorCurrent,
                manipulatorMotorTemp,
                manipulatorMotorVoltage,
                manipulatorMotorVelocity));

    inputs.manipulatorStatorCurrent = manipulatorMotorStatorCurrent.getValue();
    inputs.manipulatorSupplyCurrent = manipulatorMotorSupplyCurrent.getValue();
    inputs.manipulatorTemp = manipulatorMotorTemp.getValue();
    inputs.manipulatorVelocity = manipulatorMotorVelocity.getValue();
    inputs.manipulatorMotorVoltage = manipulatorMotorVoltage.getValue();
    inputs.isManipulatorPrimaryIRBlocked = !manipulatorIRSensor.get();
    inputs.isManipulatorSecondaryIRBlocked = !backupManipulatorIRSensor.get();
  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., Voltage) throughout the rest of the code to mitigate bugs due to unit
  // mismatches.
  @Override
  public void setManipulatorVoltage(Voltage volts) {
    this.manipulatorMotor.setControl(indexerVoltageRequest.withOutput(volts));
  }

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

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(motor, config, manipulatorConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "manipulator motor", motor);
  }
}
