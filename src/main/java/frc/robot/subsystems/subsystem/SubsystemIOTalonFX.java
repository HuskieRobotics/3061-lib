package frc.robot.subsystems.subsystem;

import static frc.robot.subsystems.subsystem.SubsystemConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;

/** TalonFX implementation of the generic SubsystemIO */
public class SubsystemIOTalonFX implements SubsystemIO {
  private TalonFX motor;

  private VoltageOut voltageRequest;
  private TorqueCurrentFOC currentRequest;
  private PositionVoltage positionRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

  private final TunableNumber kP = new TunableNumber("Subsystem/kP", POSITION_PID_P);
  private final TunableNumber kI = new TunableNumber("Subsystem/kI", POSITION_PID_I);
  private final TunableNumber kD = new TunableNumber("Subsystem/kD", POSITION_PID_D);
  private final TunableNumber kPeakOutput =
      new TunableNumber("Subsystem/kPeakOutput", POSITION_PID_PEAK_OUTPUT);

  /** Create a TalonFX-specific generic SubsystemIO */
  public SubsystemIOTalonFX() {
    configMotor(MOTOR_CAN_ID);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(SubsystemIOInputs inputs) {
    inputs.positionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            motor.getRotorPosition().getValue(), GEAR_RATIO);
    inputs.velocityRPM =
        Conversions.falconRPSToMechanismRPM(motor.getRotorVelocity().getValue(), GEAR_RATIO);
    inputs.closedLoopError = motor.getClosedLoopError().getValue();
    inputs.setpoint = motor.getClosedLoopReference().getValue();
    inputs.power = motor.getDutyCycle().getValue();
    inputs.controlMode = motor.getControlMode().toString();
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValue();
    inputs.tempCelsius = motor.getDeviceTemp().getValue();
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValue();

    // update configuration if tunables have changed
    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kPeakOutput.hasChanged()) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      this.motor.getConfigurator().refresh(config);
      config.Slot0.kP = kP.get();
      config.Slot0.kI = kI.get();
      config.Slot0.kD = kD.get();
      config.Voltage.PeakForwardVoltage = kPeakOutput.get();
      config.Voltage.PeakReverseVoltage = kPeakOutput.get();
      this.motor.getConfigurator().apply(config);
    }
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  @Override
  public void setMotorPower(double power) {
    this.motor.setControl(voltageRequest.withOutput(power * 12.0));
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  @Override
  public void setMotorCurrent(double current) {
    this.motor.setControl(currentRequest.withOutput(current));
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setMotorPosition(double position, double arbitraryFeedForward) {
    this.motor.setControl(
        positionRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
  }

  private void configMotor(int motorID) {

    this.motor = new TalonFX(motorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentThreshold = PEAK_CURRENT_LIMIT;
    currentLimits.SupplyTimeThreshold = PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
        MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = this.motor.getConfigurator().apply(config);
      if (status.isOK()) {
        configAlert.set(false);
        break;
      }
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    this.motor.setPosition(0);

    this.voltageRequest = new VoltageOut(0.0);
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "subsystem motor", motor);
  }
}
