package frc.robot.subsystems.subsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.subsystem.SubsystemConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team6328.util.LoggedTunableNumber;

/** TalonFX implementation of the generic SubsystemIO */
public class SubsystemIOTalonFX implements SubsystemIO {
  private TalonFX motor;

  private VoltageOut voltageRequest;
  private TorqueCurrentFOC currentRequest;
  private PositionVoltage positionRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Subsystem/kP", POSITION_PID_P);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Subsystem/kI", POSITION_PID_I);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Subsystem/kD", POSITION_PID_D);
  private final LoggedTunableNumber kPeakOutput =
      new LoggedTunableNumber("Subsystem/kPeakOutput", POSITION_PID_PEAK_OUTPUT);

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
            BaseStatusSignal.getLatencyCompensatedValue(
                    motor.getRotorPosition(), motor.getRotorVelocity())
                .in(Rotations),
            GEAR_RATIO);
    inputs.velocityRPM =
        Conversions.falconRPSToMechanismRPM(
            motor.getRotorVelocity().getValue().in(RotationsPerSecond), GEAR_RATIO);
    inputs.closedLoopError = motor.getClosedLoopError().getValue();
    inputs.setpoint = motor.getClosedLoopReference().getValue();
    inputs.power = motor.getDutyCycle().getValue();
    inputs.controlMode = motor.getControlMode().toString();
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValue().in(Amps);
    inputs.tempCelsius = motor.getDeviceTemp().getValue().in(Celsius);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValue().in(Amps);

    // update configuration if tunables have changed
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.motor.getConfigurator().refresh(config);
          config.Slot0.kP = pid[0];
          config.Slot0.kI = pid[1];
          config.Slot0.kD = pid[2];
          this.motor.getConfigurator().apply(config);
        },
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        peak -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.motor.getConfigurator().refresh(config);
          config.Voltage.PeakForwardVoltage = peak[0];
          config.Voltage.PeakReverseVoltage = peak[0];
          this.motor.getConfigurator().apply(config);
        },
        kPeakOutput);
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  @Override
  public void setMotorVoltage(double volts) {
    this.motor.setControl(voltageRequest.withOutput(volts));
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
    currentLimits.SupplyCurrentLimit = PEAK_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerLimit = CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerTime = PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = PEAK_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
        MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

    Phoenix6Util.applyAndCheckConfiguration(this.motor, config, configAlert);

    this.motor.setPosition(0);

    this.voltageRequest = new VoltageOut(0.0);
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "subsystem motor", motor);
  }
}
