package frc.robot.subsystems.subsystem;

import static frc.robot.subsystems.subsystem.SubsystemConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.lib.team254.drivers.TalonFXFactory;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.Conversions;
import frc.lib.team6328.util.TunableNumber;

/** TalonFX implementation of the generic SubsystemIO */
public class SubsystemIOTalonFX implements SubsystemIO {
  private TalonFX motor;

  private final TunableNumber kP = new TunableNumber("Subsystem/kP", POSITION_PID_P);
  private final TunableNumber kI = new TunableNumber("Subsystem/kI", POSITION_PID_I);
  private final TunableNumber kD = new TunableNumber("Subsystem/kD", POSITION_PID_D);
  private final TunableNumber kF = new TunableNumber("Subsystem/kF", POSITION_PID_F);
  private final TunableNumber kIz = new TunableNumber("Subsystem/kIz", POSITION_PID_I_ZONE);
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
    inputs.positionDeg = Conversions.falconToDegrees(motor.getSelectedSensorPosition(), GEAR_RATIO);
    inputs.velocityRPM = Conversions.falconToRPM(motor.getSelectedSensorVelocity(), GEAR_RATIO);
    inputs.closedLoopError = motor.getClosedLoopError();
    inputs.appliedVoltage = motor.getMotorOutputVoltage();
    inputs.setpoint = motor.getClosedLoopTarget();
    inputs.power = motor.getMotorOutputPercent();
    inputs.controlMode = motor.getControlMode().toString();
    inputs.statorCurrentAmps = new double[] {motor.getStatorCurrent()};
    inputs.tempCelsius = new double[] {motor.getTemperature()};
    inputs.supplyCurrentAmps = new double[] {motor.getSupplyCurrent()};

    // update configuration if tunables have changed
    if (kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || kF.hasChanged()
        || kIz.hasChanged()
        || kPeakOutput.hasChanged()) {
      motor.config_kF(0, kF.get());
      motor.config_kP(0, kP.get());
      motor.config_kI(0, kI.get());
      motor.config_kD(0, kD.get());
      motor.config_IntegralZone(0, kIz.get());
      motor.configClosedLoopPeakOutput(0, kPeakOutput.get());
    }
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  @Override
  public void setMotorPower(double power) {
    motor.set(ControlMode.PercentOutput, power);
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  @Override
  public void setMotorCurrent(double current) {
    motor.set(ControlMode.Current, current);
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setMotorPosition(double position, double arbitraryFeedForward) {
    motor.set(
        TalonFXControlMode.Position,
        Conversions.degreesToFalcon(position, GEAR_RATIO),
        DemandType.ArbitraryFeedForward,
        arbitraryFeedForward);
  }

  private void configMotor(int motorID) {
    TalonFXFactory.Configuration motorConfig = new TalonFXFactory.Configuration();
    motorConfig.INVERTED = MOTOR_INVERTED;
    motorConfig.BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS = 9;
    motorConfig.NEUTRAL_MODE = NeutralMode.Brake;
    motorConfig.STATOR_CURRENT_LIMIT = CURRENT_LIMIT_CONFIG;
    motorConfig.SLOT0_KP = kP.get();
    motorConfig.SLOT0_KI = kI.get();
    motorConfig.SLOT0_KD = kD.get();
    motorConfig.SLOT0_KF = kF.get();
    motorConfig.NEUTRAL_DEADBAND = 0.001;
    motor =
        TalonFXFactory.createTalon(motorID, RobotConfig.getInstance().getCANBusName(), motorConfig);
    motor.setSelectedSensorPosition(0);
    motor.configClosedLoopPeakOutput(0, kPeakOutput.get());
    motor.config_IntegralZone(0, kIz.get());
  }
}
