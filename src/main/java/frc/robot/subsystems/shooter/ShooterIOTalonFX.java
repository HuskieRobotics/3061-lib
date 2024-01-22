package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOTalonFX implements ShooterIO {
  private TalonFX rightMotor;
  private TalonFX leftMotor;

  private VoltageOut voltageRequest;
  private TorqueCurrentFOC currentRequest;
  private PositionVoltage positionRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ShooterIOTalonFX() {
    configMotor(ShooterConstants.RIGHT_MOTOR_CAN_ID, ShooterConstants.LEFT_MOTOR_CAN_ID);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.appliedVoltage = rightMotor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public void setMotorPosition(TalonFX motor, double position, double arbitraryFeedForward) {
    motor.setControl(
        positionRequest
            .withPosition(
                Conversions.degreesToFalconRotations(position, ShooterConstants.GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
  }

  public void setAppliedVoltage(double volts) {
    rightMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setRightMotor(double volts) {
    rightMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setLeftMotor(double volts) {
    leftMotor.setControl(voltageRequest.withOutput(volts));
  }

  private void configMotor(int rightMotorID, int leftMotorID) {

    this.rightMotor = new TalonFX(rightMotorID, RobotConfig.getInstance().getCANBusName());
    this.leftMotor = new TalonFX(leftMotorID, RobotConfig.getInstance().getCANBusName());

    this.leftMotor.setInverted(true);
    this.rightMotor.setInverted(true);
    // this.leftMotor.setControl(new StrictFollower(ShooterConstants.RIGHT_MOTOR_CAN_ID));

    TalonFXConfiguration config = new TalonFXConfiguration();

    this.rightMotor.setPosition(0);
    this.leftMotor.setPosition(0);

    this.voltageRequest = new VoltageOut(0.0);
    // this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    // this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
  }
}
