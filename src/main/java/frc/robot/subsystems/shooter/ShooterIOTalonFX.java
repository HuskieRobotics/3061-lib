package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public class ShooterIOTalonFX implements ShooterIO {
  private TalonFX shootMotorTop;
  private TalonFX shootMotorBottom;
  private TalonFX kickerMotor;

  private VelocityTorqueCurrentFOC velocityRequest;
  private VelocityTorqueCurrentFOC velocityRequestKicker;
  private PositionVoltage positionRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ShooterIOTalonFX() {
    configMotor(ShooterConstants.RIGHT_MOTOR_CAN_ID, ShooterConstants.LEFT_MOTOR_CAN_ID, ShooterConstants.KICKER_MOTOR_CAN_ID);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.appliedVelocity = shootMotorTop.getMotorVoltage().getValueAsDouble();
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

  public void setVelocity(double velocity) {
    shootMotorTop.setControl(velocityRequest.withVelocity(velocity));
  }

  public void setKickerMotor(double velocity) {
    kickerMotor.setControl(velocityRequestKicker.withVelocity(velocity));
  }

  public void setTopMotor(double velocity) {
    shootMotorTop.setControl(velocityRequest.withVelocity(velocity));
  }

  public void setBottomMotor(double velocity) {
    shootMotorBottom.setControl(velocityRequest.withVelocity(velocity));
  }

  private void configMotor(int rightMotorID, int leftMotorID, int kickerMotorID) {

    this.shootMotorTop = new TalonFX(rightMotorID, RobotConfig.getInstance().getCANBusName());
    this.shootMotorBottom = new TalonFX(leftMotorID, RobotConfig.getInstance().getCANBusName());
    this.kickerMotor =
        new TalonFX(
            ShooterConstants.KICKER_MOTOR_CAN_ID, RobotConfig.getInstance().getCANBusName());

    this.shootMotorBottom.setInverted(true);
    this.shootMotorTop.setInverted(true);
    // this.leftMotor.setControl(new StrictFollower(ShooterConstants.RIGHT_MOTOR_CAN_ID));

    //TalonFXConfiguration config = new TalonFXConfiguration();

    // this.rightMotor.setPosition(0);
    // this.leftMotor.setPosition(0);

    // this.velocityRequest = new VelocityTorqueCurrentFOC(0.0);
    // // this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    // // this.currentRequest = new TorqueCurrentFOC(0.0);
    // this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    // this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
  }
}
