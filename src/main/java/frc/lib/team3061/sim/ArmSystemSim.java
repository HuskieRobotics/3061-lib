package frc.lib.team3061.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ArmSystemSim {

  private TalonFX motor;
  private CANcoder encoder;
  private TalonFXSimState motorSimState;
  private CANcoderSimState encoderSimState;
  private SingleJointedArmSim systemSim;
  private double sensorToMechanismRatio;
  private double rotorToSensorRatio;
  private String subsystemName;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private LoggedMechanism2d mech2d;
  private LoggedMechanismRoot2d armPivot;
  private LoggedMechanismLigament2d armTower;
  private LoggedMechanismLigament2d arm;

  public ArmSystemSim(
      TalonFX motor,
      CANcoder encoder,
      boolean motorInverted,
      double sensorToMechanismRatio,
      double rotorToSensorRatio,
      double length,
      double mass,
      double minAngle,
      double maxAngle,
      double startingAngle,
      String subsystemName) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.motor = motor;
    this.encoder = encoder;
    this.sensorToMechanismRatio = sensorToMechanismRatio;
    this.rotorToSensorRatio = rotorToSensorRatio;

    this.motorSimState = this.motor.getSimState();
    this.motorSimState.Orientation =
        motorInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    this.encoderSimState = this.encoder.getSimState();
    this.encoderSimState.Orientation =
        motorInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    this.systemSim =
        new SingleJointedArmSim(
            DCMotor.getFalcon500Foc(1),
            sensorToMechanismRatio * rotorToSensorRatio,
            SingleJointedArmSim.estimateMOI(length, mass),
            length,
            minAngle,
            maxAngle,
            true,
            startingAngle);

    this.mech2d = new LoggedMechanism2d(1, 1);
    this.armPivot = mech2d.getRoot("ArmPivot", 0.5, 0.5);
    this.armTower = armPivot.append(new LoggedMechanismLigament2d("ArmTower", .3, -90));
    this.arm =
        armPivot.append(
            new LoggedMechanismLigament2d(
                "Arm", length, startingAngle, 6, new Color8Bit(Color.kYellow)));
    this.armTower.setColor(new Color8Bit(Color.kBlue));

    this.subsystemName = subsystemName;
  }

  public void updateSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    this.motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.encoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // update the input voltages of the models based on the outputs of the simulated TalonFXs
    double motorVoltage = this.motorSimState.getMotorVoltage();
    this.systemSim.setInput(motorVoltage);

    // update the models
    this.systemSim.update(Constants.LOOP_PERIOD_SECS);

    // update the simulated TalonFX based on the model outputs
    double mechanismRadians = this.systemSim.getAngleRads();
    double mechanismRotations = mechanismRadians / (2 * Math.PI);
    double sensorRotations = mechanismRotations * this.sensorToMechanismRatio;
    double motorRotations = sensorRotations * this.rotorToSensorRatio;
    double mechanismRadiansPerSec = this.systemSim.getVelocityRadPerSec();
    double mechanismRPS = mechanismRadiansPerSec / (2 * Math.PI);
    double encoderRPS = mechanismRPS * this.sensorToMechanismRatio;
    double motorRPS = encoderRPS * this.rotorToSensorRatio;
    this.motorSimState.setRawRotorPosition(motorRotations);
    this.motorSimState.setRotorVelocity(motorRPS);
    this.encoderSimState.setRawPosition(sensorRotations);
    this.encoderSimState.setVelocity(encoderRPS);

    // Update the Mechanism Arm angle based on the simulated arm angle
    arm.setAngle(Units.radiansToDegrees(mechanismRadians));
    Logger.recordOutput(subsystemName + "/ArmSim", mech2d);
  }
}
