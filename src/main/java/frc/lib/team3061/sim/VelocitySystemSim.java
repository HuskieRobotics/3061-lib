package frc.lib.team3061.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants;

public class VelocitySystemSim {

  private TalonFX motor;
  private TalonFXSimState motorSimState;
  private LinearSystemSim<N1, N1, N1> systemSim;
  private double gearRatio;

  public VelocitySystemSim(
      TalonFX motor, boolean motorInverted, double kV, double kA, double gearRatio) {
    this.motor = motor;
    this.gearRatio = gearRatio;

    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.motorSimState = this.motor.getSimState();
    this.motorSimState.Orientation =
        motorInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    this.systemSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(kV, kA));
  }

  public void updateSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    this.motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // update the input voltages of the models based on the outputs of the simulated TalonFXs
    this.systemSim.setInput(this.motorSimState.getMotorVoltage());

    // update the models
    this.systemSim.update(Constants.LOOP_PERIOD_SECS);

    // update the simulated TalonFX based on the model outputs
    double mechanismRadiansPerSec = this.systemSim.getOutput(0);
    double motorRPS = mechanismRadiansPerSec * this.gearRatio / (2 * Math.PI);
    double motorRotations = motorRPS * Constants.LOOP_PERIOD_SECS;
    this.motorSimState.addRotorPosition(motorRotations);
    this.motorSimState.setRotorVelocity(motorRPS);
  }
}
