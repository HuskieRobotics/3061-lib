package frc.lib.team3061.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class FlywheelSystemSim {

  private TalonFXSimState[] motorSimStates;
  private FlywheelSim systemSim;
  private double gearRatio;

  public FlywheelSystemSim(
      double kV, double kA, double gearRatio, double momentOfInertia, TalonFX... motors) {
    this.gearRatio = gearRatio;

    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.motorSimStates = new TalonFXSimState[motors.length];
    for (int i = 0; i < motors.length; i++) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      motors[i].getConfigurator().refresh(config);
      this.motorSimStates[i] = motors[i].getSimState();
      this.motorSimStates[i].Orientation =
          config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
    }

    DCMotor dcMotors = DCMotor.getKrakenX60Foc(motors.length);
    this.systemSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(dcMotors, momentOfInertia, gearRatio), dcMotors);
  }

  public void updateSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    for (TalonFXSimState motorSimState : this.motorSimStates) {
      motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    // update the input voltages of the models based on the outputs of the simulated TalonFXs
    this.systemSim.setInput(this.motorSimStates[0].getMotorVoltage());

    // update the models
    this.systemSim.update(Constants.LOOP_PERIOD_SECS);

    // update the simulated TalonFX based on the model outputs
    double mechanismRadiansPerSec = this.systemSim.getOutput(0);
    double motorRPS = mechanismRadiansPerSec * this.gearRatio / (2 * Math.PI);
    double motorRotations = motorRPS * Constants.LOOP_PERIOD_SECS;
    for (TalonFXSimState motorSimState : this.motorSimStates) {
      motorSimState.setRotorVelocity(motorRPS);
      motorSimState.addRotorPosition(motorRotations);
    }
  }
}
