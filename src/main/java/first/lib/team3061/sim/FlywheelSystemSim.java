package first.lib.team3061.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import first.robot.Constants;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.system.RobotController;

public class FlywheelSystemSim {

  private TalonFXSimState[] motorSimStates;
  private FlywheelSim systemSim;
  private double gearRatio;

  public FlywheelSystemSim(
      DCMotor gearbox,
      double kV,
      double kA,
      double gearRatio,
      double momentOfInertia,
      TalonFX... motors) {
    this.gearRatio = gearRatio;

    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.motorSimStates = TalonFXSimUtil.createSimStates(motors);

    this.systemSim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(gearbox, momentOfInertia, gearRatio), gearbox);
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
