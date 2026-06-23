package frc.lib.team3061.sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorSystemSim {

  private TalonFXSimState[] motorSimStates;
  private ElevatorSim systemSim;
  private double gearRatio;
  private double pulleyRadiusMeters;
  private String subsystemName;

  // Create a Mechanism2d display of an elevator with a fixed stage and a single extending stage.
  private LoggedMechanism2d mech2d;
  private LoggedMechanismRoot2d elevatorBase;
  private LoggedMechanismLigament2d elevatorExtension;

  public ElevatorSystemSim(
      DCMotor gearbox,
      double gearRatio,
      double carriageMassKg,
      double pulleyRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double startingHeightMeters,
      String subsystemName,
      TalonFX... motors) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.gearRatio = gearRatio;
    this.pulleyRadiusMeters = pulleyRadiusMeters;

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

    this.systemSim =
        new ElevatorSim(
            gearbox,
            gearRatio,
            carriageMassKg,
            pulleyRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            true,
            startingHeightMeters);

    this.mech2d = new LoggedMechanism2d(1, 1);
    this.elevatorBase = mech2d.getRoot("ElevatorBase", 0.7, 0.1);
    this.elevatorExtension =
        elevatorBase.append(
            new LoggedMechanismLigament2d(
                "ElevatorExtension", minHeightMeters, 90.0, 6, new Color8Bit(Color.kYellow)));

    this.subsystemName = subsystemName;
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
    double motorVoltage = this.motorSimStates[0].getMotorVoltage();
    this.systemSim.setInput(motorVoltage);

    // update the models
    this.systemSim.update(Constants.LOOP_PERIOD_SECS);

    // update the simulated TalonFX based on the model outputs
    double mechanismLength = this.systemSim.getPositionMeters();
    double pulleyRotations = mechanismLength / (2 * Math.PI * this.pulleyRadiusMeters);
    double motorRotations = pulleyRotations * this.gearRatio;
    double mechanismMetersPerSec = this.systemSim.getVelocityMetersPerSecond();
    double pulleyRPS = mechanismMetersPerSec / (2 * Math.PI * this.pulleyRadiusMeters);
    double motorRPS = pulleyRPS * this.gearRatio;

    for (TalonFXSimState motorSimState : this.motorSimStates) {
      motorSimState.setRotorVelocity(motorRPS);
      motorSimState.setRawRotorPosition(motorRotations);
    }

    // Update the Mechanism based on the simulated elevator position
    this.elevatorExtension.setLength(mechanismLength);
    Logger.recordOutput(subsystemName + "/ElevatorSim", mech2d);
  }
}
