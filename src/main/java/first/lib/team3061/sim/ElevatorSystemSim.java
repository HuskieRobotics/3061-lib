package first.lib.team3061.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import first.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.system.RobotController;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

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

  /**
   * Creates a simulation of an elevator based on its physical characteristics. This simulation is
   * only created when the robot is running in simulation mode; otherwise, this object does nothing.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param gearbox the motor(s) driving the elevator (e.g., {@code DCMotor.getKrakenX60Foc(2)} for
   *     two Kraken X60s using FOC)
   * @param gearRatio the gear ratio from the motor's rotor to the pulley (numbers greater than 1
   *     represent reductions). If a fused CANcoder is used, this is the product of the
   *     RotorToSensorRatio and the SensorToMechanismRatio.
   * @param carriageMassKg the mass of the carriage in kilograms
   * @param pulleyRadiusMeters the radius of the pulley (or drum) in meters
   * @param minHeightMeters the minimum height of the elevator in meters
   * @param maxHeightMeters the maximum height of the elevator in meters
   * @param startingHeightMeters the initial height of the elevator in meters
   * @param simulateGravity true to simulate the effect of gravity on the elevator
   * @param subsystemName the name of the subsystem, used as the prefix when logging the Mechanism2d
   * @param motors the motors driving the elevator; the first motor's output voltage is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  public ElevatorSystemSim(
      DCMotor gearbox,
      double gearRatio,
      double carriageMassKg,
      double pulleyRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double startingHeightMeters,
      boolean simulateGravity,
      String subsystemName,
      TalonFX... motors) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.gearRatio = gearRatio;
    this.pulleyRadiusMeters = pulleyRadiusMeters;

    this.motorSimStates = TalonFXSimUtil.createSimStates(motors);

    this.systemSim =
        new ElevatorSim(
            gearbox,
            gearRatio,
            carriageMassKg,
            pulleyRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            simulateGravity,
            startingHeightMeters);

    this.mech2d = new LoggedMechanism2d(1, 1);
    this.elevatorBase = mech2d.getRoot("ElevatorBase", 0.7, 0.1);
    this.elevatorExtension =
        elevatorBase.append(
            new LoggedMechanismLigament2d(
                "ElevatorExtension", minHeightMeters, 90.0, 6, new Color8Bit(Color.YELLOW)));

    this.subsystemName = subsystemName;
  }

  /**
   * Creates a simulation of an elevator based on its characterization constants (e.g., as
   * determined by SysId). This simulation is only created when the robot is running in simulation
   * mode; otherwise, this object does nothing.
   *
   * <p>The kV and kA constants are expected in the same units as the TalonFX's Slot0 gains when the
   * device is configured with a SensorToMechanismRatio (i.e., volts per pulley rotation per second
   * and volts per pulley rotation per second squared). They are converted to linear units based on
   * the pulley radius. When gravity is simulated, a constant acceleration of -9.8 m/s² is applied;
   * kG is not used.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param kV the velocity gain in volts / (pulley rotations / second)
   * @param kA the acceleration gain in volts / (pulley rotations / second²)
   * @param gearbox the motor(s) driving the elevator (e.g., {@code DCMotor.getKrakenX60Foc(2)} for
   *     two Kraken X60s using FOC); only used to calculate the simulated current draw
   * @param gearRatio the gear ratio from the motor's rotor to the pulley (numbers greater than 1
   *     represent reductions), used to convert the elevator's position and velocity to the rotor's
   *     position and velocity. If a fused CANcoder is used, this is the product of the
   *     RotorToSensorRatio and the SensorToMechanismRatio.
   * @param pulleyRadiusMeters the radius of the pulley (or drum) in meters
   * @param minHeightMeters the minimum height of the elevator in meters
   * @param maxHeightMeters the maximum height of the elevator in meters
   * @param startingHeightMeters the initial height of the elevator in meters
   * @param simulateGravity true to simulate the effect of gravity on the elevator
   * @param subsystemName the name of the subsystem, used as the prefix when logging the Mechanism2d
   * @param motors the motors driving the elevator; the first motor's output voltage is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  public ElevatorSystemSim(
      double kV,
      double kA,
      DCMotor gearbox,
      double gearRatio,
      double pulleyRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double startingHeightMeters,
      boolean simulateGravity,
      String subsystemName,
      TalonFX... motors) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.gearRatio = gearRatio;
    this.pulleyRadiusMeters = pulleyRadiusMeters;

    this.motorSimStates = TalonFXSimUtil.createSimStates(motors);

    double metersPerMechanismRotation = 2 * Math.PI * pulleyRadiusMeters;
    this.systemSim =
        new ElevatorSim(
            kV / metersPerMechanismRotation,
            kA / metersPerMechanismRotation,
            gearbox,
            minHeightMeters,
            maxHeightMeters,
            simulateGravity,
            startingHeightMeters);

    this.mech2d = new LoggedMechanism2d(1, 1);
    this.elevatorBase = mech2d.getRoot("ElevatorBase", 0.7, 0.1);
    this.elevatorExtension =
        elevatorBase.append(
            new LoggedMechanismLigament2d(
                "ElevatorExtension", minHeightMeters, 90.0, 6, new Color8Bit(Color.YELLOW)));

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
    double mechanismLength = this.systemSim.getPosition();
    double pulleyRotations = mechanismLength / (2 * Math.PI * this.pulleyRadiusMeters);
    double motorRotations = pulleyRotations * this.gearRatio;
    double mechanismMetersPerSec = this.systemSim.getVelocity();
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
