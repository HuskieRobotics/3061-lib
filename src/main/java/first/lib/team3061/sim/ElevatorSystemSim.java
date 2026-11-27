package first.lib.team3061.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import first.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.system.RobotController;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/**
 * Simulates an elevator (or other linear mechanism) with a limited range of motion that is driven
 * by a pulley or drum and, optionally, affected by gravity.
 *
 * <p>The simulation assumes that the elevator is a position mechanism controlled with voltage-based
 * control requests (e.g., {@code MotionMagicExpoVoltage} or {@code PositionVoltage}). The model's
 * input is the simulated output voltage of the motor, and kV and kA characterization constants must
 * be in volts. Phoenix calculates the simulated output voltage in every control mode, so the
 * physical-characteristics constructor works with any control mode, but the
 * characterization-constant constructor requires constants determined with voltage control. Torque
 * current (e.g., {@code TorqueCurrentFOC}) is only supported for velocity mechanisms by {@link
 * FlywheelSystemSim}.
 */
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
   * @param startingHeightMeters the initial height of the elevator in meters. The simulated rotor
   *     position is based on the elevator's absolute height (i.e., zero rotor rotations corresponds
   *     to a height of zero meters), not on its height relative to the starting height. Therefore,
   *     unlike a real motor, the simulated rotor position is only zero when the robot is powered on
   *     if the starting height is zero. The subsystem's conversion from rotor rotations to height
   *     should also be based on a height of zero corresponding to zero rotor rotations.
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
    this(
        () ->
            Models.elevatorFromPhysicalConstants(
                gearbox, carriageMassKg, pulleyRadiusMeters, gearRatio),
        gearbox,
        gearRatio,
        pulleyRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        startingHeightMeters,
        simulateGravity,
        subsystemName,
        motors);
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
   * <p>The kV and kA constants must be in volts (i.e., the elevator must be characterized with
   * voltage control) since the model's input is the simulated output voltage of the motor.
   * Constants in amps from characterizing with torque-current control (e.g., TorqueCurrentFOC) are
   * not supported.
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
   * @param startingHeightMeters the initial height of the elevator in meters. The simulated rotor
   *     position is based on the elevator's absolute height (i.e., zero rotor rotations corresponds
   *     to a height of zero meters), not on its height relative to the starting height. Therefore,
   *     unlike a real motor, the simulated rotor position is only zero when the robot is powered on
   *     if the starting height is zero. The subsystem's conversion from rotor rotations to height
   *     should also be based on a height of zero corresponding to zero rotor rotations.
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
    // convert from V/(rot/s) and V/(rot/s²) to V/(m/s) and V/(m/s²)
    this(
        () ->
            Models.elevatorFromSysId(
                kV / (2 * Math.PI * pulleyRadiusMeters), kA / (2 * Math.PI * pulleyRadiusMeters)),
        gearbox,
        gearRatio,
        pulleyRadiusMeters,
        minHeightMeters,
        maxHeightMeters,
        startingHeightMeters,
        simulateGravity,
        subsystemName,
        motors);
  }

  /**
   * Creates a simulation of an elevator based on the specified state-space model. The public
   * constructors build the model from either physical characteristics or characterization constants
   * and delegate to this constructor. This simulation is only created when the robot is running in
   * simulation mode; otherwise, this object does nothing.
   *
   * <p>The model is provided as a supplier so that it is only created in simulation mode (e.g.,
   * creating a model from characterization constants throws an exception if kA is zero, which
   * shouldn't prevent the robot code from running on a real robot).
   *
   * @param plantSupplier supplies the state-space model of the elevator whose states are [position,
   *     velocity] in meters and meters per second, whose input is [voltage], and whose outputs are
   *     [position, velocity]
   * @param gearbox the motor(s) driving the elevator; only used to calculate the simulated current
   *     draw
   * @param gearRatio the gear ratio from the motor's rotor to the pulley (numbers greater than 1
   *     represent reductions), used to convert the elevator's position and velocity to the rotor's
   *     position and velocity
   * @param pulleyRadiusMeters the radius of the pulley (or drum) in meters, used to convert the
   *     elevator's position and velocity to the rotor's position and velocity
   * @param minHeightMeters the minimum height of the elevator in meters
   * @param maxHeightMeters the maximum height of the elevator in meters
   * @param startingHeightMeters the initial height of the elevator in meters. The simulated rotor
   *     position is based on the elevator's absolute height (i.e., zero rotor rotations corresponds
   *     to a height of zero meters), not on its height relative to the starting height. Therefore,
   *     unlike a real motor, the simulated rotor position is only zero when the robot is powered on
   *     if the starting height is zero. The subsystem's conversion from rotor rotations to height
   *     should also be based on a height of zero corresponding to zero rotor rotations.
   * @param simulateGravity true to simulate the effect of gravity on the elevator
   * @param subsystemName the name of the subsystem, used as the prefix when logging the Mechanism2d
   * @param motors the motors driving the elevator; the first motor's output voltage is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  private ElevatorSystemSim(
      Supplier<LinearSystem<N2, N1, N2>> plantSupplier,
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

    this.systemSim =
        new ElevatorSim(
            plantSupplier.get(),
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
