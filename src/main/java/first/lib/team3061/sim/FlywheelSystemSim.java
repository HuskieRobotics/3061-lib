package first.lib.team3061.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import first.robot.Constants;
import java.util.function.Supplier;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.LinearSystemSim;
import org.wpilib.system.RobotController;

/**
 * Simulates a "flywheel," which, as in WPILib's {@link FlywheelSim}, refers to any mechanism that
 * spins freely without a limited range of motion and whose dynamics don't depend on its position.
 * In addition to shooter flywheels, this includes intake rollers, indexers, conveyors, and similar
 * mechanisms that are typically controlled by velocity or voltage. Use {@link ArmSystemSim} for
 * mechanisms affected by gravity based on their angle and {@link ElevatorSystemSim} for linear
 * mechanisms with a limited range of motion.
 *
 * <p>The simulated motors' rotor velocities are set based on the simulated velocity of the
 * mechanism, and their rotor positions are updated by integrating that velocity.
 *
 * <p>Velocity mechanisms are often controlled with torque current (e.g., {@code
 * VelocityTorqueCurrentFOC}), in which case characterizing them with SysId produces kV and kA in
 * amps rather than volts. The characterization-constant constructor therefore takes the {@link
 * CharacterizationUnits} of kV and kA. With {@link CharacterizationUnits#AMPS}, the model's input
 * is the simulated torque current of the motor instead of its voltage. Phoenix calculates the
 * simulated torque current in every control mode, so the motors may be controlled in any mode; only
 * the units of kV and kA determine which one to specify.
 */
public class FlywheelSystemSim {

  /** The units of the kV and kA characterization constants, which determine the model's input. */
  public enum CharacterizationUnits {
    /**
     * kV and kA are in volts (e.g., characterized with voltage control); the model's input is the
     * simulated output voltage of the motor.
     */
    VOLTS,
    /**
     * kV and kA are in amps (e.g., characterized with TorqueCurrentFOC control); the model's input
     * is the simulated torque current of the motor.
     */
    AMPS
  }

  private TalonFXSimState[] motorSimStates;
  private LinearSystemSim<N1, N1, N1> systemSim;
  private double gearRatio;
  private CharacterizationUnits characterizationUnits;

  /**
   * Creates a simulation of a flywheel (or other freely spinning mechanism, such as an intake
   * roller) based on its physical characteristics. This simulation is only created when the robot
   * is running in simulation mode; otherwise, this object does nothing.
   *
   * <p>The model's input is the simulated output voltage of the motor. Phoenix calculates this
   * voltage in every control mode (including TorqueCurrentFOC), so the motors may be controlled in
   * any mode.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param gearbox the motor(s) driving the flywheel (e.g., {@code DCMotor.getKrakenX60Foc(2)} for
   *     two Kraken X60s using FOC)
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions); this should match the TalonFX's SensorToMechanismRatio
   * @param momentOfInertia the moment of inertia of the flywheel in kg·m², including all of the
   *     rotating parts driven by the motors (e.g., all of the rollers and shafts of an intake)
   * @param motors the motors driving the flywheel; the first motor's output voltage is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  public FlywheelSystemSim(
      DCMotor gearbox, double gearRatio, double momentOfInertia, TalonFX... motors) {
    this(
        () -> Models.flywheelFromPhysicalConstants(gearbox, momentOfInertia, gearRatio),
        gearbox,
        gearRatio,
        CharacterizationUnits.VOLTS,
        motors);
  }

  /**
   * Creates a simulation of a flywheel (or other freely spinning mechanism, such as an intake
   * roller) based on its characterization constants (e.g., as determined by SysId). This simulation
   * is only created when the robot is running in simulation mode; otherwise, this object does
   * nothing.
   *
   * <p>The kV and kA constants are expected in the same units as the TalonFX's Slot0 gains when the
   * device is configured with a SensorToMechanismRatio (i.e., volts or amps per flywheel rotation
   * per second and volts or amps per flywheel rotation per second squared). They are converted to
   * radians internally.
   *
   * <p>If the flywheel is characterized with voltage control, specify {@link
   * CharacterizationUnits#VOLTS}, and the model's input is the simulated output voltage of the
   * motor. If the flywheel is characterized with torque-current control (e.g., TorqueCurrentFOC),
   * specify {@link CharacterizationUnits#AMPS}, and the model's input is the simulated torque
   * current of the first motor. In this case, kV and kA are expected to be based on the torque
   * current of a single motor (e.g., the leader's torque current logged by SysId) with all of the
   * specified motors driving the flywheel.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param kV the velocity gain in volts or amps / (flywheel rotations / second)
   * @param kA the acceleration gain in volts or amps / (flywheel rotations / second²)
   * @param characterizationUnits the units of kV and kA, which determine whether the model's input
   *     is the motor's simulated output voltage or torque current
   * @param gearbox the motor(s) driving the flywheel (e.g., {@code DCMotor.getKrakenX60Foc(2)} for
   *     two Kraken X60s using FOC); only used to calculate the simulated current draw and ignored
   *     if characterizationUnits is {@link CharacterizationUnits#AMPS}
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions), used to convert the flywheel's velocity to the rotor's velocity;
   *     this should match the TalonFX's SensorToMechanismRatio
   * @param motors the motors driving the flywheel; the first motor's output voltage or torque
   *     current is used as the input to the simulation, so it must be the leader if any followers
   *     are specified
   */
  public FlywheelSystemSim(
      double kV,
      double kA,
      CharacterizationUnits characterizationUnits,
      DCMotor gearbox,
      double gearRatio,
      TalonFX... motors) {
    // convert from units/(rot/s) and units/(rot/s²) to units/(rad/s) and units/(rad/s²); the model
    // has the same form whether its input is in volts or amps
    this(
        () -> Models.flywheelFromSysId(kV / (2 * Math.PI), kA / (2 * Math.PI)),
        gearbox,
        gearRatio,
        characterizationUnits,
        motors);
  }

  /**
   * Creates a simulation of a flywheel (or other freely spinning mechanism, such as an intake
   * roller) based on the specified state-space model. The public constructors build the model from
   * either physical characteristics or characterization constants and delegate to this constructor.
   * This simulation is only created when the robot is running in simulation mode; otherwise, this
   * object does nothing.
   *
   * <p>The model is provided as a supplier so that it is only created in simulation mode (e.g.,
   * creating a model from characterization constants throws an exception if kA is zero, which
   * shouldn't prevent the robot code from running on a real robot).
   *
   * @param plantSupplier supplies the state-space model of the flywheel whose state is [angular
   *     velocity] in radians per second, whose input is [voltage] or [torque current] (as specified
   *     by characterizationUnits), and whose output is [angular velocity]
   * @param gearbox the motor(s) driving the flywheel; used to calculate the simulated current draw
   *     and ignored if characterizationUnits is {@link CharacterizationUnits#AMPS}
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions), used to convert the flywheel's velocity to the rotor's velocity
   * @param characterizationUnits the units of the model's input
   * @param motors the motors driving the flywheel; the first motor's output voltage or torque
   *     current is used as the input to the simulation, so it must be the leader if any followers
   *     are specified
   */
  private FlywheelSystemSim(
      Supplier<LinearSystem<N1, N1, N1>> plantSupplier,
      DCMotor gearbox,
      double gearRatio,
      CharacterizationUnits characterizationUnits,
      TalonFX... motors) {
    this.gearRatio = gearRatio;
    this.characterizationUnits = characterizationUnits;

    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.motorSimStates = TalonFXSimUtil.createSimStates(motors);

    // FlywheelSim derives the gearing and moment of inertia from the model and the gearbox, which
    // is only valid if the model's input is voltage; otherwise, simulate the model directly
    if (characterizationUnits == CharacterizationUnits.AMPS) {
      this.systemSim = new LinearSystemSim<>(plantSupplier.get());
    } else {
      this.systemSim = new FlywheelSim(plantSupplier.get(), gearbox);
    }
  }

  public void updateSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    for (TalonFXSimState motorSimState : this.motorSimStates) {
      motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    // update the input of the model based on the output voltage or torque current of the simulated
    // TalonFX, depending on the units of the characterization constants
    if (this.characterizationUnits == CharacterizationUnits.AMPS) {
      this.systemSim.setInput(this.motorSimStates[0].getTorqueCurrent());
    } else {
      this.systemSim.setInput(this.motorSimStates[0].getMotorVoltage());
    }

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
