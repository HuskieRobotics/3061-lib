package first.lib.team3061.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import first.robot.Constants;
import java.util.function.Supplier;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.LinearSystemSim;
import org.wpilib.system.RobotController;

/**
 * Simulates a "flywheel," which, as in WPILib's {@link org.wpilib.simulation.FlywheelSim}, refers
 * to any mechanism that spins freely without a limited range of motion and whose dynamics don't
 * depend on its position. In addition to shooter flywheels, this includes intake rollers, indexers,
 * conveyors, and similar mechanisms that are typically controlled by velocity. Use {@link
 * ArmSystemSim} for mechanisms affected by gravity based on their angle and {@link
 * ElevatorSystemSim} for linear mechanisms with a limited range of motion.
 *
 * <p>This simulation assumes that the mechanism is controlled with torque current (e.g., {@code
 * VelocityTorqueCurrentFOC}) and tuned as described in CTRE's <a
 * href="https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/manual-pid-tuning.html">Manually
 * Tuning PID with TorqueCurrentFOC</a>. The model's input is the simulated torque current of the
 * first motor, and the model is:
 *
 * <pre>
 * I = kS * sign(v) + kV * v + kA * a
 * </pre>
 *
 * <p>where I is the torque current of each motor in amps, v is the velocity of the mechanism in
 * rotations per second, and a is its acceleration in rotations per second squared. kS and kV are
 * the TalonFX's Slot0 gains: kS models kinetic friction, and kV models drag (and may be zero). kA
 * models the inertia of the mechanism. Since the tuning procedure doesn't determine kA (it remains
 * zero in Slot0), kA must either be calculated from the mechanism's moment of inertia or measured
 * on the robot.
 *
 * <p>To measure kA on the robot:
 *
 * <ol>
 *   <li>With the mechanism at rest, apply a constant torque current (e.g., {@code new
 *       TorqueCurrentFOC(20)}) that is well above kS.
 *   <li>Log the mechanism's velocity in rotations per second (i.e., based on the TalonFX's
 *       SensorToMechanismRatio).
 *   <li>Determine the acceleration, a, in rotations per second squared from the slope of the
 *       velocity over the first 0.1-0.2 seconds, before drag becomes significant and before the
 *       mechanism approaches its maximum velocity.
 *   <li>Calculate kA = (I - kS) / a, where I is the applied torque current. Repeat with a couple of
 *       different currents (and in both directions) and average the results.
 * </ol>
 *
 * <p>The simulation doesn't need to model the motor's back-EMF. Phoenix calculates the simulated
 * torque current based on the simulated rotor velocity and supply voltage, so the torque current
 * decreases as the mechanism approaches its maximum velocity, and current limits are applied.
 *
 * <p>The simulated motors' rotor velocities are set based on the simulated velocity of the
 * mechanism, and their rotor positions are updated by integrating that velocity.
 */
public class FlywheelSystemSim {

  private TalonFXSimState[] motorSimStates;
  private LinearSystemSim<N1, N1, N1> systemSim;
  private double gearRatio;
  private double kS;

  /**
   * Creates a simulation of a flywheel (or other freely spinning mechanism, such as an intake
   * roller) based on its kS and kV gains and its moment of inertia. kA is calculated from the
   * moment of inertia, the number of motors, and the motor's torque constant: kA = J / (number of
   * motors * Kt * gearRatio). This simulation is only created when the robot is running in
   * simulation mode; otherwise, this object does nothing.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param kS the static gain in amps (i.e., the TalonFX's Slot0 kS), which models kinetic friction
   * @param kV the velocity gain in amps / (flywheel rotations / second) (i.e., the TalonFX's Slot0
   *     kV), which models drag and may be zero
   * @param momentOfInertia the moment of inertia of the flywheel in kg·m², including all of the
   *     rotating parts driven by the motors (e.g., all of the rollers and shafts of an intake)
   * @param gearbox the type of motor driving the flywheel (e.g., {@code DCMotor.getKrakenX60Foc(1)}
   *     for Kraken X60s using FOC); only its torque constant (Kt) is used, which doesn't depend on
   *     the number of motors
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions); this should match the TalonFX's SensorToMechanismRatio
   * @param motors the motors driving the flywheel; the first motor's torque current is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  public FlywheelSystemSim(
      double kS,
      double kV,
      double momentOfInertia,
      DCMotor gearbox,
      double gearRatio,
      TalonFX... motors) {
    // the torque applied to the flywheel is (number of motors * Kt * gearRatio * I), so the current
    // per motor required for each radian per second squared of acceleration is
    // J / (number of motors * Kt * gearRatio); convert kV from A/(rot/s) to A/(rad/s)
    this(
        () ->
            Models.flywheelFromSysId(
                kV / (2 * Math.PI), momentOfInertia / (motors.length * gearbox.Kt * gearRatio)),
        kS,
        gearRatio,
        motors);
  }

  /**
   * Creates a simulation of a flywheel (or other freely spinning mechanism, such as an intake
   * roller) based on its kS, kV, and kA gains in amps. kA is typically measured on the robot, as
   * described in this class's documentation, since tuning the TalonFX's gains doesn't determine it.
   * This simulation is only created when the robot is running in simulation mode; otherwise, this
   * object does nothing.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param kS the static gain in amps (i.e., the TalonFX's Slot0 kS), which models kinetic friction
   * @param kV the velocity gain in amps / (flywheel rotations / second) (i.e., the TalonFX's Slot0
   *     kV), which models drag and may be zero
   * @param kA the acceleration gain in amps / (flywheel rotations / second²), which models the
   *     flywheel's inertia and must be greater than zero
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions), used to convert the flywheel's velocity to the rotor's velocity;
   *     this should match the TalonFX's SensorToMechanismRatio
   * @param motors the motors driving the flywheel; the first motor's torque current is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  public FlywheelSystemSim(double kS, double kV, double kA, double gearRatio, TalonFX... motors) {
    // convert from A/(rot/s) and A/(rot/s²) to A/(rad/s) and A/(rad/s²)
    this(
        () -> Models.flywheelFromSysId(kV / (2 * Math.PI), kA / (2 * Math.PI)),
        kS,
        gearRatio,
        motors);
  }

  /**
   * Creates a simulation of a flywheel (or other freely spinning mechanism, such as an intake
   * roller) based on the specified state-space model. The public constructors build the model and
   * delegate to this constructor. This simulation is only created when the robot is running in
   * simulation mode; otherwise, this object does nothing.
   *
   * <p>The model is provided as a supplier so that it is only created in simulation mode (e.g.,
   * creating the model throws an exception if kA is zero, which shouldn't prevent the robot code
   * from running on a real robot).
   *
   * @param plantSupplier supplies the state-space model of the flywheel whose state is [angular
   *     velocity] in radians per second, whose input is [torque current] in amps, and whose output
   *     is [angular velocity]
   * @param kS the static gain in amps, which models kinetic friction
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions), used to convert the flywheel's velocity to the rotor's velocity
   * @param motors the motors driving the flywheel; the first motor's torque current is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  private FlywheelSystemSim(
      Supplier<LinearSystem<N1, N1, N1>> plantSupplier,
      double kS,
      double gearRatio,
      TalonFX... motors) {
    this.gearRatio = gearRatio;
    this.kS = kS;

    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.motorSimStates = TalonFXSimUtil.createSimStates(motors);

    this.systemSim = new LinearSystemSim<>(plantSupplier.get());
  }

  public void updateSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    for (TalonFXSimState motorSimState : this.motorSimStates) {
      motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    // update the input of the model based on the torque current of the simulated TalonFX less the
    // current required to overcome friction
    double current = this.motorSimStates[0].getTorqueCurrent();
    double previousRadiansPerSec = this.systemSim.getOutput(0);
    double netCurrent;
    if (previousRadiansPerSec != 0.0) {
      // kinetic friction opposes the direction of motion
      netCurrent = current - Math.copySign(this.kS, previousRadiansPerSec);
    } else if (Math.abs(current) > this.kS) {
      // the current overcomes static friction
      netCurrent = current - Math.copySign(this.kS, current);
    } else {
      // static friction prevents the flywheel from moving
      netCurrent = 0.0;
    }
    this.systemSim.setInput(netCurrent);

    // update the models
    this.systemSim.update(Constants.LOOP_PERIOD_SECS);

    // if friction would reverse the direction of the flywheel within a single step, stop it
    // instead (unless the current is large enough to reverse its direction)
    double mechanismRadiansPerSec = this.systemSim.getOutput(0);
    boolean reversed =
        previousRadiansPerSec != 0.0
            && Math.signum(mechanismRadiansPerSec) != Math.signum(previousRadiansPerSec);
    boolean currentReverses =
        Math.abs(current) > this.kS && Math.signum(current) == Math.signum(mechanismRadiansPerSec);
    if (reversed && !currentReverses) {
      this.systemSim.setState(VecBuilder.fill(0.0));
      mechanismRadiansPerSec = 0.0;
    }

    // update the simulated TalonFX based on the model outputs
    double motorRPS = mechanismRadiansPerSec * this.gearRatio / (2 * Math.PI);
    double motorRotations = motorRPS * Constants.LOOP_PERIOD_SECS;
    for (TalonFXSimState motorSimState : this.motorSimStates) {
      motorSimState.setRotorVelocity(motorRPS);
      motorSimState.addRotorPosition(motorRotations);
    }
  }
}
