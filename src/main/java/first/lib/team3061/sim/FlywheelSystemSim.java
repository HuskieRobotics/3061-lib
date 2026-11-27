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
import org.wpilib.system.RobotController;

public class FlywheelSystemSim {

  private TalonFXSimState[] motorSimStates;
  private FlywheelSim systemSim;
  private double gearRatio;

  /**
   * Creates a simulation of a flywheel based on its physical characteristics. This simulation is
   * only created when the robot is running in simulation mode; otherwise, this object does nothing.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param gearbox the motor(s) driving the flywheel (e.g., {@code DCMotor.getKrakenX60Foc(2)} for
   *     two Kraken X60s using FOC)
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions); this should match the TalonFX's SensorToMechanismRatio
   * @param momentOfInertia the moment of inertia of the flywheel in kg·m²
   * @param motors the motors driving the flywheel; the first motor's output voltage is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  public FlywheelSystemSim(
      DCMotor gearbox, double gearRatio, double momentOfInertia, TalonFX... motors) {
    this(
        () -> Models.flywheelFromPhysicalConstants(gearbox, momentOfInertia, gearRatio),
        gearbox,
        gearRatio,
        motors);
  }

  /**
   * Creates a simulation of a flywheel based on its characterization constants (e.g., as determined
   * by SysId). This simulation is only created when the robot is running in simulation mode;
   * otherwise, this object does nothing.
   *
   * <p>The kV and kA constants are expected in the same units as the TalonFX's Slot0 gains when the
   * device is configured with a SensorToMechanismRatio (i.e., volts per flywheel rotation per
   * second and volts per flywheel rotation per second squared). They are converted to radians
   * internally.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param kV the velocity gain in volts / (flywheel rotations / second)
   * @param kA the acceleration gain in volts / (flywheel rotations / second²)
   * @param gearbox the motor(s) driving the flywheel (e.g., {@code DCMotor.getKrakenX60Foc(2)} for
   *     two Kraken X60s using FOC); only used to calculate the simulated current draw
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions), used to convert the flywheel's velocity to the rotor's velocity;
   *     this should match the TalonFX's SensorToMechanismRatio
   * @param motors the motors driving the flywheel; the first motor's output voltage is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  public FlywheelSystemSim(
      double kV, double kA, DCMotor gearbox, double gearRatio, TalonFX... motors) {
    // convert from V/(rot/s) and V/(rot/s²) to V/(rad/s) and V/(rad/s²)
    this(
        () -> Models.flywheelFromSysId(kV / (2 * Math.PI), kA / (2 * Math.PI)),
        gearbox,
        gearRatio,
        motors);
  }

  /**
   * Creates a simulation of a flywheel based on the specified state-space model. The public
   * constructors build the model from either physical characteristics or characterization constants
   * and delegate to this constructor. This simulation is only created when the robot is running in
   * simulation mode; otherwise, this object does nothing.
   *
   * <p>The model is provided as a supplier so that it is only created in simulation mode (e.g.,
   * creating a model from characterization constants throws an exception if kA is zero, which
   * shouldn't prevent the robot code from running on a real robot).
   *
   * @param plantSupplier supplies the state-space model of the flywheel whose state is [angular
   *     velocity] in radians per second, whose input is [voltage], and whose output is [angular
   *     velocity]
   * @param gearbox the motor(s) driving the flywheel; used to calculate the simulated current draw
   * @param gearRatio the gear ratio from the motor's rotor to the flywheel (numbers greater than 1
   *     represent reductions), used to convert the flywheel's velocity to the rotor's velocity
   * @param motors the motors driving the flywheel; the first motor's output voltage is used as the
   *     input to the simulation, so it must be the leader if any followers are specified
   */
  private FlywheelSystemSim(
      Supplier<LinearSystem<N1, N1, N1>> plantSupplier,
      DCMotor gearbox,
      double gearRatio,
      TalonFX... motors) {
    this.gearRatio = gearRatio;

    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.motorSimStates = TalonFXSimUtil.createSimStates(motors);

    this.systemSim = new FlywheelSim(plantSupplier.get(), gearbox);
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
