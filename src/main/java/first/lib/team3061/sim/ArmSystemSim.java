package first.lib.team3061.sim;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
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
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

public class ArmSystemSim {

  private TalonFXSimState[] motorSimStates;
  private SingleJointedArmSim systemSim;
  private double sensorToMechanismRatio;
  private double startingAngleRad;
  private String subsystemName;

  private boolean hasExternalEncoder;
  // these are only defined if the encoder is external
  private CANcoder encoder;
  private CANcoderSimState encoderSimState;
  private double rotorToSensorRatio;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private LoggedMechanism2d mech2d;
  private LoggedMechanismRoot2d armPivot;
  private LoggedMechanismLigament2d armTower;
  private LoggedMechanismLigament2d arm;

  /**
   * Creates a simulation of a single-jointed arm based on its physical characteristics, where the
   * arm's position is measured by the motor's integrated rotor sensor. This simulation is only
   * created when the robot is running in simulation mode; otherwise, this object does nothing.
   *
   * <p>Angles are measured from horizontal, with counterclockwise positive. The simulated rotor
   * position is relative to the starting angle (i.e., it reads zero at the starting angle), which
   * matches the behavior of a real motor when the robot is powered on.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param gearbox the motor(s) driving the arm (e.g., {@code DCMotor.getKrakenX60Foc(2)} for two
   *     Kraken X60s using FOC)
   * @param sensorToMechanismRatio the gear ratio from the motor's rotor to the arm (numbers greater
   *     than 1 represent reductions); this should match the TalonFX's SensorToMechanismRatio
   * @param length the length of the arm in meters
   * @param mass the mass of the arm in kilograms
   * @param minAngle the minimum angle of the arm in radians
   * @param maxAngle the maximum angle of the arm in radians
   * @param startingAngle the initial angle of the arm in radians
   * @param simulateGravity true to simulate the effect of gravity on the arm
   * @param subsystemName the name of the subsystem, used as the prefix when logging the Mechanism2d
   * @param motors the motors driving the arm; the first motor's output voltage is used as the input
   *     to the simulation, so it must be the leader if any followers are specified
   */
  public ArmSystemSim(
      DCMotor gearbox,
      double sensorToMechanismRatio,
      double length,
      double mass,
      double minAngle,
      double maxAngle,
      double startingAngle,
      boolean simulateGravity,
      String subsystemName,
      TalonFX... motors) {
    this(
        gearbox,
        null,
        sensorToMechanismRatio,
        1,
        length,
        mass,
        minAngle,
        maxAngle,
        startingAngle,
        simulateGravity,
        subsystemName,
        motors);
  }

  /**
   * Creates a simulation of a single-jointed arm based on its physical characteristics, where the
   * arm's position is measured by a CANcoder (e.g., remote or fused). This simulation is only
   * created when the robot is running in simulation mode; otherwise, this object does nothing.
   *
   * <p>Angles are measured from horizontal, with counterclockwise positive. The simulated CANcoder
   * reports the absolute angle of the arm; the simulated rotor position is relative to the starting
   * angle (i.e., it reads zero at the starting angle), which matches the behavior of real devices
   * when the robot is powered on.
   *
   * <p>The CANcoder must be configured before this object is constructed. Its MagnetOffset and
   * SensorDirection are read from its configuration so that the simulated CANcoder reports the
   * arm's angle regardless of those settings.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param gearbox the motor(s) driving the arm (e.g., {@code DCMotor.getKrakenX60Foc(2)} for two
   *     Kraken X60s using FOC)
   * @param encoder the CANcoder measuring the arm's position, or null if the motor's integrated
   *     rotor sensor is used
   * @param sensorToMechanismRatio the gear ratio from the CANcoder to the arm (numbers greater than
   *     1 represent reductions); this should match the TalonFX's SensorToMechanismRatio
   * @param rotorToSensorRatio the gear ratio from the motor's rotor to the CANcoder (numbers
   *     greater than 1 represent reductions); this should match the TalonFX's RotorToSensorRatio
   *     and is ignored if encoder is null
   * @param length the length of the arm in meters
   * @param mass the mass of the arm in kilograms
   * @param minAngle the minimum angle of the arm in radians
   * @param maxAngle the maximum angle of the arm in radians
   * @param startingAngle the initial angle of the arm in radians
   * @param simulateGravity true to simulate the effect of gravity on the arm
   * @param subsystemName the name of the subsystem, used as the prefix when logging the Mechanism2d
   * @param motors the motors driving the arm; the first motor's output voltage is used as the input
   *     to the simulation, so it must be the leader if any followers are specified
   */
  public ArmSystemSim(
      DCMotor gearbox,
      CANcoder encoder,
      double sensorToMechanismRatio,
      double rotorToSensorRatio,
      double length,
      double mass,
      double minAngle,
      double maxAngle,
      double startingAngle,
      boolean simulateGravity,
      String subsystemName,
      TalonFX... motors) {
    this(
        () ->
            Models.singleJointedArmFromPhysicalConstants(
                gearbox,
                SingleJointedArmSim.estimateMOI(length, mass),
                sensorToMechanismRatio * rotorToSensorRatio),
        gearbox,
        encoder,
        sensorToMechanismRatio,
        rotorToSensorRatio,
        length,
        minAngle,
        maxAngle,
        startingAngle,
        simulateGravity,
        subsystemName,
        motors);
  }

  /**
   * Creates a simulation of a single-jointed arm based on its characterization constants (e.g., as
   * determined by SysId), where the arm's position is measured by the motor's integrated rotor
   * sensor. This simulation is only created when the robot is running in simulation mode;
   * otherwise, this object does nothing.
   *
   * <p>The kV and kA constants are expected in the same units as the TalonFX's Slot0 gains when the
   * device is configured with a SensorToMechanismRatio (i.e., volts per arm rotation per second and
   * volts per arm rotation per second squared). They are converted to radians internally. When
   * gravity is simulated, it is modeled based on the arm's length; kG is not used.
   *
   * <p>Angles are measured from horizontal, with counterclockwise positive. The simulated rotor
   * position is relative to the starting angle (i.e., it reads zero at the starting angle), which
   * matches the behavior of a real motor when the robot is powered on.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param kV the velocity gain in volts / (arm rotations / second)
   * @param kA the acceleration gain in volts / (arm rotations / second²)
   * @param gearbox the motor(s) driving the arm (e.g., {@code DCMotor.getKrakenX60Foc(2)} for two
   *     Kraken X60s using FOC); only used to calculate the simulated current draw
   * @param sensorToMechanismRatio the gear ratio from the motor's rotor to the arm (numbers greater
   *     than 1 represent reductions); this should match the TalonFX's SensorToMechanismRatio
   * @param length the length of the arm in meters, used to simulate gravity
   * @param minAngle the minimum angle of the arm in radians
   * @param maxAngle the maximum angle of the arm in radians
   * @param startingAngle the initial angle of the arm in radians
   * @param simulateGravity true to simulate the effect of gravity on the arm
   * @param subsystemName the name of the subsystem, used as the prefix when logging the Mechanism2d
   * @param motors the motors driving the arm; the first motor's output voltage is used as the input
   *     to the simulation, so it must be the leader if any followers are specified
   */
  public ArmSystemSim(
      double kV,
      double kA,
      DCMotor gearbox,
      double sensorToMechanismRatio,
      double length,
      double minAngle,
      double maxAngle,
      double startingAngle,
      boolean simulateGravity,
      String subsystemName,
      TalonFX... motors) {
    this(
        kV,
        kA,
        gearbox,
        null,
        sensorToMechanismRatio,
        1,
        length,
        minAngle,
        maxAngle,
        startingAngle,
        simulateGravity,
        subsystemName,
        motors);
  }

  /**
   * Creates a simulation of a single-jointed arm based on its characterization constants (e.g., as
   * determined by SysId), where the arm's position is measured by a CANcoder (e.g., remote or
   * fused). This simulation is only created when the robot is running in simulation mode;
   * otherwise, this object does nothing.
   *
   * <p>The kV and kA constants are expected in the same units as the TalonFX's Slot0 gains when the
   * device is configured with a SensorToMechanismRatio (i.e., volts per arm rotation per second and
   * volts per arm rotation per second squared). They are converted to radians internally. When
   * gravity is simulated, it is modeled based on the arm's length; kG is not used.
   *
   * <p>Angles are measured from horizontal, with counterclockwise positive. The simulated CANcoder
   * reports the absolute angle of the arm; the simulated rotor position is relative to the starting
   * angle (i.e., it reads zero at the starting angle), which matches the behavior of real devices
   * when the robot is powered on.
   *
   * <p>The CANcoder must be configured before this object is constructed. Its MagnetOffset and
   * SensorDirection are read from its configuration so that the simulated CANcoder reports the
   * arm's angle regardless of those settings.
   *
   * <p>If any of the specified motors are followers, their leaders must also be specified, and the
   * follower control request must be applied before this object is constructed. The orientation of
   * each simulated motor is determined from its (or its leader's) configuration.
   *
   * @param kV the velocity gain in volts / (arm rotations / second)
   * @param kA the acceleration gain in volts / (arm rotations / second²)
   * @param gearbox the motor(s) driving the arm (e.g., {@code DCMotor.getKrakenX60Foc(2)} for two
   *     Kraken X60s using FOC); only used to calculate the simulated current draw
   * @param encoder the CANcoder measuring the arm's position, or null if the motor's integrated
   *     rotor sensor is used
   * @param sensorToMechanismRatio the gear ratio from the CANcoder to the arm (numbers greater than
   *     1 represent reductions); this should match the TalonFX's SensorToMechanismRatio
   * @param rotorToSensorRatio the gear ratio from the motor's rotor to the CANcoder (numbers
   *     greater than 1 represent reductions); this should match the TalonFX's RotorToSensorRatio
   *     and is ignored if encoder is null
   * @param length the length of the arm in meters, used to simulate gravity
   * @param minAngle the minimum angle of the arm in radians
   * @param maxAngle the maximum angle of the arm in radians
   * @param startingAngle the initial angle of the arm in radians
   * @param simulateGravity true to simulate the effect of gravity on the arm
   * @param subsystemName the name of the subsystem, used as the prefix when logging the Mechanism2d
   * @param motors the motors driving the arm; the first motor's output voltage is used as the input
   *     to the simulation, so it must be the leader if any followers are specified
   */
  public ArmSystemSim(
      double kV,
      double kA,
      DCMotor gearbox,
      CANcoder encoder,
      double sensorToMechanismRatio,
      double rotorToSensorRatio,
      double length,
      double minAngle,
      double maxAngle,
      double startingAngle,
      boolean simulateGravity,
      String subsystemName,
      TalonFX... motors) {
    // convert from V/(rot/s) and V/(rot/s²) to V/(rad/s) and V/(rad/s²)
    this(
        () -> Models.singleJointedArmFromSysId(kV / (2 * Math.PI), kA / (2 * Math.PI)),
        gearbox,
        encoder,
        sensorToMechanismRatio,
        rotorToSensorRatio,
        length,
        minAngle,
        maxAngle,
        startingAngle,
        simulateGravity,
        subsystemName,
        motors);
  }

  /**
   * Creates a simulation of a single-jointed arm based on the specified state-space model. The
   * other constructors build the model from either physical characteristics or characterization
   * constants and delegate to this constructor. This simulation is only created when the robot is
   * running in simulation mode; otherwise, this object does nothing.
   *
   * <p>The model is provided as a supplier so that it is only created in simulation mode (e.g.,
   * creating a model from characterization constants throws an exception if kA is zero, which
   * shouldn't prevent the robot code from running on a real robot).
   *
   * <p>Angles are measured from horizontal, with counterclockwise positive. If a CANcoder is
   * specified, it must be configured before this object is constructed. If any of the specified
   * motors are followers, their leaders must also be specified, and the follower control request
   * must be applied before this object is constructed.
   *
   * @param plantSupplier supplies the state-space model of the arm whose states are [angle, angular
   *     velocity] in radians and radians per second, whose input is [voltage], and whose outputs
   *     are [angle, angular velocity]
   * @param gearbox the motor(s) driving the arm; only used to calculate the simulated current draw
   * @param encoder the CANcoder measuring the arm's position, or null if the motor's integrated
   *     rotor sensor is used
   * @param sensorToMechanismRatio the gear ratio from the sensor (the CANcoder, or the rotor if
   *     encoder is null) to the arm (numbers greater than 1 represent reductions); this should
   *     match the TalonFX's SensorToMechanismRatio
   * @param rotorToSensorRatio the gear ratio from the motor's rotor to the CANcoder (numbers
   *     greater than 1 represent reductions); this should match the TalonFX's RotorToSensorRatio
   *     and is ignored if encoder is null
   * @param length the length of the arm in meters, used to simulate gravity and to display the arm
   * @param minAngle the minimum angle of the arm in radians
   * @param maxAngle the maximum angle of the arm in radians
   * @param startingAngle the initial angle of the arm in radians
   * @param simulateGravity true to simulate the effect of gravity on the arm
   * @param subsystemName the name of the subsystem, used as the prefix when logging the Mechanism2d
   * @param motors the motors driving the arm; the first motor's output voltage is used as the input
   *     to the simulation, so it must be the leader if any followers are specified
   */
  private ArmSystemSim(
      Supplier<LinearSystem<N2, N1, N2>> plantSupplier,
      DCMotor gearbox,
      CANcoder encoder,
      double sensorToMechanismRatio,
      double rotorToSensorRatio,
      double length,
      double minAngle,
      double maxAngle,
      double startingAngle,
      boolean simulateGravity,
      String subsystemName,
      TalonFX... motors) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.sensorToMechanismRatio = sensorToMechanismRatio;
    this.startingAngleRad = startingAngle;

    this.motorSimStates = TalonFXSimUtil.createSimStates(motors);

    this.hasExternalEncoder = encoder != null;
    if (this.hasExternalEncoder) {
      this.encoder = encoder;
      this.rotorToSensorRatio = rotorToSensorRatio;
      this.encoderSimState = this.encoder.getSimState();

      MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
      this.encoder.getConfigurator().refresh(magnetConfig);
      this.encoderSimState.SensorOffset = magnetConfig.MagnetOffset;
      this.encoderSimState.Orientation =
          magnetConfig.SensorDirection == SensorDirectionValue.Clockwise_Positive
              ? ChassisReference.Clockwise_Positive
              : ChassisReference.CounterClockwise_Positive;
    } else {
      this.rotorToSensorRatio = 1;
    }

    this.systemSim =
        new SingleJointedArmSim(
            plantSupplier.get(),
            gearbox,
            sensorToMechanismRatio * this.rotorToSensorRatio,
            length,
            minAngle,
            maxAngle,
            simulateGravity,
            startingAngle);

    this.mech2d = new LoggedMechanism2d(1, 1);
    this.armPivot = mech2d.getRoot("ArmPivot", 0.5, 0.5);
    this.armTower = armPivot.append(new LoggedMechanismLigament2d("ArmTower", .3, -90));
    this.arm =
        armPivot.append(
            new LoggedMechanismLigament2d(
                "Arm",
                length,
                Units.radiansToDegrees(startingAngle),
                6,
                new Color8Bit(Color.YELLOW)));
    this.armTower.setColor(new Color8Bit(Color.BLUE));

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

    if (this.hasExternalEncoder) {
      this.encoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    // update the input voltages of the models based on the outputs of the simulated TalonFXs
    double motorVoltage = this.motorSimStates[0].getMotorVoltage();
    this.systemSim.setInput(motorVoltage);

    // update the models
    this.systemSim.update(Constants.LOOP_PERIOD_SECS);

    // update the simulated TalonFX based on the model outputs
    double mechanismRadians = this.systemSim.getAngle();
    double mechanismRotations = mechanismRadians / (2 * Math.PI);
    double sensorRotations = mechanismRotations * this.sensorToMechanismRatio;
    double motorRotations =
        Units.radiansToRotations(mechanismRadians - this.startingAngleRad)
            * this.sensorToMechanismRatio
            * this.rotorToSensorRatio;
    double mechanismRadiansPerSec = this.systemSim.getVelocity();
    double mechanismRPS = mechanismRadiansPerSec / (2 * Math.PI);
    double encoderRPS = mechanismRPS * this.sensorToMechanismRatio;
    double motorRPS = encoderRPS * this.rotorToSensorRatio;

    for (TalonFXSimState motorSimState : this.motorSimStates) {
      motorSimState.setRotorVelocity(motorRPS);
      motorSimState.setRawRotorPosition(motorRotations);
    }

    if (this.hasExternalEncoder) {
      this.encoderSimState.setRawPosition(sensorRotations);
      this.encoderSimState.setVelocity(encoderRPS);
    }

    // Update the Mechanism Arm angle based on the simulated arm angle
    arm.setAngle(Units.radiansToDegrees(mechanismRadians));
    Logger.recordOutput(subsystemName + "/ArmSim", mech2d);
  }
}
