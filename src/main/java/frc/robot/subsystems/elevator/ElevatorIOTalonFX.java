package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ElevatorSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX elevatorMotorLead;
  private TalonFX elevatorMotorFollower;

  // We usually use MotionMagic Expo voltage to control the position of a mechanism.
  private DynamicMotionMagicExpoVoltage leadPositionRequest;
  private VoltageOut leadVoltageRequest;

  private Alert leadConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
  private Alert followerConfigAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private StatusSignal<Current> leadStatorCurrent;
  private StatusSignal<Current> followerStatorCurrent;

  private StatusSignal<Voltage> leadVoltageSupplied;
  private StatusSignal<Voltage> followerVoltageSupplied;

  private StatusSignal<Current> leadSupplyCurrent;
  private StatusSignal<Current> followerSupplyCurrent;

  private StatusSignal<Angle> elevatorPositionStatusSignal;

  private StatusSignal<Temperature> elevatorLeadTempStatusSignal;
  private StatusSignal<Temperature> elevatorFollowerTempStatusSignal;

  private StatusSignal<AngularVelocity> elevatorVelocityStatusSignal;

  private final Debouncer connectedLeadDebouncer = new Debouncer(0.5);
  private final Debouncer connectedFollowerDebouncer = new Debouncer(0.5);

  // The following enables tuning of the PID and feedforward values for the arm by changing values
  // via AdvantageScope and not needing to change values in code, compile, and re-deploy.
  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Elevator/kP", ElevatorConstants.KP_SLOT0);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Elevator/kI", ElevatorConstants.KI_SLOT0);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Elevator/kD", ElevatorConstants.KD_SLOT0);
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/kS", ElevatorConstants.KS_SLOT0);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/kV", ElevatorConstants.KV_SLOT0);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/kA", ElevatorConstants.KA_SLOT0);
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/kG", ElevatorConstants.KG_SLOT0);
  private final LoggedTunableNumber kVExpo =
      new LoggedTunableNumber("Elevator/kVExpo", ElevatorConstants.KV_EXPO);
  private final LoggedTunableNumber kAExpo =
      new LoggedTunableNumber("Elevator/kAExpo", ElevatorConstants.KA_EXPO);
  private final LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber("Elevator/Cruise Velocity", 0);

  private ElevatorSystemSim elevatorSystemSim;

  public ElevatorIOTalonFX() {

    elevatorMotorLead =
        new TalonFX(ElevatorConstants.LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    elevatorMotorFollower =
        new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID, RobotConfig.getInstance().getCANBus());

    leadStatorCurrent = elevatorMotorLead.getStatorCurrent();
    followerStatorCurrent = elevatorMotorFollower.getStatorCurrent();

    leadVoltageSupplied = elevatorMotorLead.getMotorVoltage();
    followerVoltageSupplied = elevatorMotorFollower.getMotorVoltage();

    leadSupplyCurrent = elevatorMotorLead.getSupplyCurrent();
    followerSupplyCurrent = elevatorMotorFollower.getSupplyCurrent();

    elevatorPositionStatusSignal = elevatorMotorLead.getPosition();

    elevatorLeadTempStatusSignal = elevatorMotorLead.getDeviceTemp();
    elevatorFollowerTempStatusSignal = elevatorMotorFollower.getDeviceTemp();

    elevatorVelocityStatusSignal = elevatorMotorLead.getVelocity();

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        true,
        leadStatorCurrent,
        followerStatorCurrent,
        leadVoltageSupplied,
        followerVoltageSupplied,
        leadSupplyCurrent,
        followerSupplyCurrent,
        elevatorPositionStatusSignal,
        elevatorLeadTempStatusSignal,
        elevatorFollowerTempStatusSignal,
        elevatorVelocityStatusSignal);

    leadPositionRequest = new DynamicMotionMagicExpoVoltage(0, kVExpo.get(), kAExpo.get());
    leadVoltageRequest = new VoltageOut(0);

    configElevatorMotorLead(elevatorMotorLead);
    configElevatorMotorFollower(elevatorMotorFollower);

    // Set the control for the follower motor to follow the lead motor. Whether the follower opposes
    // the direction of the lead depends on the mechanical design of the elevator.
    elevatorMotorFollower.setControl(
        new Follower(elevatorMotorLead.getDeviceID(), MotorAlignmentValue.Opposed));

    // Create a simulation object for the elevator. The specific parameters for the simulation
    // are determined based on the mechanical design of the elevator. The ElevatorSystemSim class
    // creates a Mechanism2d that can be visualized in AdvantageScope to test code in simulation
    // when the physical mechanism is not available.
    elevatorSystemSim =
        new ElevatorSystemSim(
            elevatorMotorLead,
            ElevatorConstants.IS_INVERTED,
            ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.ELEVATOR_MASS_KG,
            PULLEY_CIRCUMFERENCE.div(Math.PI * 2).in(Meters),
            ElevatorConstants.MIN_HEIGHT.in(Meters),
            ElevatorConstants.MAX_HEIGHT.in(Meters),
            0.0,
            ElevatorConstants.SUBSYSTEM_NAME);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Determine if the motors for the elevator are still connected (i.e., reachable on the CAN
    // bus). We do this by verifying that none of the status signals for the device report an error.
    inputs.connectedLead =
        connectedLeadDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                leadVoltageSupplied,
                leadStatorCurrent,
                leadSupplyCurrent,
                elevatorLeadTempStatusSignal,
                elevatorVelocityStatusSignal,
                elevatorPositionStatusSignal));
    inputs.connectedFollower =
        connectedFollowerDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                followerVoltageSupplied,
                followerStatorCurrent,
                followerSupplyCurrent,
                elevatorFollowerTempStatusSignal));

    inputs.voltageSuppliedLead = leadVoltageSupplied.getValue();
    inputs.voltageSuppliedFollower = followerVoltageSupplied.getValue();

    inputs.statorCurrentLead = leadStatorCurrent.getValue();
    inputs.statorCurrentFollower = followerStatorCurrent.getValue();

    inputs.supplyCurrentLead = leadSupplyCurrent.getValue();
    inputs.supplyCurrentFollower = followerSupplyCurrent.getValue();

    inputs.leadTemp = elevatorLeadTempStatusSignal.getValue();
    inputs.followerTemp = elevatorFollowerTempStatusSignal.getValue();

    inputs.velocity = elevatorVelocityStatusSignal.getValue();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode. To eliminate the performance hit, only retrieve the closed loop reference
    // signals if the tuning mode is enabled. It is critical that these input values are only used
    // for tuning and not used elsewhere in the subsystem.
    if (Constants.TUNING_MODE) {
      inputs.closedLoopError =
          Rotations.of(elevatorMotorLead.getClosedLoopError().getValueAsDouble());
      inputs.closedLoopReference =
          Rotations.of(elevatorMotorLead.getClosedLoopReference().getValueAsDouble());
    }

    inputs.angularPosition = elevatorPositionStatusSignal.getValue();
    inputs.linearPosition = PULLEY_CIRCUMFERENCE.times(inputs.angularPosition.in(Rotations));

    // In order for a tunable to be useful, there must be code that checks if its value has changed.
    // When a subsystem has multiple tunables that are related, the ifChanged method is a convenient
    // to check and apply changes from multiple tunables at once.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.elevatorMotorLead.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kG = motionMagic[6];

          config.MotionMagic.MotionMagicExpo_kV = motionMagic[7];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[8];

          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[9];

          this.elevatorMotorLead.getConfigurator().apply(config);
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG,
        kVExpo,
        kAExpo,
        cruiseVelocity);

    // The last step in the updateInputs method is to update the simulation.
    elevatorSystemSim.updateSim();
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    elevatorMotorLead.setControl(
        leadVoltageRequest.withLimitReverseMotion(false).withOutput(voltage));
  }

  @Override
  public void zeroPosition() {
    // Invoking the setPosition method results in a configuration call, which may take significant
    // time. As a workaround, we the withLimitReverseMotion decorator that is intended to be used
    // with external hardware limit switches to reset the position.
    elevatorMotorLead.setControl(leadVoltageRequest.withLimitReverseMotion(true).withOutput(0.0));
  }

  @Override
  public void setPosition(Distance position) {
    elevatorMotorLead.setControl(
        leadPositionRequest.withPosition(
            Rotations.of(position.div(PULLEY_CIRCUMFERENCE).magnitude())));
  }

  private void configElevatorMotorLead(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.MotorOutput.Inverted =
        IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kS = kS.get();
    config.Slot0.kV = kV.get();
    config.Slot0.kA = kA.get();
    config.Slot0.kG = kG.get();
    config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

    config.MotionMagic.MotionMagicExpo_kA = kAExpo.get();
    config.MotionMagic.MotionMagicExpo_kV = kVExpo.get();
    config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.get();

    // Invoking the setPosition method results in a configuration call, which may take significant
    // time. As a workaround, we will configure an external hardware limit switch. There is no
    // physical switch, but we will trigger it in a control call when we want to zero the elevator's
    // position.
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
    config.HardwareLimitSwitch.ReverseLimitEnable = true;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(elevatorMotorLead, config, leadConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance()
        .registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Lead", motor);
  }

  public void configElevatorMotorFollower(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // While the follower motor should be driven with the same voltage as the lead and draw the same
    // current, due to mechanical differences it may not. Therefore, we want to protect the
    // mechanism by configuring current limits on the follower as well.
    config.CurrentLimits.SupplyCurrentLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(elevatorMotorFollower, config, followerConfigAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance()
        .registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Follower", motor);
  }
}
