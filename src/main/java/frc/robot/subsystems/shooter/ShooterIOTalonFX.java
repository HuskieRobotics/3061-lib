package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  // We usually use VelocityTorqueCurrentFOC to control the velocity of a wheel.
  private VelocityTorqueCurrentFOC leadVelocityRequest;
  private TorqueCurrentFOC leadCurrentRequest;
  private VoltageOut hoodVoltageRequest;
  private PositionVoltage hoodPositionRequest;
  private VelocityTorqueCurrentFOC kickerVelocityRequest;
  private TorqueCurrentFOC kickerCurrentRequest;

  // lead
  private StatusSignal<Current> leadStatorCurrentStatusSignal;
  private StatusSignal<Current> leadSupplyCurrentStatusSignal;
  private StatusSignal<AngularVelocity> leadVelocityStatusSignal;
  private StatusSignal<Temperature> leadTemperatureStatusSignal;
  private StatusSignal<Voltage> leadVoltageStatusSignal;

  // follower A
  private StatusSignal<Current> followerAStatorCurrentStatusSignal;
  private StatusSignal<Current> followerASupplyCurrentStatusSignal;
  private StatusSignal<Temperature> followerATemperatureStatusSignal;
  private StatusSignal<Voltage> followerAVoltageStatusSignal;

  // follower B

  private StatusSignal<Current> followerBStatorCurrentStatusSignal;
  private StatusSignal<Current> followerBSupplyCurrentStatusSignal;
  private StatusSignal<Temperature> followerBTemperatureStatusSignal;
  private StatusSignal<Voltage> followerBVoltageStatusSignal;

  // Kicker
  private StatusSignal<Current> kickerStatorCurrentStatusSignal;
  private StatusSignal<Current> kickerSupplyCurrentStatusSignal;
  private StatusSignal<AngularVelocity> kickerVelocityStatusSignal;
  private StatusSignal<Temperature> kickerTemperatureStatusSignal;
  private StatusSignal<Voltage> kickerVoltageStatusSignal;

  // Hood Motor
  private StatusSignal<Current> hoodStatorCurrentStatusSignal;
  private StatusSignal<Current> hoodSupplyCurrentStatusSignal;
  private StatusSignal<Temperature> hoodTemperatureStatusSignal;
  private StatusSignal<Voltage> hoodVoltageStatusSignal;
  private StatusSignal<Angle> hoodPositionStatusSignal;

  private Angle hoodReferenceAngle = Degrees.of(0.0);

  // game piece detector
  private StatusSignal<Distance> gamePieceDistanceStatusSignal;
  private StatusSignal<Double> gamePieceSignalStrengthStatusSignal;
  private StatusSignal<Boolean> gamePieceDetectedStatusSignal;

  private AngularVelocity leadReferenceVelocity = RotationsPerSecond.of(0.0);
  private AngularVelocity kickerReferenceVelocity = RotationsPerSecond.of(0.0);

  private final Debouncer leadConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerAConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerBConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer hoodConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer kickerConnectedDebouncer = new Debouncer(0.5);

  private final Debouncer gamePieceSensorConnectedDebouncer = new Debouncer(0.5);

  private VelocitySystemSim shooterSim;
  private ArmSystemSim hoodSim;
  private VelocitySystemSim kickerSim;

  private Alert leadMotorConfigAlert =
      new Alert("Failed to apply configuration for lead shooter motor", AlertType.kError);
  private Alert followerAConfigAlert =
      new Alert("Failed to apply configuration for follower A shooter motor", AlertType.kError);
  private Alert followerBConfigAlert =
      new Alert("Failed to apply configuration for follower B shooter motor", AlertType.kError);
  private Alert hoodMotorConfigAlert =
      new Alert("Failed to apply configuration for hood motor", AlertType.kError);
  private Alert gamePieceDetectorConfigAlert =
      new Alert("Failed to apply configuration for shooter game piece detector.", AlertType.kError);
  private Alert kickerMotorConfigAlert =
      new Alert("Failed to apply configuration for kicker motor", AlertType.kError);

  // The following enables tuning of the PID and feedforward values for the arm by changing values
  // via AdvantageScope and not needing to change values in code, compile, and re-deploy.
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", ShooterConstants.KP);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", ShooterConstants.KI);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", ShooterConstants.KD);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", ShooterConstants.KS);

  private final LoggedTunableNumber kPHood =
      new LoggedTunableNumber("Shooter/Hood/kP", ShooterConstants.KP_HOOD);
  private final LoggedTunableNumber kIHood =
      new LoggedTunableNumber("Shooter/Hood/kI", ShooterConstants.KI_HOOD);
  private final LoggedTunableNumber kDHood =
      new LoggedTunableNumber("Shooter/Hood/kD", ShooterConstants.KD_HOOD);
  private final LoggedTunableNumber kSHood =
      new LoggedTunableNumber("Shooter/Hood/kS", ShooterConstants.KS_HOOD);
  private final LoggedTunableNumber kVHood =
      new LoggedTunableNumber("Shooter/Hood/kV", ShooterConstants.KV_HOOD);
  private final LoggedTunableNumber kAHood =
      new LoggedTunableNumber("Shooter/Hood/kA", ShooterConstants.KA_HOOD);
  private final LoggedTunableNumber kGHood =
      new LoggedTunableNumber("Shooter/Hood/kG", ShooterConstants.KG_HOOD);

  private final LoggedTunableNumber kickerKP =
      new LoggedTunableNumber("Shooter/Kicker/kP", ShooterConstants.KICKER_KP);
  private final LoggedTunableNumber kickerKI =
      new LoggedTunableNumber("Shooter/Kicker/kI", ShooterConstants.KICKER_KI);
  private final LoggedTunableNumber kickerKD =
      new LoggedTunableNumber("Shooter/Kicker/kD", ShooterConstants.KICKER_KD);
  private final LoggedTunableNumber kickerKS =
      new LoggedTunableNumber("Shooter/Kicker/kS", ShooterConstants.KICKER_KS);
  private final LoggedTunableNumber kickerKV =
      new LoggedTunableNumber("Shooter/Kicker/kV", ShooterConstants.KICKER_KV);
  private final LoggedTunableNumber kickerKA =
      new LoggedTunableNumber("Shooter/Kicker/kA", ShooterConstants.KICKER_KA);

  private final LoggedTunableNumber detectorMinSignalStrength =
      new LoggedTunableNumber(
          "Shooter/Min Signal Strength", ShooterConstants.DETECTOR_MIN_SIGNAL_STRENGTH);
  private final LoggedTunableNumber detectorProximityThreshold =
      new LoggedTunableNumber(
          "Shooter/Proximity Threshold", ShooterConstants.DETECTOR_PROXIMITY_THRESHOLD);

  // It is a bit more challenging to simulate a CANrange sensor compared to a DIO sensor. Using a
  // Tunable to simulate the distance to a game piece, requires that TUNING is set to true.
  private final LoggedTunableNumber simDetectorDistance =
      new LoggedTunableNumber("Shooter/Sim Detector Distance (m)", 1.0);

  private TalonFX leadMotor;
  private TalonFX followerAMotor;
  private TalonFX followerBMotor;
  private TalonFX hoodMotor;
  private TalonFX kickerMotor;
  private CANrange gamePieceDetector;

  public ShooterIOTalonFX() {
    leadMotor = new TalonFX(LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    followerAMotor = new TalonFX(FOLLOWER_A_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    followerBMotor = new TalonFX(FOLLOWER_B_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    hoodMotor = new TalonFX(HOOD_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    gamePieceDetector = new CANrange(GAME_PIECE_SENSOR_ID, RobotConfig.getInstance().getCANBus());
    kickerMotor = new TalonFX(KICKER_MOTOR_ID, RobotConfig.getInstance().getCANBus());

    leadVelocityRequest = new VelocityTorqueCurrentFOC(0);
    leadCurrentRequest = new TorqueCurrentFOC(0);
    hoodVoltageRequest = new VoltageOut(0);
    hoodPositionRequest = new PositionVoltage(0);
    kickerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    kickerCurrentRequest = new TorqueCurrentFOC(0);

    leadVelocityStatusSignal = leadMotor.getVelocity();
    leadStatorCurrentStatusSignal = leadMotor.getStatorCurrent();
    leadSupplyCurrentStatusSignal = leadMotor.getSupplyCurrent();
    leadTemperatureStatusSignal = leadMotor.getDeviceTemp();
    leadVoltageStatusSignal = leadMotor.getMotorVoltage();

    followerAStatorCurrentStatusSignal = followerAMotor.getStatorCurrent();
    followerASupplyCurrentStatusSignal = followerAMotor.getSupplyCurrent();
    followerATemperatureStatusSignal = followerAMotor.getDeviceTemp();
    followerAVoltageStatusSignal = followerAMotor.getMotorVoltage();

    followerBStatorCurrentStatusSignal = followerBMotor.getStatorCurrent();
    followerBSupplyCurrentStatusSignal = followerBMotor.getSupplyCurrent();
    followerBTemperatureStatusSignal = followerBMotor.getDeviceTemp();
    followerBVoltageStatusSignal = followerBMotor.getMotorVoltage();

    hoodStatorCurrentStatusSignal = hoodMotor.getStatorCurrent();
    hoodSupplyCurrentStatusSignal = hoodMotor.getSupplyCurrent();
    hoodTemperatureStatusSignal = hoodMotor.getDeviceTemp();
    hoodVoltageStatusSignal = hoodMotor.getMotorVoltage();
    hoodPositionStatusSignal = hoodMotor.getPosition();

    kickerVelocityStatusSignal = kickerMotor.getVelocity();
    kickerStatorCurrentStatusSignal = kickerMotor.getStatorCurrent();
    kickerSupplyCurrentStatusSignal = kickerMotor.getSupplyCurrent();
    kickerTemperatureStatusSignal = kickerMotor.getDeviceTemp();
    kickerVoltageStatusSignal = kickerMotor.getMotorVoltage();

    gamePieceDistanceStatusSignal = gamePieceDetector.getDistance();
    gamePieceSignalStrengthStatusSignal = gamePieceDetector.getSignalStrength();
    gamePieceDetectedStatusSignal = gamePieceDetector.getIsDetected();

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        true,
        leadVelocityStatusSignal,
        leadStatorCurrentStatusSignal,
        leadSupplyCurrentStatusSignal,
        leadTemperatureStatusSignal,
        leadVoltageStatusSignal,
        followerAStatorCurrentStatusSignal,
        followerASupplyCurrentStatusSignal,
        followerATemperatureStatusSignal,
        followerAVoltageStatusSignal,
        followerBStatorCurrentStatusSignal,
        followerBSupplyCurrentStatusSignal,
        followerBTemperatureStatusSignal,
        followerBVoltageStatusSignal,
        hoodStatorCurrentStatusSignal,
        hoodSupplyCurrentStatusSignal,
        hoodTemperatureStatusSignal,
        hoodVoltageStatusSignal,
        hoodPositionStatusSignal,
        kickerVelocityStatusSignal,
        kickerStatorCurrentStatusSignal,
        kickerSupplyCurrentStatusSignal,
        kickerTemperatureStatusSignal,
        kickerVoltageStatusSignal,
        gamePieceDistanceStatusSignal,
        gamePieceSignalStrengthStatusSignal,
        gamePieceDetectedStatusSignal);

    configLeadMotor(leadMotor, leadMotorConfigAlert);
    configFollowerMotor(followerAMotor, "Follower A Motor", followerAConfigAlert);
    configFollowerMotor(followerBMotor, "Follower B Motor", followerBConfigAlert);
    configHoodMotor(hoodMotor, "Hood Motor", hoodMotorConfigAlert);
    configKickerMotor(kickerMotor, kickerMotorConfigAlert);

    followerAMotor.setControl(
        new Follower(
            FOLLOWER_A_MOTOR_ID,
            IS_FOLLOWER_A_INVERTED ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    followerBMotor.setControl(
        new Follower(
            FOLLOWER_B_MOTOR_ID,
            IS_FOLLOWER_B_INVERTED ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    configGamePieceDetector(gamePieceDetector, gamePieceDetectorConfigAlert);

    // Create a simulation objects for the shooter. The specific parameters for the simulation
    // are determined based on the mechanical design of the shooter.
    this.shooterSim =
        new VelocitySystemSim(
            ShooterConstants.IS_LEAD_INVERTED,
            0.05,
            0.01,
            ShooterConstants.SHOOT_MOTORS_GEAR_RATIO,
            leadMotor,
            followerAMotor);

    this.hoodSim =
        new ArmSystemSim(
            hoodMotor,
            IS_HOOD_INVERTED,
            HOOD_GEAR_RATIO,
            HOOD_LENGTH_METERS,
            HOOD_MASS_KG,
            HOOD_STARTING_ANGLE.in(Radians),
            HOOD_MAX_ANGLE.in(Radians),
            HOOD_STARTING_ANGLE.in(Radians),
            ShooterConstants.SUBSYSTEM_NAME + " Hood");

    this.kickerSim =
        new VelocitySystemSim(
            ShooterConstants.KICKER_MOTOR_INVERTED,
            0.05,
            0.01,
            ShooterConstants.KICKER_GEAR_RATIO,
            kickerMotor,
            null);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Determine if the motors for the shooter are still connected (i.e., reachable on the CAN bus).
    // We do this by verifying that none of the status signals for the device report an error.
    inputs.leadConnected =
        leadConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                leadStatorCurrentStatusSignal,
                leadSupplyCurrentStatusSignal,
                leadTemperatureStatusSignal,
                leadVoltageStatusSignal,
                leadVelocityStatusSignal));
    inputs.followerAConnected =
        followerAConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                followerAStatorCurrentStatusSignal,
                followerASupplyCurrentStatusSignal,
                followerATemperatureStatusSignal,
                followerAVoltageStatusSignal));
    inputs.followerBConnected =
        followerBConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                followerBStatorCurrentStatusSignal,
                followerBSupplyCurrentStatusSignal,
                followerBTemperatureStatusSignal,
                followerBVoltageStatusSignal));
    inputs.hoodConnected =
        hoodConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                hoodStatorCurrentStatusSignal,
                hoodSupplyCurrentStatusSignal,
                hoodTemperatureStatusSignal,
                hoodVoltageStatusSignal,
                hoodPositionStatusSignal));
    inputs.kickerConnected =
        kickerConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                kickerStatorCurrentStatusSignal,
                kickerSupplyCurrentStatusSignal,
                kickerTemperatureStatusSignal,
                kickerVoltageStatusSignal,
                kickerVelocityStatusSignal));
    inputs.sensorConnected =
        gamePieceSensorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                gamePieceDistanceStatusSignal,
                gamePieceSignalStrengthStatusSignal,
                gamePieceDetectedStatusSignal));

    // Updates Lead Motor Inputs
    inputs.leadMotorStatorCurrent = leadStatorCurrentStatusSignal.getValue();
    inputs.leadMotorSupplyCurrent = leadSupplyCurrentStatusSignal.getValue();
    inputs.leadMotorVelocity = leadVelocityStatusSignal.getValue();
    inputs.leadMotorTemp = leadTemperatureStatusSignal.getValue();
    inputs.leadMotorVoltage = leadVoltageStatusSignal.getValue();
    inputs.leadMotorReferenceVelocity = this.leadReferenceVelocity.copy();

    // Updates Kicker Motor Inputs
    inputs.kickerMotorStatorCurrent = kickerStatorCurrentStatusSignal.getValue();
    inputs.kickerMotorSupplyCurrent = kickerSupplyCurrentStatusSignal.getValue();
    inputs.kickerMotorVelocity = kickerVelocityStatusSignal.getValue();
    inputs.kickerMotorTemp = kickerTemperatureStatusSignal.getValue();
    inputs.kickerMotorVoltage = kickerVoltageStatusSignal.getValue();
    inputs.kickerMotorReferenceVelocity = this.kickerReferenceVelocity.copy();

    // Updates Follower Motor Inputs
    inputs.followerAMotorStatorCurrent = followerAStatorCurrentStatusSignal.getValue();
    inputs.followerAMotorSupplyCurrent = followerASupplyCurrentStatusSignal.getValue();
    inputs.followerAMotorTemp = followerATemperatureStatusSignal.getValue();
    inputs.followerAMotorVoltage = followerAVoltageStatusSignal.getValue();

    inputs.followerBMotorStatorCurrent = followerBStatorCurrentStatusSignal.getValue();
    inputs.followerBMotorSupplyCurrent = followerBSupplyCurrentStatusSignal.getValue();
    inputs.followerBMotorTemp = followerBTemperatureStatusSignal.getValue();
    inputs.followerBMotorVoltage = followerBVoltageStatusSignal.getValue();

    // Updates Hood Motor Inputs
    inputs.hoodMotorStatorCurrent = hoodStatorCurrentStatusSignal.getValue();
    inputs.hoodMotorSupplyCurrent = hoodSupplyCurrentStatusSignal.getValue();
    inputs.hoodMotorTemp = hoodTemperatureStatusSignal.getValue();
    inputs.hoodMotorVoltage = hoodVoltageStatusSignal.getValue();
    inputs.hoodPosition = hoodPositionStatusSignal.getValue();

    // Update Game Piece Detection Inputs
    inputs.hasGamePiece = gamePieceDetectedStatusSignal.getValue();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode. To eliminate the performance hit, only retrieve the closed loop reference
    // signals if the tuning mode is enabled. It is critical that these input values are only used
    // for tuning and not used elsewhere in the subsystem. For example, the
    // shootMotorTopReferenceVelocityRPS property should be used throughout the subsystem since it
    // will always be populated.
    if (Constants.TUNING_MODE) {
      inputs.leadMotorClosedLoopReferenceVelocity =
          RotationsPerSecond.of(leadMotor.getClosedLoopReference().getValue());
      inputs.leadMotorClosedLoopErrorVelocity =
          RotationsPerSecond.of(leadMotor.getClosedLoopError().getValue());

      inputs.kickerMotorClosedLoopReferenceVelocity =
          RotationsPerSecond.of(kickerMotor.getClosedLoopReference().getValue());
      inputs.kickerMotorClosedLoopErrorVelocity =
          RotationsPerSecond.of(kickerMotor.getClosedLoopError().getValue());

      inputs.distanceToGamePiece = gamePieceDistanceStatusSignal.getValue();
      inputs.signalStrength = gamePieceSignalStrengthStatusSignal.getValue();
    }

    // In order for a tunable to be useful, there must be code that checks if its value has changed.
    // When a subsystem has multiple tunables that are related, the ifChanged method is a convenient
    // to check and apply changes from multiple tunables at once.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.leadMotor.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];

          this.leadMotor.getConfigurator().apply(config);
        },
        kP,
        kI,
        kD,
        kS);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.kickerMotor.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];
          config.kV = pid[4];
          config.kA = pid[5];

          this.kickerMotor.getConfigurator().apply(config);
        },
        kickerKP,
        kickerKI,
        kickerKD,
        kickerKS,
        kickerKV,
        kickerKA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.hoodMotor.getConfigurator().refresh(config);
          config.Slot0.kP = pid[0];
          config.Slot0.kI = pid[1];
          config.Slot0.kD = pid[2];
          config.Slot0.kS = pid[3];
          config.Slot0.kV = pid[4];
          config.Slot0.kA = pid[5];
          config.Slot0.kG = pid[6];

          this.hoodMotor.getConfigurator().apply(config);
        },
        kPHood,
        kIHood,
        kDHood,
        kSHood,
        kVHood,
        kAHood,
        kGHood);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        detectorConfig -> {
          ProximityParamsConfigs config = new ProximityParamsConfigs();
          this.gamePieceDetector.getConfigurator().refresh(config);
          config.MinSignalStrengthForValidMeasurement = detectorConfig[0];
          config.ProximityThreshold = detectorConfig[1];

          this.gamePieceDetector.getConfigurator().apply(config);
        },
        detectorMinSignalStrength,
        detectorProximityThreshold);

    // The last step in the updateInputs method is to update the simulation.
    if (Constants.getMode() == Constants.Mode.SIM) {
      this.shooterSim.updateSim();
      this.hoodSim.updateSim();
      this.kickerSim.updateSim();
      this.gamePieceDetector.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
      this.gamePieceDetector.getSimState().setDistance(simDetectorDistance.get());
    }
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    leadMotor.setControl(leadVelocityRequest.withVelocity(velocity));

    // To improve performance, we store the reference velocity as an instance variable to avoid
    // having to retrieve the status signal object from the device in the updateInputs method.
    this.leadReferenceVelocity = velocity.copy();
  }

  @Override
  public void setHoodAngle(Angle angle) {

    hoodMotor.setControl(hoodPositionRequest.withPosition(angle.in(Rotations)));
    this.hoodReferenceAngle = angle.copy();
  }

  @Override
  public void setHoodVoltage(Voltage voltage) {
    hoodMotor.setControl(hoodVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setShooterCurrent(Current amps) {
    leadMotor.setControl(leadCurrentRequest.withOutput(amps));
  }

  @Override
  public void setKickerVelocity(AngularVelocity velocity) {
    kickerMotor.setControl(kickerVelocityRequest.withVelocity(velocity));

    // To improve performance, we store the reference velocity as an instance variable to avoid
    // having to retrieve the status signal object from the device in the updateInputs method.
    this.kickerReferenceVelocity = velocity.copy();
  }

  @Override
  public void setKickerCurrent(Current amps) {
    kickerMotor.setControl(kickerCurrentRequest.withOutput(amps));
  }

  private void configLeadMotor(TalonFX motor, Alert configAlert) {

    TalonFXConfiguration leadMotorConfig = new TalonFXConfiguration();

    leadMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.SHOOTER_PEAK_CURRENT_LIMIT;
    leadMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.SHOOTER_PEAK_CURRENT_LIMIT;

    leadMotorConfig.Slot0.kP = kP.get();
    leadMotorConfig.Slot0.kI = kI.get();
    leadMotorConfig.Slot0.kD = kD.get();
    leadMotorConfig.Slot0.kS = kS.get();
    leadMotorConfig.Slot0.kV = KV;
    leadMotorConfig.Slot0.kA = KA;

    leadMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOT_MOTORS_GEAR_RATIO;

    leadMotorConfig.MotorOutput.Inverted =
        IS_LEAD_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leadMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(motor, leadMotorConfig, configAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Shooter Lead Motor", motor);
  }

  private void configFollowerMotor(TalonFX motor, String deviceDescription, Alert configAlert) {

    TalonFXConfiguration followerMotorConfig = new TalonFXConfiguration();

    followerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.SHOOTER_PEAK_CURRENT_LIMIT;
    followerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.SHOOTER_PEAK_CURRENT_LIMIT;

    followerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    Phoenix6Util.applyAndCheckConfiguration(motor, followerMotorConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, deviceDescription, motor);
  }

  public void configHoodMotor(TalonFX motor, String deviceDescription, Alert configAlert) {
    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();

    angleMotorConfig.CurrentLimits.SupplyCurrentLimit =
        ShooterConstants.HOOD_MOTOR_PEAK_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
        ShooterConstants.HOOD_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
        ShooterConstants.HOOD_MOTOR_PEAK_CURRENT_DURATION;
    angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    angleMotorConfig.CurrentLimits.StatorCurrentLimit =
        ShooterConstants.HOOD_MOTOR_PEAK_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleMotorConfig.Slot0.kP = kPHood.get();
    angleMotorConfig.Slot0.kI = kIHood.get();
    angleMotorConfig.Slot0.kD = kDHood.get();
    angleMotorConfig.Slot0.kS = kSHood.get();
    angleMotorConfig.Slot0.kG = kGHood.get();
    angleMotorConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
    angleMotorConfig.Slot0.kA = kAHood.get();
    angleMotorConfig.Slot0.kV = kVHood.get();

    angleMotorConfig.MotorOutput.Inverted =
        ShooterConstants.IS_HOOD_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Software limit switches are used to prevent the arm from moving beyond its physical limits.
    SoftwareLimitSwitchConfigs angleMotorLimitSwitches = angleMotorConfig.SoftwareLimitSwitch;
    angleMotorLimitSwitches.ForwardSoftLimitEnable = true;
    angleMotorLimitSwitches.ForwardSoftLimitThreshold =
        ShooterConstants.UPPER_ANGLE_LIMIT.in(Rotations);
    angleMotorLimitSwitches.ReverseSoftLimitEnable = true;
    angleMotorLimitSwitches.ReverseSoftLimitThreshold =
        ShooterConstants.LOWER_ANGLE_LIMIT.in(Rotations);

    angleMotorConfig.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;

    Phoenix6Util.applyAndCheckConfiguration(motor, angleMotorConfig, configAlert);

    motor.setPosition(ShooterConstants.HOOD_STARTING_ANGLE.in(Rotations));
  }

  private void configKickerMotor(TalonFX motor, Alert configAlert) {

    TalonFXConfiguration kickerMotorConfig = new TalonFXConfiguration();

    kickerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.KICKER_PEAK_CURRENT_LIMIT;
    kickerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.KICKER_PEAK_CURRENT_LIMIT;

    kickerMotorConfig.Slot0.kP = kP.get();
    kickerMotorConfig.Slot0.kI = kI.get();
    kickerMotorConfig.Slot0.kD = kD.get();
    kickerMotorConfig.Slot0.kS = kS.get();
    kickerMotorConfig.Slot0.kV = KV;
    kickerMotorConfig.Slot0.kA = KA;
    kickerMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOT_MOTORS_GEAR_RATIO;

    kickerMotorConfig.MotorOutput.Inverted =
        IS_LEAD_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    kickerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(motor, kickerMotorConfig, configAlert);
    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Shooter Kicker Motor", motor);
  }

  private void configGamePieceDetector(CANrange detector, Alert configAlert) {
    CANrangeConfiguration config = new CANrangeConfiguration();

    // if CANrange has a signal strength of at least 2000, it is a valid measurement
    config.ProximityParams.MinSignalStrengthForValidMeasurement = detectorMinSignalStrength.get();

    // if CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal
    config.ProximityParams.ProximityThreshold = detectorProximityThreshold.get();

    // make the CANrange update as fast as possible at 100 Hz. This requires short-range mode
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(detector, config, configAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "GamePieceDetector", detector);
  }
}
