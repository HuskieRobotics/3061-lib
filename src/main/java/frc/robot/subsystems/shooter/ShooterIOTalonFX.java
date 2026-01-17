package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
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
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {

  // We usually use VelocityTorqueCurrentFOC to control the velocity of a wheel.
  private VelocityTorqueCurrentFOC leadVelocityRequest;
  private TorqueCurrentFOC leadCurrentRequest;

  // lead
  private StatusSignal<Current> leadStatorCurrentStatusSignal;
  private StatusSignal<Current> leadSupplyCurrentStatusSignal;
  private StatusSignal<AngularVelocity> leadVelocityStatusSignal;
  private StatusSignal<AngularVelocity> leadClosedLoopReferenceStatusSignal;
  private StatusSignal<Temperature> leadTemperatureStatusSignal;
  private StatusSignal<Voltage> leadVoltageStatusSignal;

  // follower
  private StatusSignal<Current> followerStatorCurrentStatusSignal;
  private StatusSignal<Current> followerSupplyCurrentStatusSignal;
  private StatusSignal<Temperature> followerTemperatureStatusSignal;
  private StatusSignal<Voltage> followerVoltageStatusSignal;

  private StatusSignal<Distance> gamePieceDistanceStatusSignal;
  private StatusSignal<Double> gamePieceSignalStrengthStatusSignal;
  private StatusSignal<Boolean> gamePieceDetectedStatusSignal;

  private AngularVelocity leadReferenceVelocity = RotationsPerSecond.of(0.0);

  private final Debouncer leadConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer gamePieceSensorConnectedDebouncer = new Debouncer(0.5);

  private VelocitySystemSim shooterSim;

  private Alert leadMotorConfigAlert =
      new Alert("Failed to apply configuration for lead shooter motor", AlertType.kError);
  private Alert followerConfigAlert =
      new Alert("Failed to apply configuration for follower shooter motor", AlertType.kError);
  private Alert gamePieceDetectorConfigAlert =
      new Alert("Failed to apply configuration for shooter game piece detector.", AlertType.kError);

  // The following enables tuning of the PID and feedforward values for the arm by changing values
  // via AdvantageScope and not needing to change values in code, compile, and re-deploy.
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", ShooterConstants.KP);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", ShooterConstants.KI);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", ShooterConstants.KD);
  private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", ShooterConstants.KS);
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
  private TalonFX followerMotor;
  private CANrange gamePieceDetector;

  public ShooterIOTalonFX() {
    leadMotor = new TalonFX(LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    followerMotor = new TalonFX(FOLLOWER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    gamePieceDetector = new CANrange(GAME_PIECE_SENSOR_ID, RobotConfig.getInstance().getCANBus());

    leadVelocityRequest = new VelocityTorqueCurrentFOC(0);

    leadCurrentRequest = new TorqueCurrentFOC(0);

    leadClosedLoopReferenceStatusSignal = leadMotor.getVelocity();
    leadVelocityStatusSignal = leadMotor.getVelocity();
    leadStatorCurrentStatusSignal = leadMotor.getStatorCurrent();
    followerStatorCurrentStatusSignal = followerMotor.getStatorCurrent();
    leadSupplyCurrentStatusSignal = leadMotor.getSupplyCurrent();
    followerSupplyCurrentStatusSignal = followerMotor.getSupplyCurrent();
    leadTemperatureStatusSignal = leadMotor.getDeviceTemp();
    followerTemperatureStatusSignal = followerMotor.getDeviceTemp();
    leadVoltageStatusSignal = leadMotor.getMotorVoltage();
    followerVoltageStatusSignal = followerMotor.getMotorVoltage();

    gamePieceDistanceStatusSignal = gamePieceDetector.getDistance();
    gamePieceSignalStrengthStatusSignal = gamePieceDetector.getSignalStrength();
    gamePieceDetectedStatusSignal = gamePieceDetector.getIsDetected();

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        true,
        leadClosedLoopReferenceStatusSignal,
        leadVelocityStatusSignal,
        leadStatorCurrentStatusSignal,
        followerStatorCurrentStatusSignal,
        leadSupplyCurrentStatusSignal,
        followerSupplyCurrentStatusSignal,
        leadTemperatureStatusSignal,
        followerTemperatureStatusSignal,
        leadVoltageStatusSignal,
        followerVoltageStatusSignal,
        gamePieceDistanceStatusSignal,
        gamePieceSignalStrengthStatusSignal,
        gamePieceDetectedStatusSignal);

    configLeadMotor(leadMotor, leadMotorConfigAlert);
    configFollowerMotor(followerMotor, followerConfigAlert);

    followerMotor.setControl(new Follower(FOLLOWER_MOTOR_ID, MotorAlignmentValue.Opposed));

    configGamePieceDetector(gamePieceDetector, gamePieceDetectorConfigAlert);

    //     // Create a simulation objects for the shooter. The specific parameters for the
    // simulation
    //     // are determined based on the mechanical design of the shooter.
    //     this.shootMotorBottomSim =
    //         new VelocitySystemSim(
    //             followerMotor,
    //             ShooterConstants.SHOOT_BOTTOM_INVERTED,
    //             0.05,
    //             0.01,
    //             ShooterConstants.SHOOT_MOTORS_GEAR_RATIO);
    //     this.shooterSim =
    //         new VelocitySystemSim(
    //             leadMotor,
    //             ShooterConstants.SHOOT_TOP_INVERTED,
    //             0.05,
    //             0.01,
    //             ShooterConstants.SHOOT_MOTORS_GEAR_RATIO);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Determine if the motors for the shooter are still connected (i.e., reachable on the CAN bus).
    // We do this by verifying that none of the status signals for the device report an error.
    inputs.leadConnected =
        leadConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                leadClosedLoopReferenceStatusSignal,
                leadStatorCurrentStatusSignal,
                leadSupplyCurrentStatusSignal,
                leadTemperatureStatusSignal,
                leadVoltageStatusSignal,
                leadVelocityStatusSignal));
    inputs.followerConnected =
        followerConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                followerStatorCurrentStatusSignal,
                followerSupplyCurrentStatusSignal,
                followerTemperatureStatusSignal,
                followerVoltageStatusSignal));
    inputs.sensorConnected =
        gamePieceSensorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                gamePieceDistanceStatusSignal,
                gamePieceSignalStrengthStatusSignal,
                gamePieceDetectedStatusSignal));

    // Updates Lead Motor Inputs
    inputs.leadMotorStatorCurrent = leadStatorCurrentStatusSignal.getValue();
    inputs.leadMotorSupplyCurrent = leadSupplyCurrentStatusSignal.getValue();
    inputs.leadMotorVelocity = leadClosedLoopReferenceStatusSignal.getValue();
    inputs.leadMotorTemp = leadTemperatureStatusSignal.getValue();
    inputs.leadMotorVoltage = leadVoltageStatusSignal.getValue();
    inputs.leadMotorReferenceVelocity = this.leadReferenceVelocity.copy();

    // Updates Follower Motor Inputs
    inputs.followerMotorStatorCurrent = followerStatorCurrentStatusSignal.getValue();
    inputs.followerMotorSupplyCurrent = followerSupplyCurrentStatusSignal.getValue();
    inputs.followerMotorTemp = followerTemperatureStatusSignal.getValue();
    inputs.followerMotorVoltage = followerVoltageStatusSignal.getValue();

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
      // this.shootMotorBottomSim.updateSim();
      this.shooterSim.updateSim();
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
  public void setShooterCurrent(Current amps) {
    leadMotor.setControl(leadCurrentRequest.withOutput(amps));
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

    // if (isTopMotor) {
    //   shootMotorsConfig.TorqueCurrent.PeakForwardTorqueCurrent =
    //       ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
    //   shootMotorsConfig.TorqueCurrent.PeakReverseTorqueCurrent =
    //       -ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
    // } else {
    //   shootMotorsConfig.TorqueCurrent.PeakForwardTorqueCurrent =
    //       ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
    //   shootMotorsConfig.TorqueCurrent.PeakReverseTorqueCurrent =
    //       -ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
    // }

    // if (isTopMotor) {
    //   shootMotorsConfig.Slot0.kP = kP.get();
    //   shootMotorsConfig.Slot0.kI = kI.get();
    //   shootMotorsConfig.Slot0.kD = kD.get();
    //   shootMotorsConfig.Slot0.kS = kS.get();
    // } else {
    //   shootMotorsConfig.Slot0.kP = shootMotorBottomKP.get();
    //   shootMotorsConfig.Slot0.kI = shootMotorBottomKI.get();
    //   shootMotorsConfig.Slot0.kD = shootMotorBottomKD.get();
    //   shootMotorsConfig.Slot0.kS = shootMotorBottomKS.get();
    // }

    leadMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOT_MOTORS_GEAR_RATIO;

    leadMotorConfig.MotorOutput.Inverted =
        IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
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

  private void configFollowerMotor(TalonFX motor, Alert configAlert) {

    TalonFXConfiguration followerMotorConfig = new TalonFXConfiguration();

    followerMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
        ShooterConstants.SHOOTER_PEAK_CURRENT_LIMIT;
    followerMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
        -ShooterConstants.SHOOTER_PEAK_CURRENT_LIMIT;

    Phoenix6Util.applyAndCheckConfiguration(motor, followerMotorConfig, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Shooter Follower Motor", motor);
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
