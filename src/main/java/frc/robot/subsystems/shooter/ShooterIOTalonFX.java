package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
  private VelocityTorqueCurrentFOC shootMotorTopVelocityRequest;
  private VelocityTorqueCurrentFOC shootMotorBottomVelocityRequest;
  private TorqueCurrentFOC shootMotorTopCurrentRequest;
  private TorqueCurrentFOC shootMotorBottomCurrentRequest;

  private StatusSignal<Current> shootMotorTopStatorCurrentStatusSignal;
  private StatusSignal<Current> shootMotorBottomStatorCurrentStatusSignal;
  private StatusSignal<Current> shootMotorTopSupplyCurrentStatusSignal;
  private StatusSignal<Current> shootMotorBottomSupplyCurrentStatusSignal;
  private StatusSignal<AngularVelocity> shootMotorTopVelocityStatusSignal;
  private StatusSignal<AngularVelocity> shootMotorBottomVelocityStatusSignal;
  private StatusSignal<Temperature> shootMotorTopTemperatureStatusSignal;
  private StatusSignal<Temperature> shootMotorBottomTemperatureStatusSignal;
  private StatusSignal<Voltage> shootMotorTopVoltageStatusSignal;
  private StatusSignal<Voltage> shootMotorBottomVoltageStatusSignal;
  private StatusSignal<Distance> gamePieceDistanceStatusSignal;
  private StatusSignal<Double> gamePieceSignalStrengthStatusSignal;
  private StatusSignal<Boolean> gamePieceDetectedStatusSignal;

  private AngularVelocity shootTopMotorReferenceVelocity = RotationsPerSecond.of(0.0);
  private AngularVelocity shootBottomMotorReferenceVelocity = RotationsPerSecond.of(0.0);

  private final Debouncer topMotorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer bottomMotorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer gamePieceSensorConnectedDebouncer = new Debouncer(0.5);

  private VelocitySystemSim shootMotorTopSim;
  private VelocitySystemSim shootMotorBottomSim;

  private Alert topMotorConfigAlert =
      new Alert("Failed to apply configuration for shooter top motor.", AlertType.kError);
  private Alert bottomMotorConfigAlert =
      new Alert("Failed to apply configuration for shooter bottom motor.", AlertType.kError);
  private Alert gamePieceDetectorConfigAlert =
      new Alert("Failed to apply configuration for shooter game piece detector.", AlertType.kError);

  // The following enables tuning of the PID and feedforward values for the arm by changing values
  // via AdvantageScope and not needing to change values in code, compile, and re-deploy.
  private final LoggedTunableNumber shootMotorTopKP =
      new LoggedTunableNumber("Shooter/Top kP", ShooterConstants.TOP_SHOOT_KP);
  private final LoggedTunableNumber shootMotorTopKI =
      new LoggedTunableNumber("Shooter/Top kI", ShooterConstants.TOP_SHOOT_KI);
  private final LoggedTunableNumber shootMotorTopKD =
      new LoggedTunableNumber("Shooter/Top kD", ShooterConstants.TOP_SHOOT_KD);
  private final LoggedTunableNumber shootMotorTopKS =
      new LoggedTunableNumber("Shooter/Top kS", ShooterConstants.TOP_SHOOT_KS);
  private final LoggedTunableNumber shootMotorBottomKP =
      new LoggedTunableNumber("Shooter/Bottom kP", ShooterConstants.BOTTOM_SHOOT_KP);
  private final LoggedTunableNumber shootMotorBottomKI =
      new LoggedTunableNumber("Shooter/Bottom kI", ShooterConstants.BOTTOM_SHOOT_KI);
  private final LoggedTunableNumber shootMotorBottomKD =
      new LoggedTunableNumber("Shooter/Bottom kD", ShooterConstants.BOTTOM_SHOOT_KD);
  private final LoggedTunableNumber shootMotorBottomKS =
      new LoggedTunableNumber("Shooter/Bottom kS", ShooterConstants.BOTTOM_SHOOT_KS);
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

  private TalonFX shootMotorTop;
  private TalonFX shootMotorBottom;
  private CANrange gamePieceDetector;

  public ShooterIOTalonFX() {
    shootMotorTop = new TalonFX(TOP_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    shootMotorBottom = new TalonFX(BOTTOM_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBus());
    gamePieceDetector = new CANrange(GAME_PIECE_SENSOR_ID, RobotConfig.getInstance().getCANBus());

    shootMotorTopVelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotorBottomVelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotorTopCurrentRequest = new TorqueCurrentFOC(0);
    shootMotorBottomCurrentRequest = new TorqueCurrentFOC(0);

    shootMotorTopVelocityStatusSignal = shootMotorTop.getVelocity();
    shootMotorBottomVelocityStatusSignal = shootMotorBottom.getVelocity();
    shootMotorTopStatorCurrentStatusSignal = shootMotorTop.getStatorCurrent();
    shootMotorBottomStatorCurrentStatusSignal = shootMotorBottom.getStatorCurrent();
    shootMotorTopSupplyCurrentStatusSignal = shootMotorTop.getSupplyCurrent();
    shootMotorBottomSupplyCurrentStatusSignal = shootMotorBottom.getSupplyCurrent();
    shootMotorTopTemperatureStatusSignal = shootMotorTop.getDeviceTemp();
    shootMotorBottomTemperatureStatusSignal = shootMotorBottom.getDeviceTemp();
    shootMotorTopVoltageStatusSignal = shootMotorTop.getMotorVoltage();
    shootMotorBottomVoltageStatusSignal = shootMotorBottom.getMotorVoltage();

    gamePieceDistanceStatusSignal = gamePieceDetector.getDistance();
    gamePieceSignalStrengthStatusSignal = gamePieceDetector.getSignalStrength();
    gamePieceDetectedStatusSignal = gamePieceDetector.getIsDetected();

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        true,
        shootMotorTopVelocityStatusSignal,
        shootMotorBottomVelocityStatusSignal,
        shootMotorTopStatorCurrentStatusSignal,
        shootMotorBottomStatorCurrentStatusSignal,
        shootMotorTopSupplyCurrentStatusSignal,
        shootMotorBottomSupplyCurrentStatusSignal,
        shootMotorTopTemperatureStatusSignal,
        shootMotorBottomTemperatureStatusSignal,
        shootMotorTopVoltageStatusSignal,
        shootMotorBottomVoltageStatusSignal,
        gamePieceDistanceStatusSignal,
        gamePieceSignalStrengthStatusSignal,
        gamePieceDetectedStatusSignal);

    configShootMotor(shootMotorTop, SHOOT_TOP_INVERTED, true, topMotorConfigAlert);
    configShootMotor(shootMotorBottom, SHOOT_BOTTOM_INVERTED, false, bottomMotorConfigAlert);
    configGamePieceDetector(gamePieceDetector, gamePieceDetectorConfigAlert);

    // Create a simulation objects for the shooter. The specific parameters for the simulation
    // are determined based on the mechanical design of the shooter.
    this.shootMotorBottomSim =
        new VelocitySystemSim(
            shootMotorBottom,
            ShooterConstants.SHOOT_BOTTOM_INVERTED,
            0.05,
            0.01,
            ShooterConstants.SHOOT_MOTORS_GEAR_RATIO);
    this.shootMotorTopSim =
        new VelocitySystemSim(
            shootMotorTop,
            ShooterConstants.SHOOT_TOP_INVERTED,
            0.05,
            0.01,
            ShooterConstants.SHOOT_MOTORS_GEAR_RATIO);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Determine if the motors for the shooter are still connected (i.e., reachable on the CAN bus).
    // We do this by verifying that none of the status signals for the device report an error.
    inputs.shootMotorTopConnected =
        topMotorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                shootMotorTopVelocityStatusSignal,
                shootMotorTopStatorCurrentStatusSignal,
                shootMotorTopSupplyCurrentStatusSignal,
                shootMotorTopTemperatureStatusSignal,
                shootMotorTopVoltageStatusSignal));
    inputs.shootMotorBottomConnected =
        bottomMotorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                shootMotorBottomVelocityStatusSignal,
                shootMotorBottomStatorCurrentStatusSignal,
                shootMotorBottomSupplyCurrentStatusSignal,
                shootMotorBottomTemperatureStatusSignal,
                shootMotorBottomVoltageStatusSignal));
    inputs.sensorConnected =
        gamePieceSensorConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                gamePieceDistanceStatusSignal,
                gamePieceSignalStrengthStatusSignal,
                gamePieceDetectedStatusSignal));

    // Updates Top Shooter Motor Inputs
    inputs.shootMotorTopStatorCurrent = shootMotorTopStatorCurrentStatusSignal.getValue();
    inputs.shootMotorTopSupplyCurrent = shootMotorTopSupplyCurrentStatusSignal.getValue();
    inputs.shootMotorTopVelocity = shootMotorTopVelocityStatusSignal.getValue();
    inputs.shootMotorTopTemperature = shootMotorTopTemperatureStatusSignal.getValue();
    inputs.shootMotorTopVoltage = shootMotorTopVoltageStatusSignal.getValue();
    inputs.shootMotorTopReferenceVelocity = this.shootTopMotorReferenceVelocity.copy();

    // Updates Bottom Shooter Motor Inputs
    inputs.shootMotorBottomStatorCurrent = shootMotorBottomStatorCurrentStatusSignal.getValue();
    inputs.shootMotorBottomSupplyCurrent = shootMotorBottomSupplyCurrentStatusSignal.getValue();
    inputs.shootMotorBottomVelocity = shootMotorBottomVelocityStatusSignal.getValue();
    inputs.shootMotorBottomTemperature = shootMotorBottomTemperatureStatusSignal.getValue();
    inputs.shootMotorBottomVoltage = shootMotorBottomVoltageStatusSignal.getValue();
    inputs.shootMotorBottomReferenceVelocity = this.shootBottomMotorReferenceVelocity.copy();

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
      inputs.shootMotorTopClosedLoopReferenceVelocity =
          RotationsPerSecond.of(shootMotorTop.getClosedLoopReference().getValue());
      inputs.shootMotorTopClosedLoopErrorVelocity =
          RotationsPerSecond.of(shootMotorTop.getClosedLoopError().getValue());
      inputs.shootMotorBottomClosedLoopReferenceVelocity =
          RotationsPerSecond.of(shootMotorBottom.getClosedLoopReference().getValue());
      inputs.shootMotorBottomClosedLoopErrorVelocity =
          RotationsPerSecond.of(shootMotorBottom.getClosedLoopError().getValue());
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
          this.shootMotorTop.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];

          this.shootMotorTop.getConfigurator().apply(config);
        },
        shootMotorTopKP,
        shootMotorTopKI,
        shootMotorTopKD,
        shootMotorTopKS);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          Slot0Configs config = new Slot0Configs();
          this.shootMotorBottom.getConfigurator().refresh(config);
          config.kP = pid[0];
          config.kI = pid[1];
          config.kD = pid[2];
          config.kS = pid[3];

          this.shootMotorBottom.getConfigurator().apply(config);
        },
        shootMotorBottomKP,
        shootMotorBottomKI,
        shootMotorBottomKD,
        shootMotorBottomKS);
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
      this.shootMotorBottomSim.updateSim();
      this.shootMotorTopSim.updateSim();
      this.gamePieceDetector.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
      this.gamePieceDetector.getSimState().setDistance(simDetectorDistance.get());
    }
  }

  @Override
  public void setShooterWheelTopVelocity(AngularVelocity velocity) {
    shootMotorTop.setControl(shootMotorTopVelocityRequest.withVelocity(velocity));

    // To improve performance, we store the reference velocity as an instance variable to avoid
    // having to retrieve the status signal object from the device in the updateInputs method.
    this.shootTopMotorReferenceVelocity = velocity.copy();
  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., AngularVelocity) throughout the rest of the code to mitigate bugs due
  // to unit mismatches.
  @Override
  public void setShooterWheelBottomVelocity(AngularVelocity velocity) {
    shootMotorBottom.setControl(shootMotorBottomVelocityRequest.withVelocity(velocity));

    // To improve performance, we store the reference velocity as an instance variable to avoid
    // having to retrieve the status signal object from the device in the updateInputs method.
    this.shootBottomMotorReferenceVelocity = velocity.copy();
  }

  @Override
  public void setShooterWheelTopCurrent(Current amps) {
    shootMotorTop.setControl(shootMotorTopCurrentRequest.withOutput(amps));
  }

  @Override
  public void setShooterWheelBottomCurrent(Current amps) {
    shootMotorBottom.setControl(shootMotorBottomCurrentRequest.withOutput(amps));
  }

  private void configShootMotor(
      TalonFX shootMotor, boolean isInverted, boolean isTopMotor, Alert configAlert) {

    TalonFXConfiguration shootMotorsConfig = new TalonFXConfiguration();

    if (isTopMotor) {
      shootMotorsConfig.TorqueCurrent.PeakForwardTorqueCurrent =
          ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
      shootMotorsConfig.TorqueCurrent.PeakReverseTorqueCurrent =
          -ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
    } else {
      shootMotorsConfig.TorqueCurrent.PeakForwardTorqueCurrent =
          ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
      shootMotorsConfig.TorqueCurrent.PeakReverseTorqueCurrent =
          -ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
    }

    if (isTopMotor) {
      shootMotorsConfig.Slot0.kP = shootMotorTopKP.get();
      shootMotorsConfig.Slot0.kI = shootMotorTopKI.get();
      shootMotorsConfig.Slot0.kD = shootMotorTopKD.get();
      shootMotorsConfig.Slot0.kS = shootMotorTopKS.get();
    } else {
      shootMotorsConfig.Slot0.kP = shootMotorBottomKP.get();
      shootMotorsConfig.Slot0.kI = shootMotorBottomKI.get();
      shootMotorsConfig.Slot0.kD = shootMotorBottomKD.get();
      shootMotorsConfig.Slot0.kS = shootMotorBottomKS.get();
    }

    shootMotorsConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOT_MOTORS_GEAR_RATIO;

    shootMotorsConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    shootMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(shootMotor, shootMotorsConfig, configAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, isTopMotor ? "TopMotor" : "BottomMotor", shootMotor);
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
