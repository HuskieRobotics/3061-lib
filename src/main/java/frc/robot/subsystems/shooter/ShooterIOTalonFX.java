package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
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

  private AngularVelocity shootTopMotorReferenceVelocity;
  private AngularVelocity shootBottomMotorReferenceVelocity;

  private final Debouncer topMotorConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer bottomMotorConnectedDebouncer = new Debouncer(0.5);

  private VelocitySystemSim shootMotorTopSim;
  private VelocitySystemSim shootMotorBottomSim;

  private Alert configAlert =
      new Alert("Failed to apply configuration for shooter.", AlertType.kError);

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

  private TalonFX shootMotorTop;
  private TalonFX shootMotorBottom;

  public ShooterIOTalonFX() {

    shootMotorTop = new TalonFX(TOP_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    shootMotorBottom =
        new TalonFX(BOTTOM_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());

    shootMotorTopVelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotorBottomVelocityRequest = new VelocityTorqueCurrentFOC(0);

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
        shootMotorBottomVoltageStatusSignal);

    configShootMotor(shootMotorTop, SHOOT_TOP_INVERTED, true);
    configShootMotor(shootMotorBottom, SHOOT_BOTTOM_INVERTED, false);

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

    // Updates Top Shooter Motor Inputs
    inputs.shootMotorTopStatorCurrentAmps =
        shootMotorTopStatorCurrentStatusSignal.getValueAsDouble();
    inputs.shootMotorTopSupplyCurrentAmps =
        shootMotorTopSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.shootMotorTopVelocityRPS = shootMotorTopVelocityStatusSignal.getValueAsDouble();
    inputs.shootMotorTopTemperatureCelsius =
        shootMotorTopTemperatureStatusSignal.getValueAsDouble();
    inputs.shootMotorTopVoltage = shootMotorTopVoltageStatusSignal.getValueAsDouble();
    inputs.shootMotorTopReferenceVelocityRPS =
        shootMotorTop.getClosedLoopReference().getValueAsDouble();

    // Updates Bottom Shooter Motor Inputs
    inputs.shootMotorBottomStatorCurrentAmps =
        shootMotorBottomStatorCurrentStatusSignal.getValueAsDouble();
    inputs.shootMotorBottomSupplyCurrentAmps =
        shootMotorBottomSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.shootMotorBottomVelocityRPS = shootMotorBottomVelocityStatusSignal.getValueAsDouble();
    inputs.shootMotorBottomTemperatureCelsius =
        shootMotorBottomTemperatureStatusSignal.getValueAsDouble();
    inputs.shootMotorBottomVoltage = shootMotorBottomVoltageStatusSignal.getValueAsDouble();
    inputs.shootMotorBottomReferenceVelocityRPS =
        shootMotorBottom.getClosedLoopReference().getValueAsDouble();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode. To eliminate the performance hit, only retrieve the closed loop reference
    // signals if the tuning mode is enabled. It is critical that these input values are only used
    // for tuning and not used elsewhere in the subsystem. For example, the
    // shootMotorTopReferenceVelocityRPS property should be used throughout the subsystem since it
    // will always be populated.
    if (Constants.TUNING_MODE) {
      inputs.shootMotorTopClosedLoopReferenceVelocityRPS =
          shootMotorTop.getClosedLoopReference().getValue();
      inputs.shootMotorTopClosedLoopErrorVelocityRPS =
          shootMotorTop.getClosedLoopError().getValue();
      inputs.shootMotorBottomClosedLoopReferenceVelocityRPS =
          shootMotorBottom.getClosedLoopReference().getValue();
      inputs.shootMotorBottomClosedLoopErrorVelocityRPS =
          shootMotorBottom.getClosedLoopError().getValue();
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

    // The last step in the updateInputs method is to update the simulation.
    this.shootMotorBottomSim.updateSim();
    this.shootMotorTopSim.updateSim();
  }

  @Override
  public void setShooterWheelTopVelocity(AngularVelocity velocity) {
    shootMotorTop.setControl(shootMotorTopVelocityRequest.withVelocity(velocity));

    // To improve performance, we store the reference velocity as an instance variable to avoid
    // having to retrieve the status signal object from the device in the updateInputs method.
    this.shootTopMotorReferenceVelocity = velocity;
  }

  // While we cannot use subtypes of Measure in the inputs class due to logging limitations, we do
  // strive to use them (e.g., AngularVelocity) throughout the rest of the code to mitigate bugs due
  // to unit mismatches.
  @Override
  public void setShooterWheelBottomVelocity(AngularVelocity velocity) {
    shootMotorBottom.setControl(shootMotorBottomVelocityRequest.withVelocity(velocity));

    // To improve performance, we store the reference velocity as an instance variable to avoid
    // having to retrieve the status signal object from the device in the updateInputs method.
    this.shootBottomMotorReferenceVelocity = velocity;
  }

  private void configShootMotor(TalonFX shootMotor, boolean isInverted, boolean isTopMotor) {

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
}
