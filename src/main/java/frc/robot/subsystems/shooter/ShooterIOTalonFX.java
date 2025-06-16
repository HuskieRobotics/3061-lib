package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class ShooterIOTalonFX implements ShooterIO {

  // Using VelocityTorqueCurrentFOC to set the velocity of the motors
  private VelocityTorqueCurrentFOC shootMotorTopVelocityRequest;
  private VelocityTorqueCurrentFOC shootMotorBottomVelocityRequest;

  // Using StatusSignal to get the stator current of the motors
  private StatusSignal<Current> shootMotorTopStatorCurrentStatusSignal;
  private StatusSignal<Current> shootMotorBottomStatorCurrentStatusSignal;

  // Using StatusSignal to get the supply current of the motors
  private StatusSignal<Current> shootMotorTopSupplyCurrentStatusSignal;
  private StatusSignal<Current> shootMotorBottomSupplyCurrentStatusSignal;

  // Using StatusSignal to get the velocity of the motors
  private StatusSignal<AngularVelocity> shootMotorTopVelocityStatusSignal;
  private StatusSignal<AngularVelocity> shootMotorBottomVelocityStatusSignal;

  private StatusSignal<Temperature> shootMotorTopTemperatureStatusSignal;
  private StatusSignal<Temperature> shootMotorBottomTemperatureStatusSignal;

  private StatusSignal<Voltage> shootMotorTopVoltageStatusSignal;
  private StatusSignal<Voltage> shootMotorBottomVoltageStatusSignal;

  private VelocitySystemSim shootMotorTopSim;
  private VelocitySystemSim shootMotorBottomSim;

  private Alert configAlert =
      new Alert("Failed to apply configuration for shooter.", AlertType.kError);

  // Shoot PID Tunable Numbers
  private final LoggedTunableNumber shootMotorTopKP =
      new LoggedTunableNumber("Shooter/SHOOT_TOP_KP", ShooterConstants.TOP_SHOOT_KP);
  private final LoggedTunableNumber shootMotorTopKI =
      new LoggedTunableNumber("Shooter/SHOOT_TOP_KI", ShooterConstants.TOP_SHOOT_KI);
  private final LoggedTunableNumber shootMotorTopKD =
      new LoggedTunableNumber("Shooter/SHOOT_TOP_KD", ShooterConstants.TOP_SHOOT_KD);
  private final LoggedTunableNumber shootMotorTopKS =
      new LoggedTunableNumber("Shooter/SHOOT_TOP_KS", ShooterConstants.TOP_SHOOT_KS);

  private final LoggedTunableNumber shootMotorBottomKP =
      new LoggedTunableNumber("Shooter/SHOOT_BOTTOM_KP", ShooterConstants.BOTTOM_SHOOT_KP);
  private final LoggedTunableNumber shootMotorBottomKI =
      new LoggedTunableNumber("Shooter/SHOOT_BOTTOM_KI", ShooterConstants.BOTTOM_SHOOT_KI);
  private final LoggedTunableNumber shootMotorBottomKD =
      new LoggedTunableNumber("Shooter/SHOOT_BOTTOM_KD", ShooterConstants.BOTTOM_SHOOT_KD);
  private final LoggedTunableNumber shootMotorBottomKS =
      new LoggedTunableNumber("Shooter/SHOOT_BOTTOM_KS", ShooterConstants.BOTTOM_SHOOT_KS);

  private TalonFX shootMotorTop;
  private TalonFX shootMotorBottom;

  private double topWheelVelocity;
  private double bottomWheelVelocity;

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
  public void updateInputs(ShooterIOInputs shooterInputs) {
    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.

    // Updates Top Shooter Motor Inputs
    shooterInputs.shootMotorTopStatorCurrentAmps =
        shootMotorTopStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopSupplyCurrentAmps =
        shootMotorTopSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopVelocityRPS = shootMotorTopVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopReferenceVelocityRPS = this.topWheelVelocity;
    shooterInputs.shootMotorTopClosedLoopReferenceRPS =
        shootMotorTop.getClosedLoopReference().getValueAsDouble();
    shooterInputs.shootMotorTopTemperatureCelsius =
        shootMotorTopTemperatureStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopVoltage = shootMotorTopVoltageStatusSignal.getValueAsDouble();

    // Updates Bottom Shooter Motor Inputs
    shooterInputs.shootMotorBottomStatorCurrentAmps =
        shootMotorBottomStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomSupplyCurrentAmps =
        shootMotorBottomSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomVelocityRPS =
        shootMotorBottomVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomReferenceVelocityRPS = this.bottomWheelVelocity;
    shooterInputs.shootMotorBottomClosedLoopReferenceRPS =
        shootMotorBottom.getClosedLoopReference().getValueAsDouble();
    shooterInputs.shootMotorBottomTemperatureCelsius =
        shootMotorBottomTemperatureStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomVoltage = shootMotorBottomVoltageStatusSignal.getValueAsDouble();

    // check if the tunable numbers have changed for the top and bottom shooter motors
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

    this.shootMotorBottomSim.updateSim();
    this.shootMotorTopSim.updateSim();
  }

  @Override
  public void setShooterWheelTopVelocity(double rps) {
    shootMotorTop.setControl(shootMotorTopVelocityRequest.withVelocity(rps));
    this.topWheelVelocity = rps;
  }

  @Override
  public void setShooterWheelBottomVelocity(double rps) {
    shootMotorBottom.setControl(shootMotorBottomVelocityRequest.withVelocity(rps));
    this.bottomWheelVelocity = rps;
  }

  private void configShootMotor(TalonFX shootMotor, boolean isInverted, boolean isTopMotor) {

    TalonFXConfiguration shootMotorsConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs shootMotorTorqueCurrentConfigs = new TorqueCurrentConfigs();

    if (isTopMotor) {
      shootMotorTorqueCurrentConfigs.PeakForwardTorqueCurrent =
          ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
      shootMotorTorqueCurrentConfigs.PeakReverseTorqueCurrent =
          -ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
    } else {
      shootMotorTorqueCurrentConfigs.PeakForwardTorqueCurrent =
          ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
      shootMotorTorqueCurrentConfigs.PeakReverseTorqueCurrent =
          -ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
    }

    shootMotorsConfig.TorqueCurrent = shootMotorTorqueCurrentConfigs;

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

    shootMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shootMotorsConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOT_MOTORS_GEAR_RATIO;

    shootMotorsConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(shootMotor, shootMotorsConfig, configAlert);

    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, isTopMotor ? "TopMotor" : "BottomMotor", shootMotor);
  }
}
