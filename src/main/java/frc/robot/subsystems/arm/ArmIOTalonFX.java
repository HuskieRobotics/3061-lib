package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ArmIOTalonFX implements ArmIO {

  // We usually use MotionMagic Expo voltage to control the position of a mechanism.
  private MotionMagicExpoVoltage angleMotorPositionRequest;
  private VoltageOut angleMotorVoltageRequest;

  private StatusSignal<Current> angleMotorStatorCurrentStatusSignal;
  private StatusSignal<Current> angleMotorSupplyCurrentStatusSignal;
  private StatusSignal<Angle> angleMotorPositionStatusSignal;
  private StatusSignal<Temperature> angleMotorTemperatureStatusSignal;
  private StatusSignal<Voltage> angleMotorVoltageStatusSignal;

  private ArmSystemSim angleMotorSim;

  private Alert configAlert = new Alert("Failed to apply configuration for arm.", AlertType.kError);

  // The following enables tuning of the PID and feedforward values for the arm by changing values
  // via AdvantageScope and not needing to change values in code, compile, and re-deploy.
  private final LoggedTunableNumber rotationMotorKP =
      new LoggedTunableNumber("Arm/ROTATION_KP", ArmConstants.ROTATION_KP);
  private final LoggedTunableNumber rotationMotorKI =
      new LoggedTunableNumber("Arm/ROTATION_KI", ArmConstants.ROTATION_KI);
  private final LoggedTunableNumber rotationMotorKD =
      new LoggedTunableNumber("Arm/ROTATION_KD", ArmConstants.ROTATION_KD);
  private final LoggedTunableNumber rotationMotorKS =
      new LoggedTunableNumber("Arm/ROTATION_KS", ArmConstants.ROTATION_KS);
  private final LoggedTunableNumber rotationMotorKG =
      new LoggedTunableNumber("Arm/ROTATION_KG", ArmConstants.ROTATION_KG);
  private final LoggedTunableNumber rotationMotorKA =
      new LoggedTunableNumber("Arm/ROTATION_KA", ArmConstants.ROTATION_KA);
  private final LoggedTunableNumber rotationMotorKV =
      new LoggedTunableNumber("Arm/ROTATION_KV", ArmConstants.ROTATION_KV);
  private final LoggedTunableNumber rotationMotorExpoKV =
      new LoggedTunableNumber("Arm/ROTATION_EXPO_KV", ArmConstants.ROTATION_EXPO_KV);
  private final LoggedTunableNumber rotationMotorExpoKA =
      new LoggedTunableNumber("Arm/ROTATION_EXPO_KA", ArmConstants.ROTATION_EXPO_KA);
  private final LoggedTunableNumber rotationEncoderMagnetOffset =
      new LoggedTunableNumber("Arm/ROTATION_MAGNET_OFFSET", ArmConstants.MAGNET_OFFSET);

  private TalonFX angleMotor;
  private CANcoder angleEncoder;

  public ArmIOTalonFX() {

    angleMotor = new TalonFX(ANGLE_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    angleEncoder = new CANcoder(ANGLE_ENCODER_ID, RobotConfig.getInstance().getCANBusName());

    angleMotorPositionRequest = new MotionMagicExpoVoltage(0);
    angleMotorVoltageRequest = new VoltageOut(0);

    angleMotorPositionStatusSignal = angleMotor.getPosition();
    angleMotorStatorCurrentStatusSignal = angleMotor.getStatorCurrent();
    angleMotorSupplyCurrentStatusSignal = angleMotor.getSupplyCurrent();
    angleMotorTemperatureStatusSignal = angleMotor.getDeviceTemp();
    angleMotorVoltageStatusSignal = angleMotor.getMotorVoltage();

    // To improve performance, subsystems register all their signals with Phoenix6Util. All signals
    // on the entire CAN bus will be refreshed at the same time by Phoenix6Util; so, there is no
    // need to refresh any StatusSignals in this class.
    Phoenix6Util.registerSignals(
        true,
        angleMotorPositionStatusSignal,
        angleMotorStatorCurrentStatusSignal,
        angleMotorSupplyCurrentStatusSignal,
        angleMotorTemperatureStatusSignal,
        angleMotorVoltageStatusSignal);

    configAngleMotor(angleMotor, angleEncoder);

    // Create a simulation object for the arm. The specific parameters for the simulation
    // are determined based on the mechanical design of the arm. The ArmSystemSim class creates a
    // Mechanism2d that can be visualized in AdvantageScope to test code in simulation when the
    // physical mechanism is not available.
    this.angleMotorSim =
        new ArmSystemSim(
            angleMotor,
            angleEncoder,
            ArmConstants.ANGLE_MOTOR_INVERTED,
            ArmConstants.SENSOR_TO_MECHANISM_RATIO,
            ArmConstants.ANGLE_MOTOR_GEAR_RATIO,
            Units.inchesToMeters(20.0),
            Units.lbsToKilograms(20.0),
            Units.degreesToRadians(10.0),
            Units.degreesToRadians(120.0),
            Units.degreesToRadians(10.0),
            SUBSYSTEM_NAME);
  }

  @Override
  public void updateInputs(ArmIOInputs armInputs) {

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.
    armInputs.angleMotorStatorCurrentAmps = angleMotorStatorCurrentStatusSignal.getValueAsDouble();
    armInputs.angleMotorSupplyCurrentAmps = angleMotorSupplyCurrentStatusSignal.getValueAsDouble();
    armInputs.angleMotorVoltage = angleMotorVoltageStatusSignal.getValueAsDouble();
    armInputs.angleDegrees =
        Units.rotationsToDegrees(angleMotorPositionStatusSignal.getValueAsDouble());
    armInputs.angleMotorReferenceAngleDegrees =
        Units.rotationsToDegrees(angleMotor.getClosedLoopReference().getValueAsDouble());
    armInputs.angleMotorTemperatureCelsius = angleMotorTemperatureStatusSignal.getValueAsDouble();

    // In order for a tunable to be useful, there must be code that checks if its value has changed.
    // When a subsystem has multiple tunables that are related, the ifChanged method is a convenient
    // to check and apply changes from multiple tunables at once.
    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.angleMotor.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kG = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kV = motionMagic[6];
          config.MotionMagic.MotionMagicExpo_kV = motionMagic[7];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[8];
          this.angleMotor.getConfigurator().apply(config);
        },
        rotationMotorKP,
        rotationMotorKI,
        rotationMotorKD,
        rotationMotorKS,
        rotationMotorKG,
        rotationMotorKA,
        rotationMotorKV,
        rotationMotorExpoKV,
        rotationMotorExpoKA);

    // If a tunable is unrelated to others, it can be checked and applied individually.
    if (rotationEncoderMagnetOffset.hasChanged(hashCode())) {
      CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();
      angleEncoder.getConfigurator().refresh(angleCANCoderConfig);
      angleCANCoderConfig.MagnetSensor.MagnetOffset = rotationEncoderMagnetOffset.get();
      angleEncoder.getConfigurator().apply(angleCANCoderConfig);
    }

    // The last step in the updateInputs method is to update the simulation.
    this.angleMotorSim.updateSim();
  }

  @Override
  public void setAngle(double angle) {
    angleMotor.setControl(angleMotorPositionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  @Override
  public void setAngleMotorVoltage(double voltage) {
    angleMotor.setControl(angleMotorVoltageRequest.withOutput(voltage));
  }

  private void configAngleMotor(TalonFX angleMotor, CANcoder angleEncoder) {

    CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();
    angleCANCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    angleCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    angleCANCoderConfig.MagnetSensor.MagnetOffset = rotationEncoderMagnetOffset.get();
    Phoenix6Util.applyAndCheckConfiguration(angleEncoder, angleCANCoderConfig, configAlert);

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();

    angleMotorConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
        ArmConstants.ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
        ArmConstants.ANGLE_MOTOR_PEAK_CURRENT_DURATION;
    angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    angleMotorConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleMotorConfig.Slot0.kP = rotationMotorKP.get();
    angleMotorConfig.Slot0.kI = rotationMotorKI.get();
    angleMotorConfig.Slot0.kD = rotationMotorKD.get();
    angleMotorConfig.Slot0.kS = rotationMotorKS.get();
    angleMotorConfig.Slot0.kG = rotationMotorKG.get();
    angleMotorConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
    angleMotorConfig.Slot0.kA = rotationMotorKA.get();
    angleMotorConfig.Slot0.kV = rotationMotorKV.get();

    angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity =
        ArmConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    angleMotorConfig.MotionMagic.MotionMagicExpo_kV = rotationMotorExpoKV.get();
    angleMotorConfig.MotionMagic.MotionMagicExpo_kA = rotationMotorExpoKA.get();

    angleMotorConfig.MotorOutput.Inverted =
        ArmConstants.ANGLE_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Software limit switches are used to prevent the arm from moving beyond its physical limits.
    SoftwareLimitSwitchConfigs angleMotorLimitSwitches = angleMotorConfig.SoftwareLimitSwitch;
    angleMotorLimitSwitches.ForwardSoftLimitEnable = true;
    angleMotorLimitSwitches.ForwardSoftLimitThreshold =
        Units.degreesToRotations(ArmConstants.UPPER_ANGLE_LIMIT);
    angleMotorLimitSwitches.ReverseSoftLimitEnable = true;
    angleMotorLimitSwitches.ReverseSoftLimitThreshold =
        Units.degreesToRotations(ArmConstants.LOWER_ANGLE_LIMIT);

    // For the most accurate measurement of the arm's position, fuse the CANcoder with the encoder
    // in the TalonFX.
    angleMotorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
    angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.SENSOR_TO_MECHANISM_RATIO;
    angleMotorConfig.Feedback.RotorToSensorRatio = ArmConstants.ANGLE_MOTOR_GEAR_RATIO;

    // It is critical that devices are successfully configured. The applyAndCheckConfiguration
    // method will apply the configuration, read back the configuration, and ensure that it is
    // correct. If not, it will reattempt five times and eventually, generate an alert.
    Phoenix6Util.applyAndCheckConfiguration(angleMotor, angleMotorConfig, configAlert);

    // A subsystem needs to register each device with FaultReporter. FaultReporter will check
    // devices for faults periodically when the robot is disabled and generate alerts if any faults
    // are found.
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "AngleMotor", angleMotor);
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "AngleCANcoder", angleEncoder);
  }
}
