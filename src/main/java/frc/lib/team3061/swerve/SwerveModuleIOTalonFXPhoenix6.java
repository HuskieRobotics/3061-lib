package frc.lib.team3061.swerve;

import static frc.lib.team3061.swerve.SwerveModuleConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;

/**
 * Implementation of the SwerveModuleIO interface for MK4 Swerve Modules with two Falcon 500 motors
 * and a CANcoder.
 */
public class SwerveModuleIOTalonFXPhoenix6 implements SwerveModuleIO {

  private final TunableNumber driveKp =
      new TunableNumber("Drive/DriveKp", RobotConfig.getInstance().getSwerveDriveKP());
  private final TunableNumber driveKi =
      new TunableNumber("Drive/DriveKi", RobotConfig.getInstance().getSwerveDriveKI());
  private final TunableNumber driveKd =
      new TunableNumber("Drive/DriveKd", RobotConfig.getInstance().getSwerveDriveKD());
  private final TunableNumber turnKp =
      new TunableNumber("Drive/TurnKp", RobotConfig.getInstance().getSwerveAngleKP());
  private final TunableNumber turnKi =
      new TunableNumber("Drive/TurnKi", RobotConfig.getInstance().getSwerveAngleKI());
  private final TunableNumber turnKd =
      new TunableNumber("Drive/TurnKd", RobotConfig.getInstance().getSwerveAngleKD());

  private final double wheelCircumference;
  private final double driveGearRatio;
  private final boolean driveMotorInverted;
  private final double angleGearRatio;
  private final boolean angleMotorInverted;
  private final boolean canCoderInverted;

  private final String canBusName = RobotConfig.getInstance().getCANBusName();

  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANcoder angleEncoder;
  private double angleOffsetRot;

  private VoltageOut driveVoltageRequest;
  private VelocityVoltage driveVelocityRequest;
  private PositionVoltage anglePositionRequest;

  private StatusSignal<Double> anglePositionStatusSignal;
  private StatusSignal<Double> angleVelocityStatusSignal;
  private StatusSignal<Double> anglePositionErrorStatusSignal;
  private StatusSignal<Double> drivePositionStatusSignal;
  private StatusSignal<Double> driveVelocityStatusSignal;
  private StatusSignal<Double> driveVelocityErrorStatusSignal;

  private Alert angleEncoderConfigAlert =
      new Alert("Failed to apply configuration for angle encoder.", AlertType.ERROR);
  private Alert angleMotorConfigAlert =
      new Alert("Failed to apply configuration for angle motor.", AlertType.ERROR);
  private Alert driveMotorConfigAlert =
      new Alert("Failed to apply configuration for drive motor.", AlertType.ERROR);

  private TalonFXSimState angleMotorSimState;
  private TalonFXSimState driveMotorSimState;
  private CANcoderSimState angleEncoderSimState;
  private LinearSystemSim<N1, N1, N1> driveSim;
  private FlywheelSim turnSim;

  /**
   * Make a new SwerveModuleIOTalonFX object.
   *
   * @param moduleNumber the module number (0-3); primarily used for logging
   * @param driveMotorID the CAN ID of the drive motor
   * @param angleMotorID the CAN ID of the angle motor
   * @param canCoderID the CAN ID of the CANcoder
   * @param angleOffsetRot the absolute offset of the angle encoder in rotations
   */
  public SwerveModuleIOTalonFXPhoenix6(
      int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID, double angleOffsetRot) {

    this.angleOffsetRot = angleOffsetRot;

    if (RobotConfig.getInstance().getSwerveType() == SwerveType.MK4) {

      wheelCircumference = MK4_L2_WHEEL_CIRCUMFERENCE;
      driveGearRatio = MK4_L2_DRIVE_GEAR_RATIO;
      driveMotorInverted = MK4_L2_DRIVE_MOTOR_INVERTED;
      angleGearRatio = MK4_L2_ANGLE_GEAR_RATIO;
      angleMotorInverted = MK4_L2_ANGLE_MOTOR_INVERTED;
      canCoderInverted = MK4_L2_CAN_CODER_INVERTED;
    } else { // MK4I

      wheelCircumference = MK4I_L2_WHEEL_CIRCUMFERENCE;
      driveGearRatio = MK4I_L2_DRIVE_GEAR_RATIO;
      driveMotorInverted = MK4I_L2_DRIVE_MOTOR_INVERTED;
      angleGearRatio = MK4I_L2_ANGLE_GEAR_RATIO;
      angleMotorInverted = MK4I_L2_ANGLE_MOTOR_INVERTED;
      canCoderInverted = MK4I_L2_CAN_CODER_INVERTED;
    }

    configAngleEncoder(canCoderID);
    configAngleMotor(angleMotorID, canCoderID);
    configDriveMotor(driveMotorID);
    configSim();
  }

  private void configAngleEncoder(int canCoderID) {
    angleEncoder = new CANcoder(canCoderID, canBusName);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    config.MagnetSensor.MagnetOffset = angleOffsetRot;
    config.MagnetSensor.SensorDirection =
        canCoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = this.angleEncoder.getConfigurator().apply(config);
      if (status.isOK()) {
        angleEncoderConfigAlert.set(false);
        break;
      }
    }
    if (!status.isOK()) {
      angleEncoderConfigAlert.set(true);
      angleEncoderConfigAlert.setText(status.toString());
    }
  }

  private void configAngleMotor(int angleMotorID, int canCoderID) {
    this.angleMotor = new TalonFX(angleMotorID, canBusName);

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = ANGLE_CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentThreshold = ANGLE_PEAK_CURRENT_LIMIT;
    currentLimits.SupplyTimeThreshold = ANGLE_PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = ANGLE_ENABLE_CURRENT_LIMIT;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
        angleMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = ANGLE_NEUTRAL_MODE;

    config.Slot0.kP = turnKp.get();
    config.Slot0.kI = turnKi.get();
    config.Slot0.kD = turnKd.get();

    config.ClosedLoopGeneral.ContinuousWrap = true;

    config.Feedback.FeedbackRemoteSensorID = canCoderID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = angleGearRatio;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = this.angleMotor.getConfigurator().apply(config);
      if (status.isOK()) {
        angleMotorConfigAlert.set(false);
        break;
      }
    }
    if (!status.isOK()) {
      angleMotorConfigAlert.set(true);
      angleMotorConfigAlert.setText(status.toString());
    }

    this.anglePositionStatusSignal = this.angleMotor.getPosition();
    this.angleVelocityStatusSignal = this.angleMotor.getVelocity();
    this.anglePositionErrorStatusSignal = this.angleMotor.getClosedLoopError();

    this.anglePositionRequest = new PositionVoltage(0.0).withSlot(0);
    this.anglePositionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
  }

  private void configDriveMotor(int driveMotorID) {
    this.driveMotor = new TalonFX(driveMotorID, canBusName);

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = DRIVE_CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentThreshold = DRIVE_PEAK_CURRENT_LIMIT;
    currentLimits.SupplyTimeThreshold = DRIVE_PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_CURRENT_LIMIT;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
        driveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;
    config.Slot0.kP = driveKp.get();
    config.Slot0.kI = driveKi.get();
    config.Slot0.kD = driveKd.get();
    config.Slot0.kS = RobotConfig.getInstance().getDriveKS();
    config.Slot0.kV = RobotConfig.getInstance().getDriveKV();
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = this.driveMotor.getConfigurator().apply(config);
      if (status.isOK()) {
        driveMotorConfigAlert.set(false);
        break;
      }
    }
    if (!status.isOK()) {
      driveMotorConfigAlert.set(true);
      driveMotorConfigAlert.setText(status.toString());
    }

    this.driveMotor.setRotorPosition(0.0);

    this.drivePositionStatusSignal = this.driveMotor.getPosition();
    this.driveVelocityStatusSignal = this.driveMotor.getVelocity();
    this.driveVelocityErrorStatusSignal = this.driveMotor.getClosedLoopError();

    this.driveVoltageRequest = new VoltageOut(0.0);
    this.driveVoltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.driveVelocityRequest = new VelocityVoltage(0).withSlot(0);
    this.driveVelocityRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    updateSim();

    // only invoke refresh if Phoenix is not licensed (if licensed, these signals have already been
    // refreshed)
    if (!RobotConfig.getInstance().getPhoenix6Licensed()) {
      anglePositionStatusSignal.refresh();
      angleVelocityStatusSignal.refresh();
      drivePositionStatusSignal.refresh();
      driveVelocityStatusSignal.refresh();
    }

    anglePositionErrorStatusSignal.refresh();
    driveVelocityErrorStatusSignal.refresh();

    inputs.drivePositionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            drivePositionStatusSignal.getValue(), driveGearRatio);
    inputs.driveDistanceMeters =
        Conversions.falconRotationsToMechanismMeters(
            BaseStatusSignal.getLatencyCompensatedValue(
                drivePositionStatusSignal, driveVelocityStatusSignal),
            wheelCircumference,
            driveGearRatio);
    inputs.driveVelocityMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            driveVelocityStatusSignal.getValue(), wheelCircumference, driveGearRatio);
    inputs.driveVelocityErrorMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            driveVelocityErrorStatusSignal.getValue(), wheelCircumference, driveGearRatio);
    inputs.driveAppliedPercentage = this.driveMotor.getDutyCycle().getValue() / 2.0;
    inputs.driveStatorCurrentAmps = this.driveMotor.getStatorCurrent().getValue();
    inputs.driveSupplyCurrentAmps = this.driveMotor.getSupplyCurrent().getValue();
    inputs.driveTempCelsius = this.driveMotor.getDeviceTemp().getValue();

    inputs.angleAbsolutePositionDeg = this.angleEncoder.getAbsolutePosition().getValue() * 360.0;
    // since we are using the FusedCANcoder feature, the position and velocity signal for the angle
    // motor accounts for the gear ratio; so, pass a gear ratio of 1 to just convert from rotations
    // to degrees.
    inputs.anglePositionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            BaseStatusSignal.getLatencyCompensatedValue(
                anglePositionStatusSignal, angleVelocityStatusSignal),
            1);
    inputs.anglePositionErrorDeg =
        Conversions.falconRotationsToMechanismDegrees(anglePositionErrorStatusSignal.getValue(), 1);
    inputs.angleVelocityRevPerMin =
        Conversions.falconRPSToMechanismRPM(angleVelocityStatusSignal.getValue(), 1);

    inputs.angleAppliedPercentage = this.angleMotor.getDutyCycle().getValue() / 2.0;
    inputs.angleStatorCurrentAmps = this.angleMotor.getStatorCurrent().getValue();
    inputs.angleSupplyCurrentAmps = this.angleMotor.getSupplyCurrent().getValue();
    inputs.angleTempCelsius = this.angleMotor.getDeviceTemp().getValue();

    // update tunables
    if (driveKp.hasChanged()
        || driveKi.hasChanged()
        || driveKd.hasChanged()
        || turnKp.hasChanged()
        || turnKi.hasChanged()
        || turnKd.hasChanged()) {
      Slot0Configs driveSlot0 = new Slot0Configs();
      this.driveMotor.getConfigurator().refresh(driveSlot0);
      driveSlot0.kP = driveKp.get();
      driveSlot0.kI = driveKi.get();
      driveSlot0.kD = driveKd.get();
      this.driveMotor.getConfigurator().apply(driveSlot0);

      Slot0Configs angleSlot0 = new Slot0Configs();
      this.angleMotor.getConfigurator().refresh(angleSlot0);
      angleSlot0.kP = turnKp.get();
      angleSlot0.kI = turnKi.get();
      angleSlot0.kD = turnKd.get();
      this.angleMotor.getConfigurator().apply(angleSlot0);
    }
  }

  /** Run the drive motor at the specified percentage of full power (12 V). */
  @Override
  public void setDriveMotorPercentage(double percentage) {
    this.driveMotor.setControl(driveVoltageRequest.withOutput(percentage * 12.0));
  }

  /** Run the drive motor at the specified velocity. */
  @Override
  public void setDriveVelocity(double velocity) {
    double rps = Conversions.mpsToFalconRPS(velocity, wheelCircumference, driveGearRatio);
    this.driveMotor.setControl(driveVelocityRequest.withVelocity(rps));
  }

  /** Run the turn motor to the specified angle. */
  @Override
  public void setAnglePosition(double degrees) {
    this.angleMotor.setControl(
        anglePositionRequest.withPosition(Conversions.degreesToFalconRotations(degrees, 1)));
  }

  /** Enable or disable brake mode on the drive motor. */
  @Override
  public void setDriveBrakeMode(boolean enable) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    this.driveMotor.getConfigurator().refresh(config);
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    this.driveMotor.getConfigurator().apply(config);
  }

  /** Enable or disable brake mode on the turn motor. */
  @Override
  public void setAngleBrakeMode(boolean enable) {
    // always leave the angle motor in coast mode
    MotorOutputConfigs config = new MotorOutputConfigs();
    this.angleMotor.getConfigurator().refresh(config);
    config.NeutralMode = NeutralModeValue.Coast;
    this.angleMotor.getConfigurator().apply(config);
  }

  @Override
  public List<StatusSignal<Double>> getOdometryStatusSignals() {
    ArrayList<StatusSignal<Double>> signals = new ArrayList<>();
    signals.add(drivePositionStatusSignal);
    signals.add(driveVelocityStatusSignal);
    signals.add(anglePositionStatusSignal);
    signals.add(angleVelocityStatusSignal);
    return signals;
  }

  private void configSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }
    this.angleEncoderSimState = this.angleEncoder.getSimState();
    this.angleEncoderSimState.Orientation =
        this.canCoderInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    this.angleMotorSimState = this.angleMotor.getSimState();
    this.angleMotorSimState.Orientation =
        this.angleMotorInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    this.driveMotorSimState = this.driveMotor.getSimState();
    this.driveMotorSimState.Orientation =
        this.driveMotorInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    // replace the flywheel sim with the position system after characterizing the rotation of the
    // swerve modules on the robot
    this.turnSim = new FlywheelSim(DCMotor.getFalcon500(1), angleGearRatio, 0.5);
    this.driveSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyVelocitySystem(
                RobotConfig.getInstance().getDriveKV(), RobotConfig.getInstance().getDriveKA()));
  }

  private void updateSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    this.angleEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.angleMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.driveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // update the input voltages of the models based on the outputs of the simulated TalonFXs
    this.turnSim.setInputVoltage(this.angleMotorSimState.getMotorVoltage());
    this.driveSim.setInput(this.driveMotorSimState.getMotorVoltage());

    // update the models
    this.driveSim.update(Constants.LOOP_PERIOD_SECS);
    this.turnSim.update(Constants.LOOP_PERIOD_SECS);

    // update the simulated TalonFXs and CANcoder based on the model outputs
    double turnRPM = turnSim.getAngularVelocityRPM();
    double angleEncoderRPS = turnRPM / 60;
    double angleEncoderRotations = angleEncoderRPS * Constants.LOOP_PERIOD_SECS;

    this.angleEncoderSimState.addPosition(angleEncoderRotations);
    this.angleMotorSimState.addRotorPosition(angleEncoderRotations);
    this.angleMotorSimState.setRotorVelocity(angleEncoderRPS);

    double driveMPS = driveSim.getOutput(0);
    double driveMotorRPS = Conversions.mpsToFalconRPS(driveMPS, wheelCircumference, driveGearRatio);
    double driveMotorRotations = driveMotorRPS * Constants.LOOP_PERIOD_SECS;
    this.driveMotorSimState.addRotorPosition(driveMotorRotations);
    this.driveMotorSimState.setRotorVelocity(driveMotorRPS);
  }
}
