package frc.lib.team3061.swerve;

import static frc.lib.team3061.swerve.SwerveModuleConstants.*;

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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;

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
  private final InvertedValue driveMotorInverted;
  private final double angleGearRatio;
  private final InvertedValue angleMotorInverted;
  private final boolean canCoderInverted;

  private final String canBusName = RobotConfig.getInstance().getCANBusName();

  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANcoder angleEncoder;
  private double angleOffsetRot;

  private VoltageOut driveVoltageRequest;
  private VelocityVoltage driveVelocityRequest;
  private PositionVoltage anglePositionRequest;

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
      driveMotorInverted =
          MK4_L2_DRIVE_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      angleGearRatio = MK4_L2_ANGLE_GEAR_RATIO;
      angleMotorInverted =
          MK4_L2_ANGLE_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      canCoderInverted = MK4_L2_CAN_CODER_INVERTED;
    } else { // MK4I

      wheelCircumference = MK4I_L2_WHEEL_CIRCUMFERENCE;
      driveGearRatio = MK4I_L2_DRIVE_GEAR_RATIO;
      driveMotorInverted =
          MK4I_L2_DRIVE_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      angleGearRatio = MK4I_L2_ANGLE_GEAR_RATIO;
      angleMotorInverted =
          MK4I_L2_ANGLE_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
      canCoderInverted = MK4I_L2_CAN_CODER_INVERTED;
    }

    configAngleEncoder(canCoderID);
    configAngleMotor(angleMotorID);
    configDriveMotor(driveMotorID);
  }

  private void configAngleEncoder(int canCoderID) {
    angleEncoder = new CANcoder(canCoderID, canBusName);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    config.MagnetSensor.SensorDirection =
        canCoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    angleEncoder.getConfigurator().apply(config);
  }

  private void configAngleMotor(int angleMotorID) {
    this.angleMotor = new TalonFX(angleMotorID, canBusName);

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = ANGLE_CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentThreshold = ANGLE_PEAK_CURRENT_LIMIT;
    currentLimits.SupplyTimeThreshold = ANGLE_PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = ANGLE_ENABLE_CURRENT_LIMIT;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted = angleMotorInverted;
    config.MotorOutput.NeutralMode = ANGLE_NEUTRAL_MODE;
    config.Slot0.kP = turnKp.get();
    config.Slot0.kI = turnKi.get();
    config.Slot0.kD = turnKd.get();

    this.angleMotor.getConfigurator().apply(config);

    this.angleEncoder.getAbsolutePosition().waitForUpdate(0.1);
    this.angleMotor.setRotorPosition(
        (this.angleEncoder.getAbsolutePosition().getValue() - angleOffsetRot) * angleGearRatio);

    this.anglePositionRequest = new PositionVoltage(0.0).withSlot(0);
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

    config.MotorOutput.Inverted = driveMotorInverted;
    config.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;
    config.Slot0.kP = driveKp.get();
    config.Slot0.kI = driveKi.get();
    config.Slot0.kD = driveKd.get();
    config.Slot0.kS = RobotConfig.getInstance().getDriveKS();
    config.Slot0.kV = RobotConfig.getInstance().getDriveKV();
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;

    this.driveMotor.setRotorPosition(0.0);

    this.driveVoltageRequest = new VoltageOut(0.0);
    this.driveVoltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.driveVelocityRequest = new VelocityVoltage(0).withSlot(0);
    this.driveVelocityRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            this.driveMotor.getPosition().getValue(), driveGearRatio);
    inputs.driveDistanceMeters =
        Conversions.falconRotationsToMechanismMeters(
            this.driveMotor.getPosition().getValue(), wheelCircumference, driveGearRatio);
    inputs.driveVelocityMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            this.driveMotor.getVelocity().getValue(), wheelCircumference, driveGearRatio);
    inputs.driveAppliedPercentage = this.driveMotor.getDutyCycle().getValue() / 2.0;
    inputs.driveStatorCurrentAmps = this.driveMotor.getStatorCurrent().getValue();
    inputs.driveSupplyCurrentAmps = this.driveMotor.getSupplyCurrent().getValue();
    inputs.driveTempCelsius = this.driveMotor.getDeviceTemp().getValue();

    inputs.angleAbsolutePositionDeg = this.angleEncoder.getAbsolutePosition().getValue() * 360.0;
    inputs.anglePositionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            this.angleMotor.getPosition().getValue(), angleGearRatio);
    inputs.angleVelocityRevPerMin =
        Conversions.falconRPSToMechanismRPM(
            this.angleMotor.getVelocity().getValue(), angleGearRatio);
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
        anglePositionRequest.withPosition(
            Conversions.degreesToFalconRotations(degrees, angleGearRatio)));
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
}