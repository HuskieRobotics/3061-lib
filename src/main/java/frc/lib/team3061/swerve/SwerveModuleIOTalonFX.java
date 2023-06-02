package frc.lib.team3061.swerve;

import static frc.lib.team3061.swerve.SwerveModuleConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team254.drivers.TalonFXFactory;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;

/**
 * Implementation of the SwerveModuleIO interface for MK4 Swerve Modules with two Falcon 500 motors
 * and a CANcoder.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {

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

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANCoder angleEncoder;
  private SimpleMotorFeedforward feedForward;
  private double angleOffsetDeg;

  /**
   * Make a new SwerveModuleIOTalonFX object.
   *
   * @param moduleNumber the module number (0-3); primarily used for logging
   * @param driveMotorID the CAN ID of the drive motor
   * @param angleMotorID the CAN ID of the angle motor
   * @param canCoderID the CAN ID of the CANcoder
   * @param angleOffsetDeg the absolute offset of the angle encoder in degrees
   */
  public SwerveModuleIOTalonFX(
      int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID, double angleOffsetDeg) {

    this.angleOffsetDeg = angleOffsetDeg;

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

    this.feedForward =
        new SimpleMotorFeedforward(
            RobotConfig.getInstance().getDriveKS() / 12,
            RobotConfig.getInstance().getDriveKV() / 12,
            RobotConfig.getInstance().getDriveKA() / 12);

    configAngleEncoder(canCoderID);
    configAngleMotor(angleMotorID);
    configDriveMotor(driveMotorID);
  }

  private void configAngleEncoder(int canCoderID) {
    angleEncoder = new CANCoder(canCoderID, canBusName);

    angleEncoder.configFactoryDefault();

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = canCoderInverted;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    angleEncoder.configAllSettings(config);
  }

  private void configAngleMotor(int angleMotorID) {
    TalonFXFactory.Configuration angleMotorConfig = new TalonFXFactory.Configuration();
    angleMotorConfig.SUPPLY_CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(
            ANGLE_ENABLE_CURRENT_LIMIT,
            ANGLE_CONTINUOUS_CURRENT_LIMIT,
            ANGLE_PEAK_CURRENT_LIMIT,
            ANGLE_PEAK_CURRENT_DURATION);
    angleMotorConfig.INVERTED = angleMotorInverted;
    angleMotorConfig.NEUTRAL_MODE = ANGLE_NEUTRAL_MODE;
    angleMotorConfig.SLOT0_KP = turnKp.get();
    angleMotorConfig.SLOT0_KI = turnKi.get();
    angleMotorConfig.SLOT0_KD = turnKd.get();
    angleMotorConfig.SLOT0_KF = RobotConfig.getInstance().getSwerveAngleKF();
    angleMotorConfig.GENERAL_STATUS_FRAME_RATE_MS = 9;
    angleMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 19;
    angleMotorConfig.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
    angleMotorConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 101;
    angleMotorConfig.BASE_PIDF0_STATUS_FRAME_RATE_MS = 102;

    mAngleMotor = TalonFXFactory.createTalon(angleMotorID, canBusName, angleMotorConfig);

    double absolutePosition =
        Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffsetDeg, angleGearRatio);
    mAngleMotor.setSelectedSensorPosition(absolutePosition);
    mAngleMotor.configVoltageCompSaturation(12); // default 12v voltage compensation for motors
    mAngleMotor.enableVoltageCompensation(true);
  }

  private void configDriveMotor(int driveMotorID) {
    TalonFXFactory.Configuration driveMotorConfig = new TalonFXFactory.Configuration();
    driveMotorConfig.SUPPLY_CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(
            DRIVE_ENABLE_CURRENT_LIMIT,
            DRIVE_CONTINUOUS_CURRENT_LIMIT,
            DRIVE_PEAK_CURRENT_LIMIT,
            DRIVE_PEAK_CURRENT_DURATION);
    driveMotorConfig.INVERTED = driveMotorInverted;
    driveMotorConfig.NEUTRAL_MODE = DRIVE_NEUTRAL_MODE;
    driveMotorConfig.OPEN_LOOP_RAMP_RATE = OPEN_LOOP_RAMP;
    driveMotorConfig.CLOSED_LOOP_RAMP_RATE = CLOSED_LOOP_RAMP;
    driveMotorConfig.SLOT0_KP = driveKp.get();
    driveMotorConfig.SLOT0_KI = driveKi.get();
    driveMotorConfig.SLOT0_KD = driveKd.get();
    driveMotorConfig.SLOT0_KF = RobotConfig.getInstance().getSwerveDriveKF();
    driveMotorConfig.GENERAL_STATUS_FRAME_RATE_MS = 9;
    driveMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 19;
    driveMotorConfig.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
    driveMotorConfig.MOTION_MAGIC_STATUS_FRAME_RATE_MS = 101;
    driveMotorConfig.BASE_PIDF0_STATUS_FRAME_RATE_MS = 102;

    mDriveMotor = TalonFXFactory.createTalon(driveMotorID, canBusName, driveMotorConfig);

    mDriveMotor.configVoltageCompSaturation(12); // default 12v voltage compensation for motors
    mDriveMotor.enableVoltageCompensation(true);
    mDriveMotor.setSelectedSensorPosition(0);
  }

  private Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  private double calculateFeedforward(double velocity) {
    double percentage = this.feedForward.calculate(velocity);
    // clamp the voltage to the maximum voltage
    if (percentage > 1.0) {
      return 1.0;
    }
    return percentage;
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionDeg =
        Conversions.falconToDegrees(mDriveMotor.getSelectedSensorPosition(), driveGearRatio);
    inputs.driveDistanceMeters =
        Conversions.falconToMeters(
            mDriveMotor.getSelectedSensorPosition(), wheelCircumference, driveGearRatio);
    inputs.driveVelocityMetersPerSec =
        Conversions.falconToMPS(
            mDriveMotor.getSelectedSensorVelocity(), wheelCircumference, driveGearRatio);
    inputs.driveAppliedPercentage = mDriveMotor.getMotorOutputPercent();
    inputs.driveCurrentAmps = new double[] {mDriveMotor.getStatorCurrent()};
    inputs.driveTempCelsius = new double[] {mDriveMotor.getTemperature()};

    inputs.angleAbsolutePositionDeg = angleEncoder.getAbsolutePosition();
    inputs.anglePositionDeg =
        Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), angleGearRatio);
    inputs.angleVelocityRevPerMin =
        Conversions.falconToRPM(mAngleMotor.getSelectedSensorVelocity(), angleGearRatio);
    inputs.angleAppliedPercentage = mAngleMotor.getMotorOutputPercent();
    inputs.angleCurrentAmps = new double[] {mAngleMotor.getStatorCurrent()};
    inputs.angleTempCelsius = new double[] {mAngleMotor.getTemperature()};

    // update tunables
    if (driveKp.hasChanged()
        || driveKi.hasChanged()
        || driveKd.hasChanged()
        || turnKp.hasChanged()
        || turnKi.hasChanged()
        || turnKd.hasChanged()) {
      mDriveMotor.config_kP(SLOT_INDEX, driveKp.get());
      mDriveMotor.config_kI(SLOT_INDEX, driveKi.get());
      mDriveMotor.config_kD(SLOT_INDEX, driveKd.get());
      mAngleMotor.config_kP(SLOT_INDEX, turnKp.get());
      mAngleMotor.config_kI(SLOT_INDEX, turnKi.get());
      mAngleMotor.config_kD(SLOT_INDEX, turnKd.get());
    }
  }

  /** Run the drive motor at the specified percentage of full power. */
  @Override
  public void setDriveMotorPercentage(double percentage) {
    mDriveMotor.set(ControlMode.PercentOutput, percentage);
  }

  /** Run the drive motor at the specified velocity. */
  @Override
  public void setDriveVelocity(double velocity) {
    double ticksPerSecond = Conversions.mpsToFalcon(velocity, wheelCircumference, driveGearRatio);
    mDriveMotor.set(
        ControlMode.Velocity,
        ticksPerSecond,
        DemandType.ArbitraryFeedForward,
        calculateFeedforward(velocity));
  }

  /** Run the turn motor to the specified angle. */
  @Override
  public void setAnglePosition(double degrees) {
    mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(degrees, angleGearRatio));
  }

  /** Enable or disable brake mode on the drive motor. */
  @Override
  public void setDriveBrakeMode(boolean enable) {
    mDriveMotor.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
  }

  /** Enable or disable brake mode on the turn motor. */
  @Override
  public void setAngleBrakeMode(boolean enable) {
    // always leave the angle motor in coast mode
    mAngleMotor.setNeutralMode(NeutralMode.Coast);
  }
}
