package frc.lib.team3061.swerve;

import static frc.lib.team3061.swerve.SwerveModuleConstants.*;
import static frc.robot.Constants.*;

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
import frc.lib.team3061.util.CANDeviceFinder;
import frc.lib.team3061.util.CANDeviceId.CANDeviceType;
import frc.lib.team6328.util.TunableNumber;

/**
 * Implementation of the SwerveModuleIO interface for MK4 Swerve Modules with two Falcon 500 motors
 * and a CAN coder.
 */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {

  private final TunableNumber driveKp = new TunableNumber("Drive/DriveKp", DRIVE_KP);
  private final TunableNumber driveKi = new TunableNumber("Drive/DriveKi", DRIVE_KI);
  private final TunableNumber driveKd = new TunableNumber("Drive/DriveKd", DRIVE_KD);
  private final TunableNumber turnKp = new TunableNumber("Drive/TurnKp", ANGLE_KP);
  private final TunableNumber turnKi = new TunableNumber("Drive/TurnKi", ANGLE_KI);
  private final TunableNumber turnKd = new TunableNumber("Drive/TurnKd", ANGLE_KD);

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

    this.feedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

    CANDeviceFinder can = new CANDeviceFinder();
    can.isDevicePresent(CANDeviceType.TALON, driveMotorID, "Mod " + moduleNumber + "Drive");
    can.isDevicePresent(CANDeviceType.TALON, angleMotorID, "Mod " + moduleNumber + "Angle");
    // check for the CANcoder on the CAN bus when supported by CANDeviceFinder

    configAngleEncoder(canCoderID);
    configAngleMotor(angleMotorID);
    configDriveMotor(driveMotorID);
  }

  private void configAngleEncoder(int canCoderID) {
    angleEncoder = new CANCoder(canCoderID, CAN_BUS_NAME);

    angleEncoder.configFactoryDefault();

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = CAN_CODER_INVERTED;
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
    angleMotorConfig.INVERTED = ANGLE_MOTOR_INVERTED;
    angleMotorConfig.NEUTRAL_MODE = ANGLE_NEUTRAL_MODE;
    angleMotorConfig.SLOT0_KP = turnKp.get();
    angleMotorConfig.SLOT0_KI = turnKi.get();
    angleMotorConfig.SLOT0_KD = turnKd.get();
    angleMotorConfig.SLOT0_KF = ANGLE_KF;
    angleMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 20;

    mAngleMotor = TalonFXFactory.createTalon(angleMotorID, CAN_BUS_NAME, angleMotorConfig);

    double absolutePosition =
        Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffsetDeg, ANGLE_GEAR_RATIO);
    mAngleMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configDriveMotor(int driveMotorID) {
    TalonFXFactory.Configuration driveMotorConfig = new TalonFXFactory.Configuration();
    driveMotorConfig.SUPPLY_CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(
            DRIVE_ENABLE_CURRENT_LIMIT,
            DRIVE_CONTINUOUS_CURRENT_LIMIT,
            DRIVE_PEAK_CURRENT_LIMIT,
            DRIVE_PEAK_CURRENT_DURATION);
    driveMotorConfig.INVERTED = DRIVE_MOTOR_INVERTED;
    driveMotorConfig.NEUTRAL_MODE = DRIVE_NEUTRAL_MODE;
    driveMotorConfig.OPEN_LOOP_RAMP_RATE = OPEN_LOOP_RAMP;
    driveMotorConfig.CLOSED_LOOP_RAMP_RATE = CLOSED_LOOP_RAMP;
    driveMotorConfig.SLOT0_KP = driveKp.get();
    driveMotorConfig.SLOT0_KI = driveKi.get();
    driveMotorConfig.SLOT0_KD = driveKd.get();
    driveMotorConfig.SLOT0_KF = DRIVE_KF;
    driveMotorConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 20;

    mDriveMotor = TalonFXFactory.createTalon(driveMotorID, CAN_BUS_NAME, driveMotorConfig);

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
        Conversions.falconToDegrees(
            mDriveMotor.getSelectedSensorPosition(), SwerveModuleConstants.DRIVE_GEAR_RATIO);
    inputs.driveDistanceMeters =
        Conversions.falconToMeters(
            mDriveMotor.getSelectedSensorPosition(),
            SwerveModuleConstants.WHEEL_CIRCUMFERENCE,
            SwerveModuleConstants.DRIVE_GEAR_RATIO);
    inputs.driveVelocityMetersPerSec =
        Conversions.falconToMPS(
            mDriveMotor.getSelectedSensorVelocity(),
            SwerveModuleConstants.WHEEL_CIRCUMFERENCE,
            SwerveModuleConstants.DRIVE_GEAR_RATIO);
    inputs.driveAppliedPercentage = mDriveMotor.getMotorOutputPercent();
    inputs.driveCurrentAmps = new double[] {mDriveMotor.getStatorCurrent()};
    inputs.driveTempCelsius = new double[] {mDriveMotor.getTemperature()};

    inputs.angleAbsolutePositionDeg = angleEncoder.getAbsolutePosition();
    inputs.anglePositionDeg =
        Conversions.falconToDegrees(
            mAngleMotor.getSelectedSensorPosition(), SwerveModuleConstants.ANGLE_GEAR_RATIO);
    inputs.angleVelocityRevPerMin =
        Conversions.falconToRPM(
            mAngleMotor.getSelectedSensorVelocity(), SwerveModuleConstants.ANGLE_GEAR_RATIO);
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
    double ticksPerSecond =
        Conversions.mpsToFalcon(
            velocity,
            SwerveModuleConstants.WHEEL_CIRCUMFERENCE,
            SwerveModuleConstants.DRIVE_GEAR_RATIO);
    mDriveMotor.set(
        ControlMode.Velocity,
        ticksPerSecond,
        DemandType.ArbitraryFeedForward,
        calculateFeedforward(velocity));
  }

  /** Run the turn motor to the specified angle. */
  @Override
  public void setAnglePosition(double degrees) {
    mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(degrees, ANGLE_GEAR_RATIO));
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
