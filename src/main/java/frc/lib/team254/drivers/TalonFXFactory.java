/*
 * Initially from https://github.com/frc1678/C2022
 */

package frc.lib.team254.drivers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

@java.lang.SuppressWarnings({"java:S1104", "java:S116"})

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application.
 */
public class TalonFXFactory {

  private static final int TIMEOUT_MS = 100;

  // These periods don't share any common factors, so they shouldn't run at the same time. 255 is
  // max. (initially from https://github.com/Mechanical-Advantage/RobotCode2022)
  private static int[] kPrimePeriods =
      new int[] {255, 254, 253, 251, 247, 241, 239, 233, 229, 227, 223, 217, 211, 199, 197};

  public static class Configuration {
    public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;

    // factory default
    public double NEUTRAL_DEADBAND = 0.04;

    public boolean ENABLE_SOFT_LIMIT = false;
    public boolean ENABLE_LIMIT_SWITCH = false;
    public int FORWARD_SOFT_LIMIT = 0;
    public int REVERSE_SOFT_LIMIT = 0;

    public double PEAK_OUTPUT_FORWARD = 1.0;
    public double PEAK_OUTPUT_REVERSE = -1.0;

    public boolean INVERTED = false;
    public boolean SENSOR_PHASE = false;
    public SensorInitializationStrategy SENSOR_INITIALIZATION_STRATEGY =
        SensorInitializationStrategy.BootToZero;

    public int CONTROL_FRAME_PERIOD_MS = 20; // 10
    public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;

    public int GENERAL_STATUS_FRAME_RATE_MS = 10;
    public int FEEDBACK_STATUS_FRAME_RATE_MS = 49;
    public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = kPrimePeriods[0];
    public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = kPrimePeriods[1];
    public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = kPrimePeriods[2];
    public int MOTION_MAGIC_STATUS_FRAME_RATE_MS = kPrimePeriods[3];
    public int FEEDBACK_1_STATUS_FRAME_RATE_MS = kPrimePeriods[4];
    public int BASE_PIDF0_STATUS_FRAME_RATE_MS = kPrimePeriods[5];
    public int TURN_PIDF1_STATUS_FRAME_RATE_MS = kPrimePeriods[6];
    public int FEEDBACK_INTEGRATED_STATUS_FRAME_RATE_MS = kPrimePeriods[7];
    public int BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS = kPrimePeriods[8];

    public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD =
        SensorVelocityMeasPeriod.Period_100Ms;
    public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

    public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT =
        new StatorCurrentLimitConfiguration(false, 300, 700, 1);
    public SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(false, 40, 100, 1);

    public double OPEN_LOOP_RAMP_RATE = 0.0;
    public double CLOSED_LOOP_RAMP_RATE = 0.0;

    public double SLOT0_KP = 0.0;
    public double SLOT0_KI = 0.0;
    public double SLOT0_KD = 0.0;
    public double SLOT0_KF = 0.0;

    public int REMOTE_SENSOR_DEVICE_ID = 0;
    public RemoteSensorSource REMOTE_SENSOR_SOURCE = RemoteSensorSource.Off;

    public double MOTION_ACCELERATION = 0.0;
    public double MOTION_CRUISE_VELOCITY = 0.0;
    public int MOTION_CURVE_STRENGTH = 0;
  }

  private static final Configuration kDefaultConfiguration = new Configuration();
  private static final Configuration kFollowerConfiguration = new Configuration();

  static {
    // This control frame value seems to need to be something reasonable to avoid the Talon's
    // LEDs behaving erratically. Potentially try to increase as much as possible.
    kFollowerConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
    kFollowerConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
    kFollowerConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    kFollowerConfiguration.ENABLE_SOFT_LIMIT = false;
  }

  // create a CANTalon with the default (out of the box) configuration
  public static TalonFX createDefaultTalon(int id, String canBusName) {
    return createTalon(id, canBusName, kDefaultConfiguration);
  }

  public static TalonFX createPermanentFollowerTalon(int id, String canBusName, int leaderID) {
    final TalonFX talon = createTalon(id, canBusName, kFollowerConfiguration);
    talon.set(ControlMode.Follower, leaderID);
    return talon;
  }

  public static TalonFX createTalon(int id, String canBusName, Configuration config) {
    TalonFX talon = new TalonFX(id, canBusName);
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talon.configFactoryDefault();

    talon.set(ControlMode.PercentOutput, 0.0);

    talonFXConfig.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    talonFXConfig.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
    talonFXConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    talonFXConfig.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;

    talonFXConfig.clearPositionOnLimitF = false;
    talonFXConfig.clearPositionOnLimitR = false;

    talonFXConfig.nominalOutputForward = 0.0;
    talonFXConfig.nominalOutputReverse = 0.0;
    talonFXConfig.neutralDeadband = config.NEUTRAL_DEADBAND;

    talonFXConfig.peakOutputForward = config.PEAK_OUTPUT_FORWARD;
    talonFXConfig.peakOutputReverse = config.PEAK_OUTPUT_REVERSE;

    talonFXConfig.forwardSoftLimitThreshold = config.FORWARD_SOFT_LIMIT;
    talonFXConfig.forwardSoftLimitEnable = config.ENABLE_SOFT_LIMIT;
    talonFXConfig.reverseSoftLimitThreshold = config.REVERSE_SOFT_LIMIT;
    talonFXConfig.reverseSoftLimitEnable = config.ENABLE_SOFT_LIMIT;

    talonFXConfig.initializationStrategy = config.SENSOR_INITIALIZATION_STRATEGY;

    talonFXConfig.velocityMeasurementPeriod = config.VELOCITY_MEASUREMENT_PERIOD;
    talonFXConfig.velocityMeasurementWindow = config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

    talonFXConfig.openloopRamp = config.OPEN_LOOP_RAMP_RATE;
    talonFXConfig.closedloopRamp = config.CLOSED_LOOP_RAMP_RATE;

    talonFXConfig.voltageCompSaturation = 0.0;
    talonFXConfig.voltageMeasurementFilter = 32;

    talonFXConfig.statorCurrLimit = config.STATOR_CURRENT_LIMIT;
    talonFXConfig.supplyCurrLimit = config.SUPPLY_CURRENT_LIMIT;

    talonFXConfig.slot0.kP = config.SLOT0_KP;
    talonFXConfig.slot0.kI = config.SLOT0_KI;
    talonFXConfig.slot0.kD = config.SLOT0_KD;
    talonFXConfig.slot0.kF = config.SLOT0_KF;

    talonFXConfig.motionAcceleration = config.MOTION_ACCELERATION;
    talonFXConfig.motionCruiseVelocity = config.MOTION_CRUISE_VELOCITY;
    talonFXConfig.motionCurveStrength = config.MOTION_CURVE_STRENGTH;

    talonFXConfig.remoteFilter0.remoteSensorDeviceID = config.REMOTE_SENSOR_DEVICE_ID;
    talonFXConfig.remoteFilter0.remoteSensorSource = config.REMOTE_SENSOR_SOURCE;

    talon.configAllSettings(talonFXConfig, TIMEOUT_MS);

    talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
    talon.clearMotionProfileHasUnderrun(TIMEOUT_MS);
    talon.clearMotionProfileTrajectories();

    talon.clearStickyFaults(TIMEOUT_MS);

    talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

    talon.setNeutralMode(config.NEUTRAL_MODE);

    talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

    talon.setSensorPhase(config.SENSOR_PHASE);
    talon.setInverted(config.INVERTED);

    talon.selectProfileSlot(0, 0);

    talon.enableVoltageCompensation(false);

    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General, config.GENERAL_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0, config.FEEDBACK_STATUS_FRAME_RATE_MS, TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_3_Quadrature,
        config.QUAD_ENCODER_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_4_AinTempVbat,
        config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_8_PulseWidth,
        config.PULSE_WIDTH_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic,
        config.MOTION_MAGIC_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_12_Feedback1,
        config.FEEDBACK_1_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0,
        config.BASE_PIDF0_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_14_Turn_PIDF1,
        config.TURN_PIDF1_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_21_FeedbackIntegrated,
        config.FEEDBACK_INTEGRATED_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_Brushless_Current,
        config.BRUSHLESS_CURRENT_STATUS_FRAME_RATE_MS,
        TIMEOUT_MS);

    talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

    return talon;
  }

  private TalonFXFactory() {}
}
