/*
 * Initially from https://github.com/frc1678/C2022
 */

package frc.lib.team254.drivers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

@java.lang.SuppressWarnings({"java:S1104", "java:S116"})

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application.
 */
public class TalonSRXFactory {

  private static final int TIMEOUT_MS = 100;

  public static class Configuration {
    public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
    // factory default
    public double NEUTRAL_DEADBAND = 0.04;

    public boolean ENABLE_SOFT_LIMIT = false;
    public boolean ENABLE_LIMIT_SWITCH = false;
    public int FORWARD_SOFT_LIMIT = 0;
    public int REVERSE_SOFT_LIMIT = 0;

    public boolean INVERTED = false;
    public boolean SENSOR_PHASE = false;

    public int CONTROL_FRAME_PERIOD_MS = 5;
    public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
    public int GENERAL_STATUS_FRAME_RATE_MS = 5;
    public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
    public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

    public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD =
        SensorVelocityMeasPeriod.Period_100Ms;
    public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

    public SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT =
        new SupplyCurrentLimitConfiguration(false, 0, 40, 0.2);

    public double OPEN_LOOP_RAMP_RATE = 0.0;
    public double CLOSED_LOOP_RAMP_RATE = 0.0;

    public double SLOT0_KP = 0.0;
    public double SLOT0_KI = 0.0;
    public double SLOT0_KD = 0.0;
    public double SLOT0_KF = 0.0;
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
  public static TalonSRX createDefaultTalon(int id) {
    return createTalon(id, kDefaultConfiguration);
  }

  public static TalonSRX createPermanentFollowerTalon(int id, int leaderID) {
    final TalonSRX talon = createTalon(id, kFollowerConfiguration);
    talon.set(ControlMode.Follower, leaderID);
    return talon;
  }

  public static TalonSRX createTalon(int id, Configuration config) {
    TalonSRX talon = new LazyTalonSRX(id);
    TalonSRXConfiguration talonSRXConfig = new TalonSRXConfiguration();

    talon.configFactoryDefault();

    talon.set(ControlMode.PercentOutput, 0.0);

    talonSRXConfig.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    talonSRXConfig.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
    talonSRXConfig.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
    talonSRXConfig.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;

    talonSRXConfig.clearPositionOnLimitF = false;
    talonSRXConfig.clearPositionOnLimitR = false;

    talonSRXConfig.nominalOutputForward = 0.0;
    talonSRXConfig.nominalOutputReverse = 0.0;
    talonSRXConfig.neutralDeadband = config.NEUTRAL_DEADBAND;

    talonSRXConfig.peakOutputForward = 1.0;
    talonSRXConfig.peakOutputReverse = -1.0;

    talonSRXConfig.forwardSoftLimitThreshold = config.FORWARD_SOFT_LIMIT;
    talonSRXConfig.forwardSoftLimitEnable = config.ENABLE_SOFT_LIMIT;
    talonSRXConfig.reverseSoftLimitThreshold = config.REVERSE_SOFT_LIMIT;
    talonSRXConfig.reverseSoftLimitEnable = config.ENABLE_SOFT_LIMIT;

    talonSRXConfig.velocityMeasurementPeriod = config.VELOCITY_MEASUREMENT_PERIOD;
    talonSRXConfig.velocityMeasurementWindow = config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW;

    talonSRXConfig.openloopRamp = config.OPEN_LOOP_RAMP_RATE;
    talonSRXConfig.closedloopRamp = config.CLOSED_LOOP_RAMP_RATE;

    talonSRXConfig.voltageCompSaturation = 0.0;
    talonSRXConfig.voltageMeasurementFilter = 32;

    talonSRXConfig.slot0.kP = config.SLOT0_KP;
    talonSRXConfig.slot0.kI = config.SLOT0_KI;
    talonSRXConfig.slot0.kD = config.SLOT0_KD;
    talonSRXConfig.slot0.kF = config.SLOT0_KF;

    talon.configAllSettings(talonSRXConfig, TIMEOUT_MS);

    talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
    talon.clearMotionProfileHasUnderrun(TIMEOUT_MS);
    talon.clearMotionProfileTrajectories();

    talon.clearStickyFaults(TIMEOUT_MS);

    talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

    talon.setNeutralMode(config.NEUTRAL_MODE);

    talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

    talon.setInverted(config.INVERTED);
    talon.setSensorPhase(config.SENSOR_PHASE);

    talon.selectProfileSlot(0, 0);

    talon.enableVoltageCompensation(false);

    talon.configSupplyCurrentLimit(config.SUPPLY_CURRENT_LIMIT);

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

    talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

    return talon;
  }

  private TalonSRXFactory() {}
}
