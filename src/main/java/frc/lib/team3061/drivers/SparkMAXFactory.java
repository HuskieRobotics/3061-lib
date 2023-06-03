package frc.lib.team3061.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

@java.lang.SuppressWarnings({"java:S1104", "java:S116"})

/**
 * Creates SparkMAX objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application.
 */
public class SparkMAXFactory {

  // These periods don't share any common factors, so they shouldn't run at the same time. 255 is
  // max. (initially from https://github.com/Mechanical-Advantage/RobotCode2022)
  private static int[] kPrimePeriods =
      new int[] {255, 254, 253, 251, 247, 241, 239, 233, 229, 227, 223, 217, 211, 199, 197};

  public static class Configuration {
    public boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public double VOLTAGE_COMPENSATION = 12.0;

    public boolean ENABLE_SOFT_LIMIT = false;
    public int FORWARD_SOFT_LIMIT = 0;
    public int REVERSE_SOFT_LIMIT = 0;

    public double CLOSED_LOOP_RAMP_RATE = 0.0;
    public double OPEN_LOOP_RAMP_RATE = 0.0;

    public CANSparkMax.IdleMode IDLE_MODE = CANSparkMax.IdleMode.kCoast;

    public boolean INVERTED = false;

    public int SMART_FREE_CURRENT_LIMIT = 30;
    public int SMART_STALL_CURRENT_LIMIT = 20;

    public int CAN_TIMEOUT_MS = 0;

    public int STATUS0_FRAME_RATE_MS = kPrimePeriods[0];
    public int STATUS1_FRAME_RATE_MS = kPrimePeriods[1];
    public int STATUS2_FRAME_RATE_MS = kPrimePeriods[2];
    public int STATUS3_FRAME_RATE_MS = kPrimePeriods[3];
    public int STATUS4_FRAME_RATE_MS = kPrimePeriods[4];
    public int STATUS5_FRAME_RATE_MS = kPrimePeriods[5];
    public int STATUS6_FRAME_RATE_MS = kPrimePeriods[6];
    public int CONTROL_FRAME_RATE_MS = 10;
  }

  private static final Configuration kDefaultConfiguration = new Configuration();
  private static final Configuration kFollowerConfiguration = new Configuration();

  // create a CANTalon with the default (out of the box) configuration
  public static CANSparkMax createDefaultSparkMax(int id) {
    return createSparkMax(id, kDefaultConfiguration);
  }

  public static CANSparkMax createPermanentFollowerTalon(int id, CANSparkMax leader) {
    final CANSparkMax sparkMax = createSparkMax(id, kFollowerConfiguration);
    sparkMax.follow(leader);
    return sparkMax;
  }

  public static CANSparkMax createSparkMax(int id, Configuration config) {
    CANSparkMax sparkMax = new CANSparkMax(id, MotorType.kBrushless);

    sparkMax.restoreFactoryDefaults();

    sparkMax.setCANTimeout(config.CAN_TIMEOUT_MS);

    if (config.ENABLE_VOLTAGE_COMPENSATION) {
      sparkMax.enableVoltageCompensation(config.VOLTAGE_COMPENSATION);
    }

    if (config.ENABLE_SOFT_LIMIT) {
      sparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, config.FORWARD_SOFT_LIMIT);
      sparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, config.REVERSE_SOFT_LIMIT);
    }

    sparkMax.setClosedLoopRampRate(config.CLOSED_LOOP_RAMP_RATE);
    sparkMax.setOpenLoopRampRate(config.OPEN_LOOP_RAMP_RATE);

    sparkMax.setIdleMode(config.IDLE_MODE);

    sparkMax.setInverted(config.INVERTED);

    sparkMax.setSmartCurrentLimit(
        config.SMART_STALL_CURRENT_LIMIT, config.SMART_FREE_CURRENT_LIMIT);

    sparkMax.clearFaults();

    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.STATUS0_FRAME_RATE_MS);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.STATUS1_FRAME_RATE_MS);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.STATUS2_FRAME_RATE_MS);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, config.STATUS3_FRAME_RATE_MS);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, config.STATUS4_FRAME_RATE_MS);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, config.STATUS5_FRAME_RATE_MS);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, config.STATUS6_FRAME_RATE_MS);

    sparkMax.setControlFramePeriodMs(config.CONTROL_FRAME_RATE_MS);

    // invoke set last which updates control frame period
    sparkMax.set(0.0);

    return sparkMax;
  }

  private SparkMAXFactory() {}
}
