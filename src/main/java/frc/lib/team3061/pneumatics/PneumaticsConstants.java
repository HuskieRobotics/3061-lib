package frc.lib.team3061.pneumatics;

public final class PneumaticsConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private PneumaticsConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final int PNEUMATICS_HUB_ID = 20;
  public static final int FLOW_SENSOR_CHANNEL = 0;
  public static final int REV_HIGH_PRESSURE_SENSOR_CHANNEL = 0;
  public static final int REV_LOW_PRESSURE_SENSOR_CHANNEL = 1;

  public static final int MIN_LOW_PRESSURE = 50;
  public static final int MAX_LOW_PRESSURE = 60;
  public static final int MIN_HIGH_PRESSURE = 80;
  public static final int MAX_HIGH_PRESSURE = 120;
}
