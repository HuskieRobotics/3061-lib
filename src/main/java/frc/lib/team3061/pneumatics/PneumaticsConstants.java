package frc.lib.team3061.pneumatics;

public final class PneumaticsConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private PneumaticsConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final int MIN_LOW_PRESSURE = 50;
  public static final int MAX_LOW_PRESSURE = 60;
  public static final int MIN_HIGH_PRESSURE = 80;
  public static final int MAX_HIGH_PRESSURE = 120;
}
