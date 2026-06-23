package frc.lib.team3061.util;

public class MathUtils {

  private MathUtils() {
    throw new IllegalStateException("utility class");
  }

  public static boolean isNear(double a, double b, double tolerance) {
    return Math.abs(a - b) <= tolerance;
  }
}
