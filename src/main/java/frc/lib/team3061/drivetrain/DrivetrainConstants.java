package frc.lib.team3061.drivetrain;

public class DrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private DrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Drivetrain";

  public static final boolean ENABLE_TELEPORT_DETECTION = false;
  public static final double LEDS_FALLEN_ANGLE_DEGREES = 60.0; // Threshold to detect falls
}
