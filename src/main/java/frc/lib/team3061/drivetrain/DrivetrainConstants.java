package frc.lib.team3061.drivetrain;

public class DrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private DrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Drivetrain";

  public static final boolean ENABLE_TELEPORT_DETECTION = false;
  public static final double TELEPORT_DETECTION_THRESHOLD_METERS = 0.4;

  public static final double DEMO_MODE_MAX_VELOCITY = 0.5;

  public enum SysIDCharacterizationMode {
    TRANSLATION_VOLTS,
    TRANSLATION_CURRENT,
    STEER_VOLTS,
    STEER_CURRENT,
    ROTATION_VOLTS
  }
}
