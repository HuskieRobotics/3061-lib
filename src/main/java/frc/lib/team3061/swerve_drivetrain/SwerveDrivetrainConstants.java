package frc.lib.team3061.swerve_drivetrain;

public class SwerveDrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private SwerveDrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "SwerveDrivetrain";

  public static final boolean ENABLE_TELEPORT_DETECTION = false;
  public static final double TELEPORT_DETECTION_THRESHOLD_METERS = 0.4;

  public static final double DEMO_MODE_MAX_VELOCITY = 0.5;

  public static final double TILT_THRESHOLD_DEG = 5.0;
  public static final double UNTILT_VELOCITY_MPS = 0.5;

  public static final double SYSTEM_TEST_VELOCITY_TOLERANCE = 0.25;
  public static final double SYSTEM_TEST_ANGLE_TOLERANCE_DEG = 1.25;

  public enum SysIDCharacterizationMode {
    TRANSLATION_VOLTS,
    TRANSLATION_CURRENT,
    STEER_VOLTS,
    STEER_CURRENT,
    ROTATION_VOLTS
  }
}
