package frc.lib.team3061.swerve_drivetrain;

import edu.wpi.first.math.util.Units;

public class SwerveDrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private SwerveDrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Drivetrain";

  public static final double DEMO_MODE_MAX_VELOCITY_MPS = 0.5;

  public static final double TILT_THRESHOLD_DEG = 5.0;
  public static final double UNTILT_VELOCITY_MPS = 0.5;

  public static final double SYSTEM_TEST_VELOCITY_TOLERANCE_MPS = 0.25;
  public static final double SYSTEM_TEST_ANGLE_TOLERANCE_ROT = Units.degreesToRotations(10.0);

  public enum SysIDCharacterizationMode {
    TRANSLATION_VOLTS,
    TRANSLATION_CURRENT,
    STEER_VOLTS,
    STEER_CURRENT,
    ROTATION_VOLTS
  }
}
