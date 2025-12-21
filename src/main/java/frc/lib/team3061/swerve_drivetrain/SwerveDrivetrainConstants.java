package frc.lib.team3061.swerve_drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveDrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private SwerveDrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Drivetrain";

  public static final LinearVelocity DEMO_MODE_MAX_VELOCITY = MetersPerSecond.of(0.5);

  public static final Angle TILT_THRESHOLD = Degrees.of(5.0);
  public static final LinearVelocity UNTILT_VELOCITY = MetersPerSecond.of(0.5);

  public static final LinearVelocity SYSTEM_TEST_VELOCITY_TOLERANCE = MetersPerSecond.of(0.25);
  public static final Angle SYSTEM_TEST_ANGLE_TOLERANCE = Degrees.of(1.25);

  public enum SysIDCharacterizationMode {
    TRANSLATION_VOLTS,
    TRANSLATION_CURRENT,
    STEER_VOLTS,
    STEER_CURRENT,
    ROTATION_VOLTS
  }
}
