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

  public static final double SKID_RATIO_THRESHOLD = 2.0;

  public static final double SYSTEM_TEST_VELOCITY_TOLERANCE_MPS = 0.25;
  public static final double SYSTEM_TEST_ANGLE_TOLERANCE_ROT = Units.degreesToRotations(10.0);

  /**
   * Duration, in seconds, of odometry samples that the queues fed by CTRE's odometry thread can
   * hold. The queues are drained once per iteration, so this must exceed the longest expected loop
   * overrun or samples will be discarded. At the default 250 Hz odometry frequency, 0.5 s is 125
   * samples.
   */
  public static final double ODOMETRY_QUEUE_CAPACITY_SECONDS = 0.5;

  /**
   * Multiplier applied to the robot's maximum velocity and maximum angular velocity when validating
   * the wheel distance and yaw deltas of an odometry sample. Wheel slip lets a wheel travel faster
   * than the chassis, so this must be greater than 1.0 to avoid rejecting valid samples.
   */
  public static final double ODOMETRY_MAX_DELTA_SCALAR = 3.0;

  /**
   * Minimum wheel distance delta, in meters, that is always considered valid regardless of the
   * elapsed time between samples. This keeps encoder quantization and timestamp jitter from
   * rejecting valid samples when the elapsed time is very small.
   */
  public static final double ODOMETRY_MIN_WHEEL_DELTA_METERS = 0.05;

  /**
   * Minimum change in the gyro's yaw, in degrees, that is always considered valid regardless of the
   * elapsed time between samples. The counterpart to ODOMETRY_MIN_WHEEL_DELTA_METERS: it keeps
   * timestamp jitter from rejecting valid samples when the elapsed time is very small, and is sized
   * so that the elapsed-time term takes over at roughly the nominal sample spacing. At 250 Hz and a
   * maximum angular velocity of 540 deg/s, that term is 6.5 degrees, and the robot can physically
   * rotate only 2.2 degrees between samples.
   */
  public static final double ODOMETRY_MIN_YAW_DELTA_DEG = 7.5;

  /**
   * Maximum distance, in meters, that the estimated pose may be outside of the field before it is
   * no longer constrained back onto the field. Small excursions are caused by wheel slip and are
   * worth correcting. Larger excursions indicate corrupt odometry; pinning the pose to the field
   * boundary in that case would destroy an otherwise recoverable estimate.
   */
  public static final double CONSTRAIN_POSE_TO_FIELD_MAX_ERROR_METERS = 1.0;

  /**
   * Distance, in meters, that the estimated pose must be outside of the field before it is
   * constrained back onto the field. Resetting the pose clears the pose estimator's vision
   * corrections, so this deadband avoids resetting the pose on every loop iteration while the robot
   * is pressed against a wall.
   */
  public static final double CONSTRAIN_POSE_TO_FIELD_DEADBAND_METERS = 0.05;

  public enum SysIDCharacterizationMode {
    TRANSLATION_VOLTS,
    TRANSLATION_CURRENT,
    STEER_VOLTS,
    STEER_CURRENT,
    ROTATION_VOLTS
  }
}
