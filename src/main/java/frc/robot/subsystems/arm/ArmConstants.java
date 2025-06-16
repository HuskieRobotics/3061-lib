package frc.robot.subsystems.arm;

public class ArmConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Arm";

  public static final int ANGLE_MOTOR_ID = 29;
  public static final int ANGLE_ENCODER_ID = 9;

  // Shooter Rotation PID Constants
  public static final double ROTATION_KP = 50;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 10;
  public static final double ROTATION_KS = 0.16206;
  public static final double ROTATION_KG = 0.22544;
  public static final double ROTATION_KV = 20.835;
  public static final double ROTATION_KA = 0.16507;
  public static final double ROTATION_EXPO_KV = 80.0;
  public static final double ROTATION_EXPO_KA = 24.0;
  public static final double ROTATION_PID_PEAK_OUTPUT = 1.0;

  // Angle Motor
  public static final double ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_LIMIT = 20;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 0.1;
  public static final double ANGLE_MOTOR_STATOR_CURRENT_LIMIT_TIME = 0.5;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0;
  public static final double ANGLE_MOTOR_GEAR_RATIO = 45.0;
  public static final boolean ANGLE_MOTOR_INVERTED = true;
  public static final double MAGNET_OFFSET = -0.70727;
  public static final double SENSOR_TO_MECHANISM_RATIO = 4.0;
  public static final double ANGLE_TOLERANCE_DEGREES = 1.5;
  public static final double LOWER_ANGLE_LIMIT = 14.7;
  public static final double ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE = 1.0;
  public static final double UPPER_ANGLE_LIMIT = 135.0;
}
