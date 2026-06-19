package frc.robot.subsystems.shooter;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int TOP_SHOOTER_MOTOR_ID = 27;
  public static final int BOTTOM_SHOOTER_MOTOR_ID = 28;
  public static final int GAME_PIECE_SENSOR_ID = 29;

  // PID constants are determined empirically through tuning
  public static final double TOP_SHOOT_KP = 5.0;
  public static final double TOP_SHOOT_KI = 0.0;
  public static final double TOP_SHOOT_KD = 0.0;
  public static final double BOTTOM_SHOOT_KP = 5.0;
  public static final double BOTTOM_SHOOT_KI = 0.0;
  public static final double BOTTOM_SHOOT_KD = 0.0;

  // feed forward constants are determined through running SysId commands and analyzing the results
  // in SysId
  public static final double TOP_SHOOT_KS = 6.0;
  public static final double BOTTOM_SHOOT_KS = 7.0;

  // current limits are determined based on current budget for the robot
  public static final double SHOOT_MOTOR_TOP_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT = 60;
  public static final double SHOOT_MOTOR_TOP_PEAK_CURRENT_DURATION = 0.1;
  public static final double SHOOT_MOTOR_BOTTOM_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT = 60;
  public static final double SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_DURATION = 0.1;

  // the following are determined based on the mechanical design of the shooter
  public static final double SHOOT_MOTORS_GEAR_RATIO = 0.5;
  public static final boolean SHOOT_TOP_INVERTED = false;
  public static final boolean SHOOT_BOTTOM_INVERTED = false;
  public static final double SHOOTER_IDLE_VELOCITY_RPS = 10.0;
  public static final double VELOCITY_TOLERANCE_RPS = 5.0;

  // game piece detection constants
  public static final double DETECTOR_MIN_SIGNAL_STRENGTH = 2000;
  public static final double DETECTOR_PROXIMITY_THRESHOLD = 0.1;
}
