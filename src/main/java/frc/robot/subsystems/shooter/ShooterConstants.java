package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int LEAD_MOTOR_ID = 27;
  public static final int FOLLOWER_A_MOTOR_ID = 28;
  public static final int FOLLOWER_B_MOTOR_ID = 29;
  public static final int GAME_PIECE_SENSOR_ID = 30;

  // PID constants are determined empirically through tuning
  public static final double KP = 5.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;

  // feed forward constants are determined through running SysId commands and analyzing the results
  // in SysId
  public static final double KS = 6.0;

  // current limits are determined based on current budget for the robot
  public static final double SHOOTER_PEAK_CURRENT_LIMIT = 80;

  // the following are determined based on the mechanical design of the shooter
  public static final double SHOOT_MOTORS_GEAR_RATIO = 1.0;
  public static final boolean IS_LEAD_INVERTED = false;
  public static final boolean IS_FOLLOWER_A_INVERTED = false;
  public static final boolean IS_FOLLOWER_B_INVERTED = true;
  public static final AngularVelocity SHOOTER_IDLE_VELOCITY = RotationsPerSecond.of(10.0);
  public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(5.0);

  // game piece detection constants
  public static final double DETECTOR_MIN_SIGNAL_STRENGTH = 2000;
  public static final double DETECTOR_PROXIMITY_THRESHOLD = 0.1;
}
