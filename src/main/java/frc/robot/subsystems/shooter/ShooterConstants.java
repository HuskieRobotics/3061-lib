package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int LEAD_MOTOR_ID = 20;
  public static final int FOLLOWER_A_MOTOR_ID = 21;
  public static final int FOLLOWER_B_MOTOR_ID = 22;
  public static final int HOOD_MOTOR_ID = 23;

  public static final int KICKER_MOTOR_ID = 19;

  public static final int GAME_PIECE_SENSOR_ID = 30;

  // PID constants are determined empirically through tuning
  public static final double KP = 10.0;
  public static final double KI = 0.0;
  public static final double KD = 0.0;

  // feed forward constants are determined through running SysId commands and analyzing the results
  // in SysId
  public static final double KS = 5.1; // same as 1:1 - prototype: 7.1914
  public static final double KV = 0.035; // 0.12928
  public static final double KA = 0.0; // 0.034191

  // current limits are determined based on current budget for the robot
  public static final double SHOOTER_PEAK_CURRENT_LIMIT = 80;

  // the following are determined based on the mechanical design of the shooter
  public static final double SHOOT_MOTORS_GEAR_RATIO = 1.5;
  public static final boolean IS_LEAD_INVERTED = true; // CW positive
  public static final boolean IS_FOLLOWER_A_INVERTED_FROM_LEAD = true; // CCW positive
  public static final boolean IS_FOLLOWER_B_INVERTED_FROM_LEAD = false; // CW positive
  public static final boolean IS_HOOD_INVERTED = false;
  public static final AngularVelocity SHOOTER_IDLE_VELOCITY = RotationsPerSecond.of(10.0);
  public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(5.0);

  // Hood motor constants
  public static final double HOOD_GEAR_RATIO = 144.0;
  public static final double HOOD_LENGTH_METERS = 0.2; // example length, replace with actual
  public static final double HOOD_MASS_KG = 2.0; // example mass, replace with actual

  // FIXME: confirm all these values
  // For theoretical maximum positions
  public static final Angle HOOD_MAX_ANGLE = Degrees.of(52.0);
  public static final Angle HOOD_STARTING_ANGLE = Degrees.of(20.0);
  // For soft limits
  public static final Angle UPPER_ANGLE_LIMIT = Degrees.of(50.0);
  public static final Angle LOWER_ANGLE_LIMIT = Degrees.of(22.0);

  // FIXME: change these values
  public static final double KP_HOOD = 400;
  public static final double KI_HOOD = 0.0;
  public static final double KD_HOOD = 0.0;
  public static final double KS_HOOD = 0.50489;
  public static final double KV_HOOD = 21.619;
  public static final double KA_HOOD = 0.83438;
  public static final double KG_HOOD = 0.0;

  public static final Distance HOOD_PULLY_CIRCUMFERENCE_METERS = Inches.of(2.0);

  public static final double HOOD_MOTOR_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double HOOD_MOTOR_PEAK_CURRENT_LIMIT = 20;
  public static final double HOOD_MOTOR_PEAK_CURRENT_DURATION = 0.1;

  public static final double HOOD_MOTOR_MANUAL_CONTROL_VOLTAGE = 1.0;

  //

  // PID constants are determined empirically through tuning
  public static final double KICKER_KP = 12;
  public static final double KICKER_KI = 0.0;
  public static final double KICKER_KD = 0.0;

  // feed forward constants are determined through running SysId commands and analyzing the results
  // in SysId
  public static final double KICKER_KS = 0.12132; // same as 1:1 - prototype: 7.1914
  public static final double KICKER_KV = 0.14996; // 0.12928
  public static final double KICKER_KA = 0.030084; // 0.034191

  // current limits are determined based on current budget for the robot
  public static final double KICKER_PEAK_CURRENT_LIMIT = 80;

  // the following are determined based on the mechanical design of the shooter
  public static final double KICKER_GEAR_RATIO = 1; // FIXME: find
  public static final boolean KICKER_MOTOR_INVERTED = true;

  // game piece detection constants
  public static final double DETECTOR_MIN_SIGNAL_STRENGTH = 2000;
  public static final double DETECTOR_PROXIMITY_THRESHOLD = 0.1;
}
