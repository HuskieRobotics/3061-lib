package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public class ArmConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ArmConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Arm";

  public static final int ANGLE_MOTOR_ID = 29;
  public static final int ANGLE_ENCODER_ID = 9;

  // PID constants are determined empirically through tuning
  public static final double ROTATION_KP = 50;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 10;

  // feed forward constants are determined through running SysId commands and analyzing the results
  // in SysId
  public static final double ROTATION_KS = 0.16206;
  public static final double ROTATION_KG = 0.22544;
  public static final double ROTATION_KV = 20.835;
  public static final double ROTATION_KA = 0.16507;

  // Motion magic constants are determined empirically through tuning
  public static final double ROTATION_EXPO_KV = 80.0;
  public static final double ROTATION_EXPO_KA = 24.0;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0; // don't limit the cruise velocity

  // current limits are determined based on current budget for the robot
  public static final double ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_LIMIT = 20;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 0.1;

  // the following are determined based on the mechanical design of the arm
  public static final boolean ANGLE_MOTOR_INVERTED = true;
  public static final double ANGLE_MOTOR_GEAR_RATIO = 45.0;
  public static final double SENSOR_TO_MECHANISM_RATIO = 4.0;
  public static final double MAGNET_OFFSET = -0.70727;
  public static final Angle LOWER_ANGLE_LIMIT = Degrees.of(14.7);
  public static final Angle UPPER_ANGLE_LIMIT = Degrees.of(135.0);
  public static final Angle ANGLE_TOLERANCE = Degrees.of(1.5);

  public static final double ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE = 1.0;
}
