package frc.robot;

import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.SwerveModuleConstants;

public class SierraRobotConfig implements RobotConfig {

  // FIXME: tune PID values for the angle and drive motors

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 0.6;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 12.0;
  private static final double ANGLE_KF = 0.0;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 0.10;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KF = 0.0;

  // FIXME: characterize the drivetrain and update these constants

  /* Drive Motor Characterization Values */
  // divide by 12 to convert from volts to percent output for CTRE
  public static final double DRIVE_KS = (0.55493 / 12);
  public static final double DRIVE_KV = (2.3014 / 12);
  public static final double DRIVE_KA = (0.12872 / 12);

  // FIXME: update all CAN IDs
  // FIXME: update all steer offsets
  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 118.0371;

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 102.9968;

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 9;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -189.7051;

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 17;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 40.3335;

  public static final int PIGEON_ID = 18;

  // FIXME: update robot dimensions

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * <p>Should be measured from center to center.
   */
  public static final double TRACKWIDTH_METERS = 0.5715; // 22.5 inches

  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   */
  public static final double WHEELBASE_METERS = 0.5969; // 23.5 inches

  public static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89; // meters
  public static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91; // meters

  // FIXME: determine maximum velocities empirically

  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      6380.0
          / 60.0
          / SwerveModuleConstants.DRIVE_GEAR_RATIO
          * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;

  // Swerve Module PID accessors
  public double getSwerveAngleKP() {
    return ANGLE_KP;
  }

  public double getSwerveAngleKI() {
    return ANGLE_KI;
  }

  public double getSwerveAngleKD() {
    return ANGLE_KD;
  }

  public double getSwerveAngleKF() {
    return ANGLE_KF;
  }

  public double getSwerveDriveKP() {
    return DRIVE_KP;
  }

  public double getSwerveDriveKI() {
    return DRIVE_KI;
  }

  public double getSwerveDriveKD() {
    return DRIVE_KD;
  }

  public double getSwerveDriveKF() {
    return DRIVE_KF;
  }

  // Drive Characterization accessors
  public double getDriveKS() {
    return DRIVE_KS;
  }

  public double getDriveKV() {
    return DRIVE_KV;
  }

  public double getDriveKA() {
    return DRIVE_KA;
  }

  // Swerve Module CAN IDs (FL, FR, BL, BR)
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_DRIVE_MOTOR,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      BACK_LEFT_MODULE_DRIVE_MOTOR,
      BACK_RIGHT_MODULE_DRIVE_MOTOR
    };
  }

  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_MOTOR,
      FRONT_RIGHT_MODULE_STEER_MOTOR,
      BACK_LEFT_MODULE_STEER_MOTOR,
      BACK_RIGHT_MODULE_STEER_MOTOR
    };
  }

  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_ENCODER,
      FRONT_RIGHT_MODULE_STEER_ENCODER,
      BACK_LEFT_MODULE_STEER_ENCODER,
      BACK_RIGHT_MODULE_STEER_ENCODER
    };
  }

  public double[] getSwerveSteerOffsets() {
    return new double[] {
      FRONT_LEFT_MODULE_STEER_OFFSET,
      FRONT_RIGHT_MODULE_STEER_OFFSET,
      BACK_LEFT_MODULE_STEER_OFFSET,
      BACK_RIGHT_MODULE_STEER_OFFSET
    };
  }

  public int getPigeonCANID() {
    return PIGEON_ID;
  }

  // robot dimensions accessors
  public double getTrackwidth() {
    return TRACKWIDTH_METERS;
  }

  public double getWheelbase() {
    return WHEELBASE_METERS;
  }

  public double getRobotWidthWithBumpers() {
    return ROBOT_WIDTH_WITH_BUMPERS;
  }

  public double getRobotLengthWithBumpers() {
    return ROBOT_LENGTH_WITH_BUMPERS;
  }

  public double getRobotMaxVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }
}
