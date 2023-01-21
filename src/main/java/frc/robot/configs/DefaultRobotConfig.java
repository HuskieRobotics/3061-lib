package frc.robot.configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.SwerveModuleConstants.SwerveType;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class DefaultRobotConfig extends RobotConfig {

  // FIXME: update all CAN IDs and steer offsets
  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8;
  private static final double FRONT_LEFT_MODULE_STEER_OFFSET = 118.0371;

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14;
  private static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 102.9968;

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 9;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
  private static final double BACK_LEFT_MODULE_STEER_OFFSET = -189.7051;

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 17;
  private static final double BACK_RIGHT_MODULE_STEER_OFFSET = 40.3335;

  private static final int GYRO_ID = 18;

  // FIXME: update robot dimensions
  private static final double TRACKWIDTH_METERS = 0.5715; // 22.5 inches
  private static final double WHEELBASE_METERS = 0.5969; // 23.5 inches
  private static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89; // meters
  private static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91; // meters

  // FIXME: tune PID values for the angle and drive motors for the swerve modules

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
  private static final double DRIVE_KS = 0.55493;
  private static final double DRIVE_KV = 2.3014;
  private static final double DRIVE_KA = 0.12872;

  // FIXME: specify the type of swerve module (MK4 and MK4i are supported)
  private static final SwerveType SWERVE_TYPE = SwerveType.MK4I;

  // FIXME: determine maximum velocities empirically
  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.25;
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  // FIXME: specify the name of the CANivore CAN FD bus as appropriate (an empty string uses the
  // default CAN bus)
  private static final String CAN_BUS_NAME = "";

  // FIXME: specify the name of the camera used for detecting AprilTags
  private static final String CAMERA_NAME = "ov9268";

  // FIXME: update this with the actual transform from the robot to the camera
  private static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  // FIXME: specify the configuration for pneumatics
  private static final int PNEUMATICS_HUB_ID = 20;
  private static final int FLOW_SENSOR_CHANNEL = 0;
  private static final int REV_HIGH_PRESSURE_SENSOR_CHANNEL = 0;
  private static final int REV_LOW_PRESSURE_SENSOR_CHANNEL = 1;

  // FIXME: specify maximum velocity and acceleration and tune PID values for auto paths

  private static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 2.0;
  private static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
  private static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 10.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  @Override
  public double getSwerveAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public double getSwerveAngleKI() {
    return ANGLE_KI;
  }

  @Override
  public double getSwerveAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public double getSwerveAngleKF() {
    return ANGLE_KF;
  }

  @Override
  public double getSwerveDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public double getSwerveDriveKI() {
    return DRIVE_KI;
  }

  @Override
  public double getSwerveDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public double getSwerveDriveKF() {
    return DRIVE_KF;
  }

  @Override
  public double getDriveKS() {
    return DRIVE_KS;
  }

  @Override
  public double getDriveKV() {
    return DRIVE_KV;
  }

  @Override
  public double getDriveKA() {
    return DRIVE_KA;
  }

  @Override
  public SwerveType getSwerveType() {
    return SWERVE_TYPE;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_DRIVE_MOTOR,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      BACK_LEFT_MODULE_DRIVE_MOTOR,
      BACK_RIGHT_MODULE_DRIVE_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_MOTOR,
      FRONT_RIGHT_MODULE_STEER_MOTOR,
      BACK_LEFT_MODULE_STEER_MOTOR,
      BACK_RIGHT_MODULE_STEER_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_ENCODER,
      FRONT_RIGHT_MODULE_STEER_ENCODER,
      BACK_LEFT_MODULE_STEER_ENCODER,
      BACK_RIGHT_MODULE_STEER_ENCODER
    };
  }

  @Override
  public double[] getSwerveSteerOffsets() {
    return new double[] {
      FRONT_LEFT_MODULE_STEER_OFFSET,
      FRONT_RIGHT_MODULE_STEER_OFFSET,
      BACK_LEFT_MODULE_STEER_OFFSET,
      BACK_RIGHT_MODULE_STEER_OFFSET
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_ID;
  }

  @Override
  public double getTrackwidth() {
    return TRACKWIDTH_METERS;
  }

  @Override
  public double getWheelbase() {
    return WHEELBASE_METERS;
  }

  @Override
  public double getRobotWidthWithBumpers() {
    return ROBOT_WIDTH_WITH_BUMPERS;
  }

  @Override
  public double getRobotLengthWithBumpers() {
    return ROBOT_LENGTH_WITH_BUMPERS;
  }

  @Override
  public Transform3d getRobotToCameraTransform() {
    return ROBOT_TO_CAMERA;
  }

  @Override
  public double getRobotMaxVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getAutoMaxSpeed() {
    return AUTO_MAX_SPEED_METERS_PER_SECOND;
  }

  @Override
  public double getAutoMaxAcceleration() {
    return AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public double getAutoDriveKP() {
    return AUTO_DRIVE_P_CONTROLLER;
  }

  @Override
  public double getAutoDriveKI() {
    return AUTO_DRIVE_I_CONTROLLER;
  }

  @Override
  public double getAutoDriveKD() {
    return AUTO_DRIVE_D_CONTROLLER;
  }

  @Override
  public double getAutoTurnKP() {
    return AUTO_TURN_P_CONTROLLER;
  }

  @Override
  public double getAutoTurnKI() {
    return AUTO_TURN_I_CONTROLLER;
  }

  @Override
  public double getAutoTurnKD() {
    return AUTO_TURN_D_CONTROLLER;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public String getCameraName() {
    return CAMERA_NAME;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return PNEUMATICS_HUB_ID;
  }

  @Override
  public int getFlowSensorChannel() {
    return FLOW_SENSOR_CHANNEL;
  }

  @Override
  public int getRevHighPressureSensorChannel() {
    return REV_HIGH_PRESSURE_SENSOR_CHANNEL;
  }

  @Override
  public int getRevLowPressureSensorChannel() {
    return REV_LOW_PRESSURE_SENSOR_CHANNEL;
  }
}
