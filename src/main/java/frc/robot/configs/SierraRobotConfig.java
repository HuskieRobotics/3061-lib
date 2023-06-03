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
public class SierraRobotConfig extends RobotConfig {

  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8;
  private static final double FRONT_LEFT_MODULE_STEER_OFFSET = 119.26;

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 14;
  private static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 102.44;

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 9;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
  private static final double BACK_LEFT_MODULE_STEER_OFFSET = 172.44;

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 17;
  private static final double BACK_RIGHT_MODULE_STEER_OFFSET = 40.78;

  private static final int GYRO_ID = 18;

  private static final double TRACKWIDTH_METERS = 0.5715; // 22.5 inches
  private static final double WHEELBASE_METERS = 0.5969; // 23.5 inches
  private static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89; // meters
  private static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91; // meters

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

  private static final double DRIVE_KS = 0.67841;
  private static final double DRIVE_KV = 2.27615;
  private static final double DRIVE_KA = 0.12872;

  private static final SwerveType SWERVE_TYPE = SwerveType.MK4;

  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.25;
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;
  private static final double MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED = 4.0;
  private static final double MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 4.0;
  private static final double SLOW_MODE_MULTIPLIER = 0.75;

  private static final String CAN_BUS_NAME = "";

  private static final String CAMERA_NAME = "OV9281";

  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(new Translation3d(0.254, 0, 0.648), new Rotation3d(0, 0, 0));

  private static final int PNEUMATICS_HUB_ID = 20;
  private static final int FLOW_SENSOR_CHANNEL = 0;
  private static final int REV_HIGH_PRESSURE_SENSOR_CHANNEL = 0;
  private static final int REV_LOW_PRESSURE_SENSOR_CHANNEL = 1;

  private static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 2.0;
  private static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
  private static final double AUTO_DRIVE_P_CONTROLLER = 3.5;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 10.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP = 2.5;
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP = 7.0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final double DRIVE_TO_POSE_DRIVE_TOLERANCE_METERS = 0.01;
  private static final double DRIVE_TO_POSE_THETA_TOLERANCE_RADIANS = 0.035;

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
  public Transform3d[] getRobotToCameraTransforms() {
    return new Transform3d[] {ROBOT_TO_CAMERA_0};
  }

  @Override
  public double getRobotMaxVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxDriveAcceleration() {
    return MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public double getRobotMaxTurnAcceleration() {
    return MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
  }

  @Override
  public double getRobotSlowModeMultiplier() {
    return SLOW_MODE_MULTIPLIER;
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
  public String[] getCameraNames() {
    return new String[] {CAMERA_NAME};
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

  @Override
  public double getDriveToPoseDriveKP() {
    return DRIVE_TO_POSE_DRIVE_KP;
  }

  @Override
  public double getDriveToPoseDriveKD() {
    return DRIVE_TO_POSE_DRIVE_KD;
  }

  @Override
  public double getDriveToPoseThetaKP() {
    return DRIVE_TO_POSE_THETA_KP;
  }

  @Override
  public double getDriveToPoseThetaKD() {
    return DRIVE_TO_POSE_THETA_KD;
  }

  @Override
  public double getDriveToPoseDriveMaxVelocity() {
    return getAutoMaxSpeed();
  }

  @Override
  public double getDriveToPoseDriveMaxAcceleration() {
    return getAutoMaxAcceleration();
  }

  @Override
  public double getDriveToPoseTurnMaxVelocity() {
    return getDriveToPoseDriveMaxVelocity()
        / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  @Override
  public double getDriveToPoseTurnMaxAcceleration() {
    return getDriveToPoseDriveMaxAcceleration()
        / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  @Override
  public double getDriveToPoseDriveTolerance() {
    return DRIVE_TO_POSE_DRIVE_TOLERANCE_METERS;
  }

  @Override
  public double getDriveToPoseThetaTolerance() {
    return DRIVE_TO_POSE_THETA_TOLERANCE_RADIANS;
  }
}
