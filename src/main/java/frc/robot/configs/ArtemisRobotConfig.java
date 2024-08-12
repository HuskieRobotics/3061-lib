package frc.robot.configs;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class ArtemisRobotConfig extends RobotConfig {
  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 22;
  private static final double FRONT_LEFT_MODULE_STEER_OFFSET_ROT = 0.358398;

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 23;
  private static final double FRONT_RIGHT_MODULE_STEER_OFFSET_ROT = 0.024414;

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 24;
  private static final double BACK_LEFT_MODULE_STEER_OFFSET_ROT = -0.025635;

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 25;
  private static final double BACK_RIGHT_MODULE_STEER_OFFSET_ROT = -0.491699;

  private static final int GYRO_ID = 26;

  private static final double TRACKWIDTH_METERS = 0.57785; // 22.75
  private static final double WHEELBASE_METERS = 0.57785; // 22.75
  private static final double WHEEL_DIAMETER_METERS = 0.1027125;
  private static final double ROBOT_WIDTH_WITH_BUMPERS =
      0.88265; // meters //34.75 in , measure the actual bumpers
  private static final double ROBOT_LENGTH_WITH_BUMPERS = 0.88265; // meters // 34.75 in same above

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 100.0;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.05;

  private static final double ANGLE_KS = 0.24719;
  private static final double ANGLE_KV = 2.5845; // rps
  private static final double ANGLE_KA = 0.030892;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 10.0;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;

  private static final double DRIVE_KS = 0.23819;
  private static final double DRIVE_KV = 0.0;
  private static final double DRIVE_KA = 0.0;

  private static final double MAX_VELOCITY_METERS_PER_SECOND =
      4.5; // FIXME: confirm max velocity with real robot
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND =
      0.05; // FIXME: Values taken from nova, need to be updated
  private static final double SLOW_MODE_MULTIPLIER =
      0.75; // FIXME: Values taken from nova, need to be updated

  private static final double MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED =
      9.467; // from Choreo estimate
  private static final double MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED =
      33.436; // from Choreo estimate

  private static final String CAN_BUS_NAME = "canbus1";

  private static final String CAMERA_NAME_0 = "OV2311FR";
  private static final String CAMERA_NAME_1 = "OV2311BR";
  private static final String CAMERA_NAME_2 = "OV2311FL";
  private static final String CAMERA_NAME_3 = "OV2311BL";

  // Front right camera
  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(11.064),
              Units.inchesToMeters(-10.778),
              Units.inchesToMeters(8.189)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(0)));
  // pitch 45 degrees

  // Back right camera
  private static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.778),
              Units.inchesToMeters(-11.064),
              Units.inchesToMeters(8.189)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-90)));

  // Front left camera
  private static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(10.778),
              Units.inchesToMeters(11.064),
              Units.inchesToMeters(8.189)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(90)));

  // Back left camera
  private static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11.064),
              Units.inchesToMeters(10.778),
              Units.inchesToMeters(8.189)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));

  private static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 3.5;
  private static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 10;
  private static final double AUTO_DRIVE_P_CONTROLLER = 5.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 5.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP = 2.5;
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP = 4.5;
  private static final double DRIVE_TO_POSE_THETA_KI = 0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final double DRIVE_TO_POSE_DRIVE_TOLERANCE_METERS = 0.06;
  private static final double DRIVE_TO_POSE_THETA_TOLERANCE_RADIANS = 0.02;
  private static final double DRIVE_TO_POSE_MAX_VELOCITY = 1.25;

  private static final double SQUARING_SPEED_METERS_PER_SECOND = 1.0;

  // Drive Facing Angle constants
  private static final double DRIVE_FACING_ANGLE_KP = 6.0;
  private static final double DRIVE_FACING_ANGLE_KD = 0.1;
  private static final double DRIVE_FACING_ANGLE_KI = 0.0;

  private static final int LED_COUNT = 35;

  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

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
  public double getSwerveAngleKS() {
    return ANGLE_KS;
  }

  @Override
  public double getSwerveAngleKV() {
    return ANGLE_KV;
  }

  @Override
  public double getSwerveAngleKA() {
    return ANGLE_KA;
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
  public SwerveConstants getSwerveConstants() {
    return SwerveConstants.MK4I_L3_PLUS_CONSTANTS;
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
      FRONT_LEFT_MODULE_STEER_OFFSET_ROT,
      FRONT_RIGHT_MODULE_STEER_OFFSET_ROT,
      BACK_LEFT_MODULE_STEER_OFFSET_ROT,
      BACK_RIGHT_MODULE_STEER_OFFSET_ROT
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
  public double getWheelDiameterMeters() {
    return WHEEL_DIAMETER_METERS;
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
    return new Transform3d[] {
      ROBOT_TO_CAMERA_0, ROBOT_TO_CAMERA_1, ROBOT_TO_CAMERA_2, ROBOT_TO_CAMERA_3
    };
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
    return new String[] {CAMERA_NAME_0, CAMERA_NAME_1, CAMERA_NAME_2, CAMERA_NAME_3};
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
  public double getDriveToPoseThetaKI() {
    return DRIVE_TO_POSE_THETA_KI;
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
    return DRIVE_TO_POSE_MAX_VELOCITY;
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

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }

  @Override
  public double getMoveToPathFinalVelocity() {
    return SQUARING_SPEED_METERS_PER_SECOND;
  }

  @Override
  public double getDriveFacingAngleThetaKP() {
    return DRIVE_FACING_ANGLE_KP;
  }

  @Override
  public double getDriveFacingAngleThetaKI() {
    return DRIVE_FACING_ANGLE_KI;
  }

  @Override
  public double getDriveFacingAngleThetaKD() {
    return DRIVE_FACING_ANGLE_KD;
  }

  @Override
  public double getOdometryUpdateFrequency() {
    return 250.0;
  }

  @Override
  public Pigeon2Configuration getPigeonConfigForSwerveDrivetrain() {
    return new Pigeon2Configuration()
        .withMountPose(new MountPoseConfigs().withMountPoseRoll(-180.0));
  }

  @Override
  public LED_HARDWARE getLEDHardware() {
    return LED_HARDWARE.RIO;
  }

  @Override
  public int getLEDCount() {
    return LED_COUNT;
  }

  @Override
  public SWERVE_CONTROL_MODE getSwerveSteerControlMode() {
    return SWERVE_CONTROL_MODE.VOLTAGE;
  }

  @Override
  public SWERVE_CONTROL_MODE getSwerveDriveControlMode() {
    return SWERVE_CONTROL_MODE.TORQUE_CURRENT_FOC;
  }
}
