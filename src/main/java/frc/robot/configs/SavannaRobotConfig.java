package frc.robot.configs;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;
import frc.lib.team6328.util.FieldConstants;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;

public class SavannaRobotConfig extends RobotConfig {
  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 24;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 53;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 25;
  private static final double FRONT_LEFT_MODULE_STEER_OFFSET_ROT = -0.071045;

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 61;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 51;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24;
  private static final double FRONT_RIGHT_MODULE_STEER_OFFSET_ROT = 0.025879;

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 40;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 25;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 17;
  private static final double BACK_LEFT_MODULE_STEER_OFFSET_ROT = 0.469727;

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 35;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 52;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
  private static final double BACK_RIGHT_MODULE_STEER_OFFSET_ROT = -0.166748;

  private static final int GYRO_ID = 18;

  private static final double TRACKWIDTH_METERS = 0.628650;
  private static final double WHEELBASE_METERS = 0.476250;
  private static final double WHEEL_RADIUS_METERS = 0.0515507563;
  private static final Translation2d FRONT_RIGHT_CORNER_POSITION =
      new Translation2d(0.302940, -0.379140);

  private static final double ROBOT_WIDTH_WITH_BUMPERS_METERS = 0.933450;
  private static final double ROBOT_LENGTH_WITH_BUMPERS_METERS = 0.768350;

  private static final double COUPLE_RATIO = 3.857142857142857;

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 90;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.1;

  // values from sysid routines
  private static final double ANGLE_KS = 0.57576;
  private static final double ANGLE_KV = 2.6475;
  // 0.4399866667 * 2 * Math.PI; // convert from V/(radians/s) to V/(rotations/s)
  private static final double ANGLE_KA = 0.15068;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 16.0;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;

  // values from sysid routines
  private static final double DRIVE_KS = 2.8277;
  private static final double DRIVE_KV = 0.060631;
  private static final double DRIVE_KA = 0.51114;

  private static final double MAX_VELOCITY_MPS = 4.936;
  private static final double MAX_COAST_VELOCITY_MPS = 0.05;
  private static final double SLOW_MODE_MULTIPLIER = 0.3;
  private static final double MAX_ACCELERATION_WHEN_LIMITED_MPSPS = 9.0;
  private static final double MAX_ANGULAR_ACCELERATION_WHEN_LIMITED_RPSPS =
      100.0; // essentially disable angular acceleration limits

  private static final String CAN_BUS_NAME = "canbus1";
  private static final CANBus CAN_BUS = new CANBus(CAN_BUS_NAME);

  private static final double AUTO_DRIVE_P_CONTROLLER = 5.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 3.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP =
      6.0; // from preliminary testing, still needs more tuning
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP =
      6.0; // 18.0; // from preliminary testing, still needs more tuning
  private static final double DRIVE_TO_POSE_THETA_KI = 10.0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final double DRIVE_TO_POSE_DRIVE_TOLERANCE_METERS = 0.06;
  private static final double DRIVE_TO_POSE_THETA_TOLERANCE_RADIANS = 0.02;
  private static final double DRIVE_TO_POSE_MAX_VELOCITY_MPS = 1.25;

  private static final double SQUARING_SPEED_MPS = 1.0;

  // Drive Facing Angle constants
  private static final double DRIVE_FACING_ANGLE_KP = 5.0;
  private static final double DRIVE_FACING_ANGLE_KD = 0.1;
  private static final double DRIVE_FACING_ANGLE_KI = 0.0;

  private static final int LED_COUNT = 26;

  private static final String BR_CAMERA_SERIAL_NUMBER = "40708542";
  private static final String BL_CAMERA_SERIAL_NUMBER = "40708556";
  private static final String BCL_CAMERA_SERIAL_NUMBER = "24608727";
  private static final String BCH_CAMERA_SERIAL_NUMBER = "40777404";

  private static final int DAA1920_160UM_EXPOSURE = 2000;
  private static final int DAA1280_54UM_EXPOSURE = 800;
  private static final double MONO_GAIN = 10;
  private static final double DAA1920_160UM_DENOISE = 1.0;
  private static final double DAA1280_54UM_DENOISE = 0.0;

  // Back right camera
  // x, y, z, pitch, yaw
  private static final Transform3d ROBOT_TO_BR_CAMERA =
      new Transform3d(
          new Translation3d(-0.226, -0.311, 0.184),
          new Rotation3d(new Quaternion(-0.686, 0.160, 0.156, 0.692)));

  // Back left camera
  // x, y, z, pitch, yaw
  private static final Transform3d ROBOT_TO_BL_CAMERA =
      new Transform3d(
          new Translation3d(-0.219768, 0.306363, 0.202798),
          new Rotation3d(new Quaternion(-0.680614, -0.147614, 0.145496, -0.702325)));

  // Back center left camera
  // x, y, z, pitch, yaw
  // FIXME: update
  private static final Transform3d ROBOT_TO_BCL_CAMERA =
      new Transform3d(
          new Translation3d(-0.249052, 0.072636, 0.160716),
          new Rotation3d(new Quaternion(0.106805, 0.165791, -0.980344, 0.005602)));

  // Back center right camera
  // x, y, z, pitch, yaw
  // FIXME: update
  private static final Transform3d ROBOT_TO_BCH_CAMERA =
      new Transform3d(
          new Translation3d(-0.231423, 0.054115, 0.222039),
          new Rotation3d(new Quaternion(0.205042, 0.094613, -0.015936, 0.974039)));

  // use AprilTag ID 13 for empirical determination of the robot-to-camera transform
  private static final Pose3d ROBOT_TO_TAG_13_BACK_CAMERAS =
      FieldConstants.defaultAprilTagType
          .getLayout()
          .getTagPose(13)
          .get()
          .transformBy(
              new Transform3d(
                  Units.inchesToMeters(48.0),
                  Units.inchesToMeters(0.0),
                  -Units.inchesToMeters(21.75),
                  new Rotation3d()));

  private static final Pose3d ROBOT_TO_TAG_13_LEFT_CAMERA =
      FieldConstants.defaultAprilTagType
          .getLayout()
          .getTagPose(13)
          .get()
          .transformBy(
              new Transform3d(
                  Units.inchesToMeters(51.0),
                  Units.inchesToMeters(2.25),
                  -Units.inchesToMeters(21.75),
                  new Rotation3d(0, 0, Units.degreesToRadians(90))));

  private static final Pose3d ROBOT_TO_TAG_13_RIGHT_CAMERA =
      FieldConstants.defaultAprilTagType
          .getLayout()
          .getTagPose(13)
          .get()
          .transformBy(
              new Transform3d(
                  Units.inchesToMeters(51.0),
                  Units.inchesToMeters(-2.25),
                  -Units.inchesToMeters(21.75),
                  new Rotation3d(0, 0, Units.degreesToRadians(-90))));

  @Override
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BR_CAMERA)
          .poseForRobotToCameraTransformCalibration(ROBOT_TO_TAG_13_RIGHT_CAMERA)
          .id(BR_CAMERA_SERIAL_NUMBER)
          .location("BR")
          .width(1800)
          .height(1200)
          .exposure(DAA1920_160UM_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(DAA1920_160UM_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BL_CAMERA)
          .poseForRobotToCameraTransformCalibration(ROBOT_TO_TAG_13_LEFT_CAMERA)
          .id(BL_CAMERA_SERIAL_NUMBER)
          .location("BL")
          .width(1800)
          .height(1200)
          .exposure(DAA1920_160UM_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(DAA1920_160UM_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BCL_CAMERA)
          .poseForRobotToCameraTransformCalibration(ROBOT_TO_TAG_13_BACK_CAMERAS)
          .id(BCL_CAMERA_SERIAL_NUMBER)
          .location("BCL")
          .width(1280)
          .height(960)
          .exposure(DAA1280_54UM_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(DAA1280_54UM_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BCH_CAMERA)
          .poseForRobotToCameraTransformCalibration(ROBOT_TO_TAG_13_BACK_CAMERAS)
          .id(BCH_CAMERA_SERIAL_NUMBER)
          .location("BCH")
          .width(1800)
          .height(1200)
          .exposure(DAA1920_160UM_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(DAA1920_160UM_DENOISE)
          .stdDevFactor(1.0)
          .build(),
    };
  }

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
    return SwerveConstants.MK5N_R2_CONSTANTS;
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
  public double[] getSwerveSteerOffsetsRots() {
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
  public double getTrackwidthMeters() {
    return TRACKWIDTH_METERS;
  }

  @Override
  public double getWheelbaseMeters() {
    return WHEELBASE_METERS;
  }

  @Override
  public double getWheelRadiusMeters() {
    return WHEEL_RADIUS_METERS;
  }

  @Override
  public Translation2d getFrontRightCornerPosition() {
    return FRONT_RIGHT_CORNER_POSITION;
  }

  @Override
  public double getRobotWidthWithBumpersMeters() {
    return ROBOT_WIDTH_WITH_BUMPERS_METERS;
  }

  @Override
  public double getRobotLengthWithBumpersMeters() {
    return ROBOT_LENGTH_WITH_BUMPERS_METERS;
  }

  @Override
  public double getRobotMaxVelocityMPS() {
    return MAX_VELOCITY_MPS;
  }

  @Override
  public double getRobotSlowModeMultiplier() {
    return SLOW_MODE_MULTIPLIER;
  }

  @Override
  public double getRobotMaxAccelerationWhenLimitedMPSPS() {
    return MAX_ACCELERATION_WHEN_LIMITED_MPSPS;
  }

  @Override
  public double getRobotMaxAngularAccelerationWhenLimitedRPSPS() {
    return MAX_ANGULAR_ACCELERATION_WHEN_LIMITED_RPSPS;
  }

  @Override
  public double getRobotMaxCoastVelocityMPS() {
    return MAX_COAST_VELOCITY_MPS;
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
  public double getMassKG() {
    return 62.4143;
  }

  @Override
  public double getMomentOfInertiaKGMM() {
    return 6.0;
  }

  @Override
  public double getWheelCOF() {
    return 1.5;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public CANBus getCANBus() {
    return CAN_BUS;
  }

  @Override
  public double getDriveToPoseDriveXKP() {
    return DRIVE_TO_POSE_DRIVE_KP;
  }

  @Override
  public double getDriveToPoseDriveYKP() {
    return DRIVE_TO_POSE_DRIVE_KP;
  }

  @Override
  public double getDriveToPoseDriveXKD() {
    return DRIVE_TO_POSE_DRIVE_KD;
  }

  @Override
  public double getDriveToPoseDriveYKD() {
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
  public double getDriveToPoseDriveMaxVelocityMPS() {
    return DRIVE_TO_POSE_MAX_VELOCITY_MPS;
  }

  @Override
  public double getDriveToPoseDriveToleranceMeters() {
    return DRIVE_TO_POSE_DRIVE_TOLERANCE_METERS;
  }

  @Override
  public double getDriveToPoseThetaToleranceRad() {
    return DRIVE_TO_POSE_THETA_TOLERANCE_RADIANS;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }

  @Override
  public double getMoveToPathFinalVelocityMPS() {
    return SQUARING_SPEED_MPS;
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

  @Override
  public double getAzimuthSteerCouplingRatio() {
    return COUPLE_RATIO;
  }
}
