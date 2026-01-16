package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;

/** Most of this is copied from Artemis; update with actual values */
public class CalypsoRobotConfig extends RobotConfig {

  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 25;
  private static final Angle FRONT_LEFT_MODULE_STEER_OFFSET = Rotations.of(-0.181152);

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24;
  private static final Angle FRONT_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.483643);

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 6;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 22;
  private static final Angle BACK_LEFT_MODULE_STEER_OFFSET = Rotations.of(0.124756);

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23;
  private static final Angle BACK_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.393066);

  private static final int GYRO_ID = 26;

  private static final Mass MASS = Pounds.of(146.5);
  private static final MomentOfInertia MOI = KilogramSquareMeters.of(6.985073979);
  private static final Distance TRACKWIDTH = Meters.of(0.57785);
  private static final Distance WHEELBASE = Meters.of(0.57785);
  private static final Distance WHEEL_RADIUS = Meters.of(0.050154);
  private static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.5;
  private static final Translation2d FRONT_RIGHT_CORNER_POSITION =
      new Translation2d(0.34925, 0.34925);

  private static final Distance ROBOT_WIDTH_WITH_BUMPERS =
      Meters.of(0.85725); // confirm with actual bumpers
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS =
      Meters.of(0.85725); // confirm with actual bumpers

  private static final double COUPLE_RATIO = 3.125; // possibly needs to be updated for mk4ns

  private static final double ANGLE_KP = 100.0;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.5;
  private static final double ANGLE_KS = 0.24055;
  private static final double ANGLE_KV = 2.2855;
  private static final double ANGLE_KA = 0.080209;

  private static final double DRIVE_KP = 14.0;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 5.4854;
  private static final double DRIVE_KV = 0.072502;
  private static final double DRIVE_KA = 0.43636;

  private static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.67);
  private static final LinearVelocity MAX_COAST_VELOCITY = MetersPerSecond.of(0.04);
  private static final double SLOW_MODE_MULTIPLIER = 0.7;

  private static final String CAN_BUS_NAME = "canbus1";
  private static final CANBus CAN_BUS = new CANBus(CAN_BUS_NAME);

  private static final String CAMERA_NAME_0 = "OV2311FR";
  private static final String CAMERA_NAME_1 = "OV2311BR";
  private static final String CAMERA_NAME_2 = "OV2311FL";
  private static final String CAMERA_NAME_3 = "OV2311BL";

  // Back cameras have positive pitches because they are on the opposite side of the robot,
  // so although they are pitched upward, robot relative that is downward

  // Front right camera
  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(
          new Translation3d(0.178, -0.268, 0.236),
          new Rotation3d(new Quaternion(-0.977, -0.032, 0.115, -0.177)));
  // pitch 45 degrees

  // Back right camera
  private static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          new Translation3d(-0.279, -0.273, 0.253),
          new Rotation3d(new Quaternion(-0.149, 0.161, 0.027, 0.975)));

  // Front left camera
  private static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(
          new Translation3d(0.178, 0.267, 0.240),
          new Rotation3d(new Quaternion(-0.976, 0.009, 0.119, 0.182)));

  // Back left camera
  private static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(
          new Translation3d(-0.276, 0.276, 0.253),
          new Rotation3d(new Quaternion(0.157, 0.163, -0.025, 0.974)));

  // default values for tunables
  private static final double AUTO_DRIVE_P_CONTROLLER = 5.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 5.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_X_KP = 2.0;
  private static final double DRIVE_TO_POSE_DRIVE_Y_KP = 4.0;
  private static final double DRIVE_TO_POSE_DRIVE_X_KD = 0.0;
  private static final double DRIVE_TO_POSE_DRIVE_Y_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP = 2.0;
  private static final double DRIVE_TO_POSE_THETA_KI = 0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final LinearVelocity DRIVE_TO_POSE_MAX_VELOCITY = MetersPerSecond.of(1.25);
  private static final LinearAcceleration DRIVE_TO_POSE_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(2.5);

  private static final LinearVelocity SQUARING_SPEED = MetersPerSecond.of(1.0);

  // Drive Facing Angle constants
  private static final double DRIVE_FACING_ANGLE_KP = 6.0;
  private static final double DRIVE_FACING_ANGLE_KD = 0.1;
  private static final double DRIVE_FACING_ANGLE_KI = 0.0;

  private static final int LED_COUNT = 42;

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
    return SwerveConstants.MK4N_L2_PLUS_CONSTANTS;
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
  public Angle[] getSwerveSteerOffsets() {
    return new Angle[] {
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
  public Distance getTrackwidth() {
    return TRACKWIDTH;
  }

  @Override
  public Distance getWheelbase() {
    return WHEELBASE;
  }

  @Override
  public Distance getWheelRadius() {
    return WHEEL_RADIUS;
  }

  @Override
  public Translation2d getFrontRightCornerPosition() {
    return FRONT_RIGHT_CORNER_POSITION;
  }

  @Override
  public Distance getRobotWidthWithBumpers() {
    return ROBOT_WIDTH_WITH_BUMPERS;
  }

  @Override
  public Distance getRobotLengthWithBumpers() {
    return ROBOT_LENGTH_WITH_BUMPERS;
  }

  @Override
  public CameraConfig[] getCameraConfigs() {

    Pose2d tagPose =
        new Pose2d(
            Units.inchesToMeters(144.003),
            Units.inchesToMeters(158.500),
            Rotation2d.fromDegrees(180));

    // FL and FR cameras are calibrated based on a centered AprilTag on the robot-to-camera
    // transform calibration jig
    Pose3d frontCenteredTagPose =
        new Pose3d(
            tagPose.transformBy(
                new Transform2d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    0,
                    Rotation2d.fromDegrees(180))));

    // the BR camera is calibrated based on an offset AprilTag on the robot-to-camera transform
    // calibration jig
    Pose3d backRightTagPose =
        new Pose3d(tagPose)
            .transformBy(
                new Transform3d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    Units.inchesToMeters(14.0),
                    -Units.inchesToMeters(1.0),
                    new Rotation3d()));

    // the BL camera is calibrated based on an offset AprilTag on the robot-to-camera transform
    // calibration jig
    Pose3d backLeftTagPose =
        new Pose3d(tagPose)
            .transformBy(
                new Transform3d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    -Units.inchesToMeters(14.0),
                    -Units.inchesToMeters(1.0),
                    new Rotation3d()));

    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_0)
          .poseForRobotToCameraTransformCalibration(frontCenteredTagPose)
          .id(CAMERA_NAME_0)
          .location("FR")
          .width(1600)
          .height(1200)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_1)
          .poseForRobotToCameraTransformCalibration(backRightTagPose)
          .id(CAMERA_NAME_1)
          .location("BR")
          .width(1600)
          .height(1200)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_2)
          .poseForRobotToCameraTransformCalibration(frontCenteredTagPose)
          .id(CAMERA_NAME_2)
          .location("FL")
          .width(1600)
          .height(1200)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_3)
          .poseForRobotToCameraTransformCalibration(backLeftTagPose)
          .id(CAMERA_NAME_3)
          .location("BL")
          .width(1600)
          .height(1200)
          .stdDevFactor(1.0)
          .build(),
    };
  }

  @Override
  public LinearVelocity getRobotMaxVelocity() {
    return MAX_VELOCITY;
  }

  @Override
  public double getRobotSlowModeMultiplier() {
    return SLOW_MODE_MULTIPLIER;
  }

  @Override
  public LinearVelocity getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY;
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
  public Mass getMass() {
    return MASS;
  }

  @Override
  public MomentOfInertia getMomentOfInertia() {
    return MOI;
  }

  @Override
  public double getWheelCOF() {
    return WHEEL_COEFFICIENT_OF_FRICTION;
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
    return DRIVE_TO_POSE_DRIVE_X_KP;
  }

  @Override
  public double getDriveToPoseDriveYKP() {
    return DRIVE_TO_POSE_DRIVE_Y_KP;
  }

  @Override
  public double getDriveToPoseDriveXKD() {
    return DRIVE_TO_POSE_DRIVE_X_KD;
  }

  @Override
  public double getDriveToPoseDriveYKD() {
    return DRIVE_TO_POSE_DRIVE_Y_KD;
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
  public LinearVelocity getDriveToPoseDriveMaxVelocity() {
    return DRIVE_TO_POSE_MAX_VELOCITY;
  }

  @Override
  public LinearAcceleration getDriveToPoseDriveMaxAcceleration() {
    return DRIVE_TO_POSE_MAX_ACCELERATION;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }

  @Override
  public LinearVelocity getMoveToPathFinalVelocity() {
    return SQUARING_SPEED;
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
