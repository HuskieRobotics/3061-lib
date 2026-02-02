package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;

public class NewPracticeRobotConfig extends RobotConfig {
  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 37;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 61;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
  private static final Angle FRONT_LEFT_MODULE_STEER_OFFSET = Rotations.of(-0.117676 + 0.5);

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 40;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 25;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
  private static final Angle FRONT_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.028809 + 0.5);

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 39;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 60;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;
  private static final Angle BACK_LEFT_MODULE_STEER_OFFSET = Rotations.of(-0.378906 + 0.5);

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 38;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 59;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
  private static final Angle BACK_RIGHT_MODULE_STEER_OFFSET = Rotations.of(0.157959 - 0.5);

  private static final int GYRO_ID = 3;

  private static final Distance TRACKWIDTH = Meters.of(0.57785); // 22.75
  private static final Distance WHEELBASE = Meters.of(0.57785); // 22.75
  private static final Distance WHEEL_RADIUS = Meters.of(0.0508);
  private static final Translation2d FRONT_RIGHT_CORNER_POSITION = new Translation2d(0.36, -0.36);

  private static final Distance ROBOT_WIDTH_WITH_BUMPERS = Meters.of(0.88026); // 34.656 in
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS = Meters.of(0.88026); // 34.656 in

  private static final double COUPLE_RATIO = 3.857142857142857;

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 77;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.1;

  // values from sysid routines
  private static final double ANGLE_KS = 0.39776;
  private static final double ANGLE_KV = 2.6176;
  // 0.4399866667 * 2 * Math.PI; // convert from V/(radians/s) to V/(rotations/s)
  private static final double ANGLE_KA = 0.18755;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 15.0; // determined after manual tuning
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;

  // values from sysid routines
  private static final double DRIVE_KS = 3.9848;
  private static final double DRIVE_KV = 0.058846;
  private static final double DRIVE_KA = 0.2817;

  private static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(5.117);
  private static final LinearVelocity MAX_COAST_VELOCITY = MetersPerSecond.of(0.05);
  private static final double SLOW_MODE_MULTIPLIER = 0.75;

  private static final String CAN_BUS_NAME = "canbus1";
  private static final CANBus CAN_BUS = new CANBus(CAN_BUS_NAME);

  private static final double AUTO_DRIVE_P_CONTROLLER = 5.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 5.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP =
      8; // from preliminary testing, still needs more tuning
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP =
      7.5; // 18.0; // from preliminary testing, still needs more tuning
  private static final double DRIVE_TO_POSE_THETA_KI = 10.0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final Distance DRIVE_TO_POSE_DRIVE_TOLERANCE = Meters.of(0.06);
  private static final Angle DRIVE_TO_POSE_THETA_TOLERANCE = Radians.of(0.02);
  private static final LinearVelocity DRIVE_TO_POSE_MAX_VELOCITY = MetersPerSecond.of(1.25);

  private static final LinearVelocity SQUARING_SPEED = MetersPerSecond.of(1.0);

  // Drive Facing Angle constants
  private static final double DRIVE_FACING_ANGLE_KP = 2.0;
  private static final double DRIVE_FACING_ANGLE_KD = 0.1;
  private static final double DRIVE_FACING_ANGLE_KI = 0.0;

  private static final int LED_COUNT = 85;

  private static final String FR_CAMERA_SERIAL_NUMBER = "40686739";
  private static final String BR_CAMERA_NAME = "40708542";
  private static final String FL_CAMERA_NAME = "40708556";
  private static final String BL_CAMERA_NAME = "40708569";
  private static final String CENTER_CAMERA_NAME = "25249734";

  private static final int MONO_EXPOSURE = 2200;
  private static final double MONO_GAIN = 17.5;
  private static final double MONO_DENOISE = 1.0;

  private static final int COLOR_EXPOSURE = 4500;
  private static final double COLOR_GAIN = 5.0;

  // Front right camera
  // x, y, z, pitch, yaw
  // 11.4425	-8.0165	6.436	60	-52.239
  private static final Transform3d ROBOT_TO_FR_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.0165),
              Units.inchesToMeters(11.4425),
              Units.inchesToMeters(7.436)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-52.239)));

  // Back right camera
  // x, y, z, pitch, yaw
  // -11.4425	-8.0165	6.436	60	-127.761
  private static final Transform3d ROBOT_TO_BR_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.0165),
              Units.inchesToMeters(-11.4425),
              Units.inchesToMeters(7.436)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-127.761)));

  // Front left camera
  // x, y, z, pitch, yaw
  // 11.4425	8.0165	6.436	60	52.239
  private static final Transform3d ROBOT_TO_FL_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(8.0165),
              Units.inchesToMeters(11.4425),
              Units.inchesToMeters(7.436)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(52.239)));
  // Back left camera
  // x, y, z, pitch, yaw
  // -11.4425	8.0165	6.436	60	142.239
  private static final Transform3d ROBOT_TO_BL_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(8.0165),
              Units.inchesToMeters(-11.4425),
              Units.inchesToMeters(7.436)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(142.239)));

  // center camera
  // for testing, this camera is currently mounted on the back left
  private static final Transform3d ROBOT_TO_CENTER_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(8.0165),
              Units.inchesToMeters(-11.4425),
              Units.inchesToMeters(7.436)),
          new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(142.239)));

  @Override
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_FR_CAMERA)
          .id(FR_CAMERA_SERIAL_NUMBER)
          .location("FR")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BR_CAMERA)
          .id(BR_CAMERA_NAME)
          .location("BR")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_FL_CAMERA)
          .id(FL_CAMERA_NAME)
          .location("FL")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CENTER_CAMERA)
          .id(CENTER_CAMERA_NAME)
          .location("center")
          .width(1280)
          .height(960)
          .exposure(COLOR_EXPOSURE)
          .gain(COLOR_GAIN)
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
    return Pounds.of(111.0);
  }

  @Override
  public MomentOfInertia getMomentOfInertia() {
    return KilogramSquareMeters.of(3.40);
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
  public LinearVelocity getDriveToPoseDriveMaxVelocity() {
    return DRIVE_TO_POSE_MAX_VELOCITY;
  }

  @Override
  public Distance getDriveToPoseDriveTolerance() {
    return DRIVE_TO_POSE_DRIVE_TOLERANCE;
  }

  @Override
  public Angle getDriveToPoseThetaTolerance() {
    return DRIVE_TO_POSE_THETA_TOLERANCE;
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
