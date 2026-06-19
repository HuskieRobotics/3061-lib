package frc.robot.configs;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class DefaultRobotConfig extends RobotConfig {

  // FIXME: update all CAN IDs and steer offsets
  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 13;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
  private static final double FRONT_LEFT_MODULE_STEER_OFFSET_ROTS = -0.22591;

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 16;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 15;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
  private static final double FRONT_RIGHT_MODULE_STEER_OFFSET_ROTS = -0.390381;

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;
  private static final double BACK_LEFT_MODULE_STEER_OFFSET_ROTS = 0.327393;

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 9;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
  private static final double BACK_RIGHT_MODULE_STEER_OFFSET_ROTS = -0.336670;

  private static final int GYRO_ID = 3;

  // FIXME: update robot dimensions
  private static final double MASS_KG =
      51.862; // FIXME: update based on measured mass of robot with battery and bumpers
  private static final double MOI = 6.0; // FIXME: measure
  private static final double TRACKWIDTH_METERS = 0.523875;
  private static final double WHEELBASE_METERS = 0.52705;
  private static final double WHEEL_RADIUS_METERS = 0.09845567409 / 2.0;
  private static final double WHEEL_COEFFICIENT_OF_FRICTION =
      1.2; // FIXME: update based on wheel coefficient of friction
  private static final Translation2d FRONT_RIGHT_CORNER_POSITION = new Translation2d(0.36, -0.36);
  private static final double ROBOT_WIDTH_WITH_BUMPERS_METERS = 0.8382;
  private static final double ROBOT_LENGTH_WITH_BUMPERS_METERS = 0.8382;

  private static final double COUPLE_RATIO = 3.125; // FIXME: tune

  // FIXME: tune PID values for the angle and drive motors for the swerve modules

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 100.0;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.05;

  // FIXME: characterize the drivetrain and update these constants
  private static final double ANGLE_KS = 0.1891233333;
  private static final double ANGLE_KV =
      0.4399866667 * 2 * Math.PI; // convert from V/(radians/s) to V/(rotations/s)
  private static final double ANGLE_KA = 0.001663333333;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 8.0;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;

  // FIXME: characterize the drivetrain and update these constants
  private static final double DRIVE_KS = 5.0;
  private static final double DRIVE_KV = 0.0;
  private static final double DRIVE_KA = 0.0;

  // FIXME: determine maximum velocities empirically
  private static final double MAX_VELOCITY_MPS = 3.5;
  private static final double MAX_COAST_VELOCITY_MPS = 0.05;
  private static final double SLOW_MODE_MULTIPLIER = 0.75;

  // FIXME: specify the name of the CANivore CAN FD bus as appropriate (an empty string uses the
  // default CAN bus)
  private static final String CAN_BUS_NAME = "";
  private static final CANBus CAN_BUS = new CANBus(CAN_BUS_NAME);

  // FIXME: specify the name of the camera used for detecting AprilTags
  private static final String CAMERA_NAME = "OV2311";

  // FIXME: update this with the actual transform from the robot to the camera
  private static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  // FIXME: specify the configuration for pneumatics
  private static final int PNEUMATICS_HUB_ID = 20;
  private static final int FLOW_SENSOR_CHANNEL = 0;
  private static final int REV_HIGH_PRESSURE_SENSOR_CHANNEL = 0;
  private static final int REV_LOW_PRESSURE_SENSOR_CHANNEL = 1;

  // FIXME: specify maximum velocity and acceleration and tune PID values for auto paths
  private static final double AUTO_DRIVE_P_CONTROLLER = 5.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 5.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // FIXME: tune PID values for drive to pose
  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP = 2.5;
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP = 18.0;
  private static final double DRIVE_TO_POSE_THETA_KI = 10.0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final double DRIVE_TO_POSE_DRIVE_TOLERANCE_METERS = 0.06;
  private static final double DRIVE_TO_POSE_THETA_TOLERANCE_RADIANS = 0.02;
  private static final double DRIVE_TO_POSE_MAX_VELOCITY_MPS = 1.25;

  private static final double SQUARING_SPEED_MPS = 1.0;

  // FIXME: tune PID values for drive facing angle
  // Drive Facing Angle constants
  private static final double DRIVE_FACING_ANGLE_KP = 6.0;
  private static final double DRIVE_FACING_ANGLE_KD = 0.1;
  private static final double DRIVE_FACING_ANGLE_KI = 0.0;

  // FIXME: specify the number of LEDs
  private static final int LED_COUNT = 85;

  @Override
  public boolean getPhoenix6Licensed() {
    // FIXME: return true if you have Phoenix 6 Pro license
    return false;
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
    // FIXME: specify the type of swerve module (MK4, MK4i, MK4n are supported)
    return SwerveConstants.MK4I_L2_CONSTANTS;
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
      FRONT_LEFT_MODULE_STEER_OFFSET_ROTS,
      FRONT_RIGHT_MODULE_STEER_OFFSET_ROTS,
      BACK_LEFT_MODULE_STEER_OFFSET_ROTS,
      BACK_RIGHT_MODULE_STEER_OFFSET_ROTS
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
    return MASS_KG;
  }

  @Override
  public double getMomentOfInertiaKGMM() {
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
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA)
          .id(CAMERA_NAME)
          .location("center")
          .width(1600)
          .height(1200)
          .stdDevFactor(1.0)
          .build(),
    };
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
    // FIXME: return 250 Hz if using the DrivetrainIOCTRE class
    return 50.0;
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
