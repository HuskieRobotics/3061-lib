package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Rotation3d;
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
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;

/** Most of this is copied from Artemis; update with actual values */
public class New2025RobotConfig extends RobotConfig {
  // FIXME: UPDATE ALL OFFSETS WITH REAL ROBOT TOMORROW

  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 25;
  private static final Angle FRONT_LEFT_MODULE_STEER_OFFSET = Rotations.of(-0.181152);

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24;
  private static final Angle FRONT_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.299805);

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
  private static final MomentOfInertia MOI =
      KilogramSquareMeters.of(6.5); // FIXME: update based on mass
  private static final Distance TRACKWIDTH = Meters.of(0.57785);
  private static final Distance WHEELBASE = Meters.of(0.57785);
  private static final Distance WHEEL_RADIUS = Meters.of(0.049930);
  private static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.5;
  private static final Translation2d FRONT_RIGHT_CORNER_POSITION =
      new Translation2d(0.34925, 0.34925);

  private static final Distance ROBOT_WIDTH_WITH_BUMPERS =
      Meters.of(0.85725); // confirm with actual bumpers
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS =
      Meters.of(0.85725); // confirm with actual bumpers

  private static final double COUPLE_RATIO = 3.125; // possibly needs to be updated for mk4ns

  /* PID Values, Update all of these based on sysid routine when new robot arrives */
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

  private static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(5.0); // actually 4.16
  private static final LinearVelocity MAX_COAST_VELOCITY =
      MetersPerSecond.of(0.04); // FIXME: determine with real robot
  private static final double SLOW_MODE_MULTIPLIER = 0.7;

  private static final String CAN_BUS_NAME = "canbus1";

  private static final String CAMERA_NAME_0 = "OV2311FR";
  private static final String CAMERA_NAME_1 = "OV2311BR";
  private static final String CAMERA_NAME_2 = "OV2311FL";
  private static final String CAMERA_NAME_3 = "OV2311BL";

  // Back cameras have positive pitches because they are on the opposite side of the robot,
  // so although they are pitched upward, robot relative that is downward

  // Front right camera
  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(6.659),
              Units.inchesToMeters(-10.725),
              Units.inchesToMeters(9.601)),
          new Rotation3d(0, Units.degreesToRadians(-24.248), Units.degreesToRadians(20.577)));
  // pitch 45 degrees

  // Back right camera
  private static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.497),
              Units.inchesToMeters(-10.741),
              Units.inchesToMeters(8.040)),
          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-162)));

  // Front left camera
  private static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(6.657),
              Units.inchesToMeters(10.724),
              Units.inchesToMeters(9.605)),
          new Rotation3d(0, Units.degreesToRadians(-24.248), Units.degreesToRadians(-20.577)));

  // Back left camera
  private static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.497),
              Units.inchesToMeters(10.741),
              Units.inchesToMeters(8.040)),
          new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(162)));

  // default values for tunables
  private static final double AUTO_DRIVE_P_CONTROLLER = 5.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 5.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP = 8.0; // FIXME: update with testing
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP = 7.5; // FIXME: update with testing
  private static final double DRIVE_TO_POSE_THETA_KI = 0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final Distance DRIVE_TO_POSE_DRIVE_TOLERANCE =
      Meters.of(0.06); // FIXME: update with testing (different from reef?)
  private static final Angle DRIVE_TO_POSE_THETA_TOLERANCE = Radians.of(0.02);
  private static final LinearVelocity DRIVE_TO_POSE_MAX_VELOCITY = MetersPerSecond.of(1.25);
  private static final LinearAcceleration DRIVE_TO_POSE_MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(2.5);

  private static final LinearVelocity SQUARING_SPEED = MetersPerSecond.of(1.0);

  // Drive Facing Angle constants
  private static final double DRIVE_FACING_ANGLE_KP = 6.0;
  private static final double DRIVE_FACING_ANGLE_KD = 0.1;
  private static final double DRIVE_FACING_ANGLE_KI = 0.0;

  private static final int LED_COUNT = 35; // FIXME: update for new leds

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
  public Transform3d[] getRobotToCameraTransforms() {
    return new Transform3d[] {
      ROBOT_TO_CAMERA_0, ROBOT_TO_CAMERA_1, ROBOT_TO_CAMERA_2, ROBOT_TO_CAMERA_3
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
  public String[] getCameraNames() {
    return new String[] {CAMERA_NAME_0, CAMERA_NAME_1, CAMERA_NAME_2, CAMERA_NAME_3};
  }

  @Override
  public double[] getCameraStdDevFactors() {
    return new double[] {1.0, 1.0, 1.0, 1.0};
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
  public LinearVelocity getDriveToPoseDriveMaxVelocity() {
    return DRIVE_TO_POSE_MAX_VELOCITY;
  }

  @Override
  public LinearAcceleration getDriveToPoseDriveMaxAcceleration() {
    return DRIVE_TO_POSE_MAX_ACCELERATION;
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
