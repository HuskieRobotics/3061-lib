package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;

public class NewPracticeRobotConfig extends RobotConfig {
  // 2 mk4n's on the front, 2 mk4is on the back

  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 43;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 44;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
  private static final Angle FRONT_LEFT_MODULE_STEER_OFFSET = Rotations.of(0.271484);

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 41;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 42;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 8;
  private static final Angle FRONT_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.487793 + 0.5);

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 38;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 37;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 17;
  private static final Angle BACK_LEFT_MODULE_STEER_OFFSET = Rotations.of(-0.280029);

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 39;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 40;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
  private static final Angle BACK_RIGHT_MODULE_STEER_OFFSET = Rotations.of(-0.105957 + 0.5);

  private static final int GYRO_ID = 3; // ?

  private static final Distance TRACKWIDTH = Meters.of(0.57785); // 22.75
  private static final Distance WHEELBASE = Meters.of(0.57785); // 22.75
  private static final Distance WHEEL_RADIUS = Meters.of(0.0954405 / 2.0);
  private static final Translation2d FRONT_RIGHT_CORNER_POSITION = new Translation2d(0.36, -0.36);

  private static final Distance ROBOT_WIDTH_WITH_BUMPERS = Meters.of(0.88026); // 34.656 in
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS = Meters.of(0.88026); // 34.656 in

  private static final double COUPLE_RATIO = 3.125; // FIXME: tune

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 100.0;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.05;

  // FIXME: tune; starting values from Phoenix Tuner X .withKS(0.1).withKV(2.33).withKA(0)
  private static final double ANGLE_KS = 0.1891233333;
  private static final double ANGLE_KV =
      0.4399866667 * 2 * Math.PI; // convert from V/(radians/s) to V/(rotations/s)
  private static final double ANGLE_KA = 0.001663333333;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 8.0;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;

  private static final double DRIVE_KS = 5.0;
  private static final double DRIVE_KV = 0.0;
  private static final double DRIVE_KA = 0.0;

  private static final LinearVelocity MAX_VELOCITY =
      MetersPerSecond.of(5.5); // FIXME: confirm max velocity with real robot
  private static final LinearVelocity MAX_COAST_VELOCITY = MetersPerSecond.of(0.05);
  private static final double SLOW_MODE_MULTIPLIER = 0.75;

  private static final String CAN_BUS_NAME = "canbus1";

  private static final double AUTO_DRIVE_P_CONTROLLER = 5.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 5.0;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP = 2.5;
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP = 18.0;
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
    return SwerveConstants.MK4N_L3_PLUS_CONSTANTS;
  }

  @Override
  public SwerveConstants getFrontSwerveConstants() {
    return SwerveConstants.MK4N_L3_PLUS_CONSTANTS;
  }

  @Override
  public SwerveConstants getBackSwerveConstants() {
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
    return Kilograms.of(51.862);
  }

  @Override
  public MomentOfInertia getMomentOfInertia() {
    return KilogramSquareMeters.of(6.0);
  }

  @Override
  public double getWheelCOF() {
    return 1.2;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
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
