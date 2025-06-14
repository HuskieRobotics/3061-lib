package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class XRPRobotConfig extends RobotConfig {

  // FIXME: update robot dimensions
  private static final Mass MASS =
      Kilograms.of(
          51.862); // FIXME: update based on measured mass of robot with battery and bumpers
  private static final MomentOfInertia MOI = KilogramSquareMeters.of(6.0); // FIXME: measure
  private static final Distance TRACKWIDTH = Meters.of(0.523875);
  private static final Distance WHEELBASE = Meters.of(0.52705);
  private static final Distance WHEEL_RADIUS = Meters.of(0.09845567409 / 2.0);
  private static final double WHEEL_COEFFICIENT_OF_FRICTION =
      1.2; // FIXME: update based on wheel coefficient of friction
  private static final Translation2d FRONT_RIGHT_CORNER_POSITION = new Translation2d(0.36, -0.36);
  private static final Distance ROBOT_WIDTH_WITH_BUMPERS = Meters.of(0.8382);
  private static final Distance ROBOT_LENGTH_WITH_BUMPERS = Meters.of(0.8382);

  // FIXME: characterize the drivetrain and update these constants
  private static final double DRIVE_KS = 5.0;
  private static final double DRIVE_KV = 0.0;
  private static final double DRIVE_KA = 0.0;

  // FIXME: determine maximum velocities empirically
  private static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(3.5);
  private static final LinearVelocity MAX_COAST_VELOCITY = MetersPerSecond.of(0.05);

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
    // the XRP doesn't support swerve, but we need to override this method to avoid errors
    return SwerveConstants.MK4I_L2_CONSTANTS;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    // the XRP doesn't support swerve; so, return an empty array
    return new int[] {};
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    // the XRP doesn't support swerve; so, return an empty array
    return new int[] {};
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    // the XRP doesn't support swerve; so, return an empty array
    return new int[] {};
  }

  @Override
  public Angle[] getSwerveSteerOffsets() {
    // the XRP doesn't support swerve; so, return an empty array
    return new Angle[] {};
  }

  @Override
  public int getGyroCANID() {
    // only one XRP gyro is supported; so an ID isn't needed (nor is it on CAN); so, just return 0
    return 0;
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
  public int getPneumaticsHubCANID() {
    return 0;
  }
}
