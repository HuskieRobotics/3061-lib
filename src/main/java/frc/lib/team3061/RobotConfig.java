package frc.lib.team3061;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.team3061.swerve.SwerveModuleConstants;

@java.lang.SuppressWarnings({"java:S3010", "java:S3400"})
public abstract class RobotConfig {

  private static RobotConfig robotConfig;

  public static RobotConfig getInstance() {
    return robotConfig;
  }

  protected RobotConfig() {
    RobotConfig.robotConfig = this;
  }

  // Swerve Module PID accessors
  public double getSwerveAngleKP() {
    return 0.0;
  }

  public double getSwerveAngleKI() {
    return 0.0;
  }

  public double getSwerveAngleKD() {
    return 0.0;
  }

  public double getSwerveAngleKF() {
    return 0.0;
  }

  public double getSwerveDriveKP() {
    return 0.0;
  }

  public double getSwerveDriveKI() {
    return 0.0;
  }

  public double getSwerveDriveKD() {
    return 0.0;
  }

  public double getSwerveDriveKF() {
    return 0.0;
  }

  // Drive Characterization accessors
  public double getDriveKS() {
    return 0.0;
  }

  public double getDriveKV() {
    return 0.0;
  }

  public double getDriveKA() {
    return 0.0;
  }

  // Swerve Module CAN IDs (FL, FR, BL, BR)
  public abstract int[] getSwerveDriveMotorCANIDs();

  public abstract int[] getSwerveSteerMotorCANIDs();

  public abstract int[] getSwerveSteerEncoderCANIDs();

  public double[] getSwerveSteerOffsets() {
    return new double[] {0.0, 0.0, 0.0, 0.0};
  }

  public abstract int getPigeonCANID();

  // robot dimensions accessors
  public abstract double getTrackwidth();

  public abstract double getWheelbase();

  /* The geometry and coordinate systems can be confusing. Refer to this document
  for a detailed explanation: https://docs.google.com/document/d/17dg5cIfqVOlQTTbo2ust4QxTZlUoPNzuBu2oe58Ov84/edit#heading=h.x4ppzp81ed1
  */
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return new SwerveDriveKinematics(
        // Front left
        new Translation2d(getWheelbase() / 2.0, getTrackwidth() / 2.0),
        // Front right
        new Translation2d(getWheelbase() / 2.0, -getTrackwidth() / 2.0),
        // Back left
        new Translation2d(-getWheelbase() / 2.0, getTrackwidth() / 2.0),
        // Back right
        new Translation2d(-getWheelbase() / 2.0, -getTrackwidth() / 2.0));
  }

  public double getRobotWidthWithBumpers() {
    return 0.0;
  }

  public double getRobotLengthWithBumpers() {
    return 0.0;
  }

  public Transform3d getRobotToCameraTransform() {
    return new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  }

  // robot maximum velocity and acceleration

  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public double getRobotMaxVelocity() {
    return 6380.0
        / 60.0
        / SwerveModuleConstants.DRIVE_GEAR_RATIO
        * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
  }

  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  public double getRobotMaxAcceleration() {
    return getRobotMaxVelocity() / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  public double getRobotMaxCoastVelocity() {
    return 0.0;
  }

  // auto max velocity and acceleration
  public double getAutoMaxSpeed() {
    return 0.0;
  }

  public double getAutoMaxAcceleration() {
    return 0.0;
  }

  // auto path PIDs
  public double getAutoDriveKP() {
    return 0.0;
  }

  public double getAutoDriveKI() {
    return 0.0;
  }

  public double getAutoDriveKD() {
    return 0.0;
  }

  public double getAutoTurnKP() {
    return 0.0;
  }

  public double getAutoTurnKI() {
    return 0.0;
  }

  public double getAutoTurnKD() {
    return 0.0;
  }

  public String getCANBusName() {
    return "";
  }

  public String getCameraName() {
    return "";
  }

  public abstract int getPneumaticsHubCANID();

  public int getFlowSensorChannel() {
    return 0;
  }

  public int getRevHighPressureSensorChannel() {
    return 0;
  }

  public int getRevLowPressureSensorChannel() {
    return 1;
  }
}
