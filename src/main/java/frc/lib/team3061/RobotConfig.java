package frc.lib.team3061;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.team3061.swerve.SwerveModuleConstants.SwerveType;

@java.lang.SuppressWarnings({"java:S3010", "java:S3400"})
public abstract class RobotConfig {

  private static RobotConfig robotConfig;

  public static RobotConfig getInstance() {
    return robotConfig;
  }

  protected RobotConfig() {
    RobotConfig.robotConfig = this;
  }

  /**
   * Returns the proportional constant for the PID controller for the angle motor on the swerve
   * module. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for the angle motor on the swerve
   *     module
   */
  public double getSwerveAngleKP() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for the angle motor on the swerve module.
   * Defaults to 0.
   *
   * @return the integral constant for the PID controller for the angle motor on the swerve module
   */
  public double getSwerveAngleKI() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for the angle motor on the swerve
   * module. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for the angle motor on the swerve module
   */
  public double getSwerveAngleKD() {
    return 0.0;
  }

  /**
   * Returns the feedforward constant for the PID controller for the angle motor on the swerve
   * module. Defaults to 0.
   *
   * @return the feedforward constant for the PID controller for the angle motor on the swerve
   *     module
   */
  public double getSwerveAngleKF() {
    return 0.0;
  }

  /**
   * Returns the proportional constant for the PID controller for the drive motor on the swerve
   * module. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for the drive motor on the swerve
   *     module
   */
  public double getSwerveDriveKP() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for the drive motor on the swerve module.
   * Defaults to 0.
   *
   * @return the integral constant for the PID controller for the drive motor on the swerve module
   */
  public double getSwerveDriveKI() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for the drive motor on the swerve
   * module. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for the drive motor on the swerve module
   */
  public double getSwerveDriveKD() {
    return 0.0;
  }

  /**
   * Returns the feedforward constant for the PID controller for the drive motor on the swerve
   * module. Defaults to 0.
   *
   * @return the feedforward constant for the PID controller for the drive motor on the swerve
   *     module
   */
  public double getSwerveDriveKF() {
    return 0.0;
  }

  /**
   * Returns the voltage needed to overcome the drivetrain's static friction. Defaults to 0.
   *
   * @return the voltage needed to overcome the drivetrain's static friction
   */
  public double getDriveKS() {
    return 0.0;
  }

  /**
   * Returns the voltage needed to hold (or “cruise”) at a given constant velocity. Defaults to 0.
   *
   * @return the voltage needed to hold (or “cruise”) at a given constant velocity
   */
  public double getDriveKV() {
    return 0.0;
  }

  /**
   * Returns the voltage needed to induce a given acceleration in the motor shaft. Defaults to 0.
   *
   * @return the voltage needed to induce a given acceleration in the motor shaft
   */
  public double getDriveKA() {
    return 0.0;
  }

  /**
   * Returns the swerve type for this robot. Must be overridden.
   *
   * @return the swerve type for this robot
   */
  public abstract SwerveType getSwerveType();

  // Swerve Module CAN IDs (FL, FR, BL, BR)
  /**
   * Returns the CAN IDs for the swerve modules' drive motors in the order of front left, front
   * right, back left, and back right. Must be overridden.
   *
   * @return the CAN IDs for the swerve modules' drive motors in the order of front left, front
   *     right, back left, and back right
   */
  public abstract int[] getSwerveDriveMotorCANIDs();

  /**
   * Returns the CAN IDs for the swerve modules' angle motors in the order of front left, front
   * right, back left, and back right. Must be overridden.
   *
   * @return the CAN IDs for the swerve modules' angle motors in the order of front left, front
   *     right, back left, and back right
   */
  public abstract int[] getSwerveSteerMotorCANIDs();

  /**
   * Returns the CAN IDs for the swerve modules' angle encoders in the order of front left, front
   * right, back left, and back right. Must be overridden.
   *
   * @return the CAN IDs for the swerve modules' angle encoders in the order of front left, front
   *     right, back left, and back right
   */
  public abstract int[] getSwerveSteerEncoderCANIDs();

  /**
   * Returns the swerve module offsets in the order of front left, front right, back left, and back
   * right. Must be overridden.
   *
   * @return the swerve module offsets in the order of front left, front right, back left, and back
   *     right
   */
  public abstract double[] getSwerveSteerOffsets();

  /**
   * Returns the CAN ID for the robot's gyro sensor. Must be overridden.
   *
   * @return the CAN ID for the robot's gyro sensor
   */
  public abstract int getGyroCANID();

  /**
   * Returns the trackwidth (i.e., the center-to-center distance between the left and right wheels)
   * of the robot in meters. Must be overridden.
   *
   * @return the trackwidth (i.e., the center-to-center distance between the left and right wheels)
   *     of the robot in meters
   */
  public abstract double getTrackwidth();

  /**
   * Returns the wheelbase (i.e., the center-to-center distance between the front and back wheels)
   * of the robot in meters. Must be overridden.
   *
   * @return the wheelbase (i.e., the center-to-center distance between the front and back wheels)
   *     of the robot in meters
   */
  public abstract double getWheelbase();

  /**
   * Returns the swerve drive kinematics object for the robot. The geometry and coordinate systems
   * can be confusing. Refer to this document for a detailed explanation:
   * https://docs.google.com/document/d/17dg5cIfqVOlQTTbo2ust4QxTZlUoPNzuBu2oe58Ov84/edit#heading=h.x4ppzp81ed1
   *
   * @return the swerve drive kinematics object for the robot
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

  /**
   * Returns the robot's width, including bumpers, in meters. Defaults to 0.
   *
   * @return the robot's width, including bumpers, in meters
   */
  public double getRobotWidthWithBumpers() {
    return 0.0;
  }

  /**
   * Returns the robot's length, including bumpers, in meters. Defaults to 0.
   *
   * @return the robot's length, including bumpers, in meters
   */
  public double getRobotLengthWithBumpers() {
    return 0.0;
  }

  /**
   * Returns the 3D transform from the center of the robot to the center of the camera. The units
   * are meters and radians. Defaults to the robot's center on the floor.
   *
   * @return the 3D transform from the center of the robot to the center of the camera
   */
  public Transform3d getRobotToCameraTransform() {
    return new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
  }

  /**
   * Returns the maximum velocity of the robot in meters per second. This is a measure of how fast
   * the robot should be able to drive in a straight line. Must be overridden.
   *
   * @return the maximum velocity of the robot in meters per second
   */
  public abstract double getRobotMaxVelocity();

  /**
   * The maximum angular velocity of the robot in radians per second. This is a measure of how fast
   * the robot can rotate in place. By default it is calculated based on the maximum velocity and
   * the robot's geometry.
   *
   * @return the maximum angular velocity of the robot in radians per second
   */
  public double getRobotMaxAngularVelocity() {
    return getRobotMaxVelocity() / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  /**
   * Returns the maximum velocity, in meters per second, at which the robot can be moving while
   * disabled before the drive motors are changed from brake to coast mode. Defaults to 0.
   *
   * @return the maximum velocity, in meters per second, at which the robot can be moving while
   *     disabled before the drive motors are changed from brake to coast mode
   */
  public double getRobotMaxCoastVelocity() {
    return 0.0;
  }

  /**
   * Returns the maximum speed, in meters per second, for the robot when following autonomous paths.
   * Must be overridden.
   *
   * @return the maximum speed, in meters per second, for the robot when following autonomous paths
   */
  public abstract double getAutoMaxSpeed();

  /**
   * Returns the maximum acceleration, in meters per second squared, for the robot when following
   * autonomous paths. Must be overridden.
   *
   * @return the maximum acceleration, in meters per second squared, for the robot when following
   *     autonomous paths
   */
  public abstract double getAutoMaxAcceleration();

  /**
   * Returns the the proportional constant for the PID controller for translational motion when
   * following autonomous paths. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for translational motion when
   *     following autonomous paths
   */
  public double getAutoDriveKP() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for translational motion when following
   * autonomous paths. Defaults to 0.
   *
   * @return the integral constant for the PID controller for translational motion when following
   *     autonomous paths
   */
  public double getAutoDriveKI() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for translational motion when following
   * autonomous paths. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for translational motion when following
   *     autonomous paths
   */
  public double getAutoDriveKD() {
    return 0.0;
  }

  /**
   * Returns the proportional constant for the PID controller for rotational motion when following
   * autonomous paths. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for rotational motion when following
   *     autonomous paths
   */
  public double getAutoTurnKP() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for rotational motion when following
   * autonomous paths. Defaults to 0.
   *
   * @return the integral constant for the PID controller for rotational motion when following
   *     autonomous paths
   */
  public double getAutoTurnKI() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for rotational motion when following
   * autonomous paths. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for rotational motion when following
   *     autonomous paths
   */
  public double getAutoTurnKD() {
    return 0.0;
  }

  /**
   * Returns the name of CAN FD (CANivore) bus. Defaults to "" which is for the default (non-FD) CAN
   * bus)
   *
   * @return the name of CAN FD (CANivore) bus
   */
  public String getCANBusName() {
    return "";
  }

  /**
   * Returns the name of the camera used by the vision subsystem. Defaults to "".
   *
   * @return the name of the camera used by the vision subsystem
   */
  public String getCameraName() {
    return "";
  }

  /**
   * Returns the CAN ID of the pneumatics hub. Must be overridden.
   *
   * @return the CAN ID of the pneumatics hub
   */
  public abstract int getPneumaticsHubCANID();

  /**
   * Returns the analog input channel number to which the flow sensor is connected. Defaults to 0.
   *
   * @return the analog input channel number to which the flow sensor is connected
   */
  public int getFlowSensorChannel() {
    return 0;
  }

  /**
   * Returns the channel on the Rev Pneumatics Hub to which the Rev pressure sensor monitoring
   * upstream of the regulator (i.e., high pressure). Defaults to 0.
   *
   * @return the channel on the Rev Pneumatics Hub to which the Rev pressure sensor monitoring
   *     upstream of the regulator (i.e., high pressure)
   */
  public int getRevHighPressureSensorChannel() {
    return 0;
  }

  /**
   * Returns the channel on the Rev Pneumatics Hub to which the Rev pressure sensor monitoring
   * downstream of the regulator (i.e., low pressure). Defaults to 1.
   *
   * @return the channel on the Rev Pneumatics Hub to which the Rev pressure sensor monitoring
   *     downstream of the regulator (i.e., low pressure)
   */
  public int getRevLowPressureSensorChannel() {
    return 1;
  }
}
