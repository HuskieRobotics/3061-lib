package frc.lib.team3061;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;

@java.lang.SuppressWarnings({"java:S3010", "java:S3400"})
public abstract class RobotConfig {

  private static RobotConfig robotConfig;

  /**
   * Returns the singleton instance of the RobotConfig object.
   *
   * @return the singleton instance of the RobotConfig object
   */
  public static RobotConfig getInstance() {
    return robotConfig;
  }

  protected RobotConfig() {
    RobotConfig.robotConfig = this;
  }

  /**
   * Returns if the CANivore and/or CTRE CAN devices are licensed and have access to Phoenix 6 Pro
   * features.
   *
   * @return true if the CANivore and/or CTRE CAN devices are licensed and have access to Phoenix 6
   *     Pro features
   */
  public boolean getPhoenix6Licensed() {
    return false;
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

  /*
   * Returns the voltage needed to overcome the swerve module's static friction. Defaults to 0.
   * This constant will be provided directly to the hardware. Therefore, the units must be that
   * as expected by the hardware.
   *
   * @return the voltage needed to overcome the swerve module's static friction
   */
  public double getSwerveAngleKS() {
    return 0.0;
  }

  /**
   * Returns the voltage needed to hold (or "cruise") at a given constant velocity. Defaults to 0.
   * This constant will be provided directly to the hardware. Therefore, the units must be that as
   * expected by the hardware.
   *
   * @return the voltage needed to hold (or "cruise") at a given constant velocity
   */
  public double getSwerveAngleKV() {
    return 0.0;
  }

  /**
   * Returns the voltage needed to induce a given acceleration in the motor shaft. Defaults to 0.
   * This constant will be provided directly to the hardware. Therefore, the units must be that as
   * expected by the hardware.
   *
   * @return the voltage needed to induce a given acceleration in the motor shaft
   */
  public double getSwerveAngleKA() {
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
   * Returns the voltage needed to overcome the drivetrain's static friction. Defaults to 0. This
   * constant will be provided directly to the hardware. Therefore, the units must be that as
   * expected by the hardware.
   *
   * @return the voltage needed to overcome the drivetrain's static friction
   */
  public double getDriveKS() {
    return 0.0;
  }

  /**
   * Returns the voltage needed to hold (or "cruise") at a given constant velocity. Defaults to 0.
   * This constant will be provided directly to the hardware. Therefore, the units must be that as
   * expected by the hardware.
   *
   * @return the voltage needed to hold (or "cruise") at a given constant velocity
   */
  public double getDriveKV() {
    return 0.0;
  }

  /**
   * Returns the voltage needed to induce a given acceleration in the motor shaft. Defaults to 0.
   * This constant will be provided directly to the hardware. Therefore, the units must be that as
   * expected by the hardware.
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
  public abstract SwerveConstants getSwerveConstants();

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
   * Returns the swerve module offsets, in units of rotations, in the order of front left, front
   * right, back left, and back right. Must be overridden.
   *
   * @return the swerve module offsets, in units of rotations, in the order of front left, front
   *     right, back left, and back right
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
   * Returns the diameter of the wheels on the robot in meters. Must be overridden.
   *
   * @return the diameter of the wheels on the robot in meters
   */
  public abstract double getWheelDiameterMeters();

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
   * Returns the 3D transforms from the center of the robot to the center of each camera. The units
   * are meters and radians. Defaults to an empty array specifying no cameras.
   *
   * @return the 3D transforms from the center of the robot to the center of each camera
   */
  public Transform3d[] getRobotToCameraTransforms() {
    return new Transform3d[] {};
  }

  /**
   * Returns the maximum translational velocity of the robot in meters per second. This is a measure
   * of how fast the robot should be able to drive in a straight line. Must be overridden.
   *
   * @return the maximum velocity of the robot in meters per second
   */
  public abstract double getRobotMaxVelocity();

  /**
   * Returns the multiplier for when the robot is in slow mode. Defaults to 1 (no effect in slow
   * mode).
   *
   * @return the multiplier for when the robot is in slow mode
   */
  public double getRobotSlowModeMultiplier() {
    return 1.0;
  }

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
   * Returns the maximum translational acceleration of the robot in meters per second squared.
   * Defaults to 1000 m/s/s.
   *
   * @return the maximum translational acceleration of the robot in meters per second squared
   */
  public double getRobotMaxDriveAcceleration() {
    return 1000.0;
  }

  /**
   * Returns the maximum angular acceleration of the robot in radians per second squared. Defaults
   * to 1000 rad/s/s.
   *
   * @return the maximum angular acceleration of the robot in radians per second squared
   */
  public double getRobotMaxTurnAcceleration() {
    return 1000.0;
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
   * Returns the names of the cameras used by the vision subsystem. Defaults to an empty array (no
   * cameras).
   *
   * @return the names of the cameras used by the vision subsystem
   */
  public String[] getCameraNames() {
    return new String[] {};
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

  /**
   * Returns the the proportional constant for the PID controller for translational motion when
   * driving to a specific pose. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for translational motion when driving
   *     to a specific pose
   */
  public double getDriveToPoseDriveKP() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for translational motion when driving to a
   * specific pose. Defaults to 0.
   *
   * @return the integral constant for the PID controller for translational motion when driving to a
   *     specific pose
   */
  public double getDriveToPoseDriveKI() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for translational motion when driving to
   * a specific pose. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for translational motion when driving to
   *     a specific pose
   */
  public double getDriveToPoseDriveKD() {
    return 0.0;
  }

  /**
   * Returns the proportional constant for the PID controller for rotational motion when driving to
   * a specific pose. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for rotational motion when driving to
   *     a specific pose
   */
  public double getDriveToPoseThetaKP() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for rotational motion when driving to a
   * specific pose. Defaults to 0.
   *
   * @return the integral constant for the PID controller for rotational motion when driving to a
   *     specific pose
   */
  public double getDriveToPoseThetaKI() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for rotational motion when driving to a
   * specific pose. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for rotational motion when driving to a
   *     specific pose
   */
  public double getDriveToPoseThetaKD() {
    return 0.0;
  }

  /**
   * Returns the maximum translational speed, in meters per second, for the robot during the
   * drive-to-pose command. Defaults to the maximum speed for autonomous.
   *
   * @return the maximum translational speed, in meters per second, for the robot during the
   *     drive-to-pose command
   */
  public double getDriveToPoseDriveMaxVelocity() {
    return getAutoMaxSpeed();
  }

  /**
   * Returns the maximum translational acceleration, in meters per second squared, for the robot
   * during the drive-to-pose command. Defaults to the maximum acceleration for autonomous.
   *
   * @return the maximum translational acceleration, in meters per second squared, for the robot
   *     during the drive-to-pose command
   */
  public double getDriveToPoseDriveMaxAcceleration() {
    return getAutoMaxAcceleration();
  }

  /**
   * Returns the maximum rotational speed, in radians per second, for the robot during the
   * drive-to-pose command. Defaults to the velocity derived from the maximum translational speed
   * and hte robot's geometry.
   *
   * @return the maximum rotational speed, in radians per second, for the robot during the
   *     drive-to-pose command
   */
  public double getDriveToPoseTurnMaxVelocity() {
    return getDriveToPoseDriveMaxVelocity()
        / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  /**
   * Returns the maximum rotational acceleration, in radians per second squared, for the robot
   * during the drive-to-pose command. Defaults to the acceleration derived from the maximum
   * translational acceleration and the robot's geometry.
   *
   * @return the maximum rotational acceleration, in radians per second squared, for the robot
   *     during the drive-to-pose command
   */
  public double getDriveToPoseTurnMaxAcceleration() {
    return getDriveToPoseDriveMaxAcceleration()
        / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  /**
   * Returns the tolerance, in meters, for which the robot's position is considered at the specified
   * pose during the drive-to-pose command. Defaults to 0.
   *
   * @return the tolerance, in meters, for which the robot's position is considered at the specified
   *     pose during the drive-to-pose command
   */
  public double getDriveToPoseDriveTolerance() {
    return 0.0;
  }

  /**
   * Returns the tolerance, in radians, for which the robot's heading is considered at the specified
   * pose during the drive-to-pose command. Defaults to 0.
   *
   * @return the tolerance, in radians, for which the robot's heading is considered at the specified
   *     pose during the drive-to-pose command
   */
  public double getDriveToPoseThetaTolerance() {
    return 0.0;
  }

  /**
   * Returns the velocity, in meters per second, of the robot when driving into a field element
   * during a move-to-pose command. Defaults to 0.
   *
   * @return the velocity, in meters per second, of the robot when driving into a field element
   *     during a move-to-pose command
   */
  public double getMoveToPathFinalVelocity() {
    return 0;
  }

  /**
   * Returns the proportional constant for the PID controller for rotational motion when driving
   * facing angle. See TeleopSwerve for more information. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for rotational motion when driving
   *     facing angle
   */
  public double getDriveFacingAngleThetaKP() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for rotational motion when driving facing
   * angle. See TeleopSwerve for more information. Defaults to 0.
   *
   * @return the integral constant for the PID controller for rotational motion when driving facing
   *     angle
   */
  public double getDriveFacingAngleThetaKI() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for rotational motion when driving
   * facing angle. See TeleopSwerve for more information. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for rotational motion when driving
   *     facing angle
   */
  public double getDriveFacingAngleThetaKD() {
    return 0.0;
  }

  /**
   * Returns the frequency at which the robot's odometry will be updated. Defaults to 250 Hz. This
   * value needs to match the hardware-specific Drivetrain code. For the DrivetrainIOCTRE class, the
   * value is 250 Hz; the DrivetrainIOGeneric class, 50 Hz.
   *
   * @return the frequency at which the robot's odometry will be updated
   */
  public double getOdometryUpdateFrequency() {
    return 250.0;
  }

  /**
   * Returns the hardware driving the LEDs. Defaults to RIO for using the roboRIO and WPILib's
   * AddressableLED class.
   */
  public LED_HARDWARE getLEDHardware() {
    return LED_HARDWARE.RIO;
  }

  /**
   * Returns the number of LEDs in the LED strip on the robot. Defaults to 0.
   *
   * @return the number of LEDs in the LED strip on the robot
   */
  public int getLEDCount() {
    return 0;
  }

  public enum LED_HARDWARE {
    RIO,
    CANDLE
  }

  /*
   * Returns the swerve control mode. Defaults to voltage. For the DrivetrainIOGeneric class, only
   * VOLTAGE is supported. For the DrivetrainIOCTRE class, TORQUE_CURRENT_FOC is also supported with
   * Phoenix Pro.
   * Returns the swerve control mode for the steer motor. Defaults to voltage. For the
   * DrivetrainIOGeneric class, only VOLTAGE is supported. For the DrivetrainIOCTRE class,
   * TORQUE_CURRENT_FOC is also supported with Phoenix Pro.
   *
   * @return the swerve control mode
   */
  public SWERVE_CONTROL_MODE getSwerveSteerControlMode() {
    return SWERVE_CONTROL_MODE.VOLTAGE;
  }

  /**
   * Returns the drive control mode for the drive motor. Defaults to voltage. For the
   * DrivetrainIOGeneric class, only VOLTAGE is supported. For the DrivetrainIOCTRE class,
   * TORQUE_CURRENT_FOC is also supported with Phoenix Pro.
   *
   * @return the swerve control mode
   */
  public SWERVE_CONTROL_MODE getSwerveDriveControlMode() {
    return SWERVE_CONTROL_MODE.VOLTAGE;
  }

  public enum SWERVE_CONTROL_MODE {
    VOLTAGE,
    TORQUE_CURRENT_FOC
  }
}
