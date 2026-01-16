package frc.lib.team3061;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;
import java.io.IOException;
import lombok.Builder;
import org.json.simple.parser.ParseException;

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
   * Returns the type of drivetrain for this robot. This is used to determine which drivetrain
   * subsystem to expect. Defaults to {@link DRIVETRAIN_TYPE#SWERVE}.
   *
   * @return the type of drivetrain for this robot
   */
  public DRIVETRAIN_TYPE getDrivetrainType() {
    return DRIVETRAIN_TYPE.SWERVE;
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
   * Returns the swerve type for the front modules for this robot. Defaults to the swerve type
   * returned by getSwerveConstants().
   *
   * @return the swerve type for the front modules for this robot
   */
  public SwerveConstants getFrontSwerveConstants() {
    return getSwerveConstants();
  }

  /**
   * Returns the swerve type for the back modules for this robot. Defaults to the swerve type
   * returned by getSwerveConstants().
   *
   * @return the swerve type for the back modules for this robot
   */
  public SwerveConstants getBackSwerveConstants() {
    return getSwerveConstants();
  }

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
  public abstract Angle[] getSwerveSteerOffsets();

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
  public abstract Distance getTrackwidth();

  /**
   * Returns the wheelbase (i.e., the center-to-center distance between the front and back wheels)
   * of the robot in meters. Must be overridden.
   *
   * @return the wheelbase (i.e., the center-to-center distance between the front and back wheels)
   *     of the robot in meters
   */
  public abstract Distance getWheelbase();

  /**
   * Returns the radius of the wheels on the robot in meters. Must be overridden.
   *
   * @return the radius of the wheels on the robot in meters
   */
  public abstract Distance getWheelRadius();

  /**
   * Returns the swerve drive kinematics object for the robot. The geometry and coordinate systems
   * can be confusing. Refer to this document for a detailed explanation:
   * https://docs.google.com/document/d/17dg5cIfqVOlQTTbo2ust4QxTZlUoPNzuBu2oe58Ov84/edit#heading=h.x4ppzp81ed1
   *
   * @return the swerve drive kinematics object for the robot
   */
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return new SwerveDriveKinematics(getSwerveModulePositions());
  }

  /**
   * Returns the differential drive kinematics object for the robot. This is used for differential
   * drive robots.
   *
   * @return the differential drive kinematics object for the robot
   */
  public DifferentialDriveKinematics getDifferentialDriveKinematics() {
    return new DifferentialDriveKinematics(getTrackwidth());
  }

  /**
   * Returns the positions of the swerve modules on the robot. The positions are relative to the
   * center of the robot. The order is front left, front right, back left, and back right.
   *
   * @return the positions of the swerve modules on the robot
   */
  public Translation2d[] getSwerveModulePositions() {
    return new Translation2d[] {
      // Front left
      new Translation2d(getWheelbase().div(2.0), getTrackwidth().div(2.0)),
      // Front right
      new Translation2d(getWheelbase().div(2.0), getTrackwidth().div(-2.0)),
      // Back left
      new Translation2d(getWheelbase().div(-2.0), getTrackwidth().div(2.0)),
      // Back right
      new Translation2d(getWheelbase().div(-2.0), getTrackwidth().div(-2.0))
    };
  }

  /**
   * Returns the position of the front left corner of the robot. Defaults to the center of the
   * robot.
   *
   * @return the position of the front left corner of the robot
   */
  public Translation2d getFrontRightCornerPosition() {
    return new Translation2d(0.0, 0.0);
  }

  /**
   * Returns the robot's width, including bumpers, in meters. Defaults to 0.
   *
   * @return the robot's width, including bumpers, in meters
   */
  public Distance getRobotWidthWithBumpers() {
    return Meters.of(0.0);
  }

  /**
   * Returns the robot's length, including bumpers, in meters. Defaults to 0.
   *
   * @return the robot's length, including bumpers, in meters
   */
  public Distance getRobotLengthWithBumpers() {
    return Meters.of(0.0);
  }

  /**
   * Returns the configurations of the cameras used by the vision subsystem. Defaults to an empty
   * array specifying no cameras.
   *
   * @return the configurations of the cameras used by the vision subsystem
   */
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {};
  }

  /**
   * Returns the maximum translational velocity of the robot in meters per second. This is a measure
   * of how fast the robot should be able to drive in a straight line. Must be overridden.
   *
   * @return the maximum velocity of the robot in meters per second
   */
  public abstract LinearVelocity getRobotMaxVelocity();

  /**
   * Returns the maximum translational acceleration, in meters per second squared, for the robot
   * when driving with acceleration limiting enabled. Defaults to twice the robot's maximum
   * velocity.
   *
   * @return the maximum translational acceleration, in meters per second squared, for the robot
   *     when driving with acceleration limiting enabled
   */
  public LinearAcceleration getRobotMaxAccelerationWhenLimited() {
    return MetersPerSecondPerSecond.of(getRobotMaxVelocity().in(MetersPerSecond) * 2.0);
  }

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
  public AngularVelocity getRobotMaxAngularVelocity() {
    return RadiansPerSecond.of(
        getRobotMaxVelocity().in(MetersPerSecond)
            / Math.hypot(getTrackwidth().in(Meters) / 2.0, getWheelbase().in(Meters) / 2.0));
  }

  /**
   * Returns the maximum rotational acceleration, in radians per second squared, for the robot when
   * driving with acceleration limiting enabled. Defaults to the acceleration derived from the
   * maximum translational acceleration and the robot's geometry.
   *
   * @return the maximum rotational acceleration, in radians per second squared, for the robot when
   *     driving with acceleration limiting enabled
   */
  public AngularAcceleration getRobotMaxAngularAccelerationWhenLimited() {
    return RadiansPerSecondPerSecond.of(
        getRobotMaxAccelerationWhenLimited().in(MetersPerSecondPerSecond)
            / Math.hypot(getTrackwidth().in(Meters) / 2.0, getWheelbase().in(Meters) / 2.0));
  }

  /**
   * Returns the maximum velocity, in meters per second, at which the robot can be moving while
   * disabled before the drive motors are changed from brake to coast mode. Defaults to 0.
   *
   * @return the maximum velocity, in meters per second, at which the robot can be moving while
   *     disabled before the drive motors are changed from brake to coast mode
   */
  public LinearVelocity getRobotMaxCoastVelocity() {
    return MetersPerSecond.of(0.0);
  }

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
   * Returns the robot's configuration as specified in the PathPlanner GUI.
   *
   * @return
   * @throws IOException
   * @throws ParseException
   */
  public com.pathplanner.lib.config.RobotConfig getPathPlannerRobotConfig() {
    return new com.pathplanner.lib.config.RobotConfig(
        getMass(),
        getMomentOfInertia(),
        new ModuleConfig(
            getWheelRadius(),
            getRobotMaxVelocity(),
            getWheelCOF(),
            DCMotor.getKrakenX60(1).withReduction(getSwerveConstants().getDriveGearRatio()),
            Amps.of(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT),
            1),
        getSwerveModulePositions());
  }

  /**
   * Returns the mass of the robot. Defaults to 50 kg.
   *
   * @return the mass of the robot
   */
  public Mass getMass() {
    return Kilograms.of(50.0);
  }

  /**
   * Returns the moment of inertia of the robot. Defaults to 6.0 kg*m^2.
   *
   * @return the moment of inertia of the robot
   */
  public MomentOfInertia getMomentOfInertia() {
    return KilogramSquareMeters.of(6.0);
  }

  /**
   * Returns the coefficient of friction for the robot's wheels. Defaults to 1.2.
   *
   * @return the coefficient of friction for the robot's wheels
   */
  public double getWheelCOF() {
    return 1.2;
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
   * Returns a reference the CAN bus. Defaults to the rio (non-FD) CAN bus.
   *
   * @return
   */
  public CANBus getCANBus() {

    return new CANBus();
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
   * Returns the the proportional constant for the PID controller for translational motion in the x
   * direction in the frame of the target pose when driving to a specific pose. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for translational motion when driving
   *     to a specific pose
   */
  public double getDriveToPoseDriveXKP() {
    return 0.0;
  }

  /**
   * Returns the the proportional constant for the PID controller for translational motion in the y
   * direction in the frame of the target pose when driving to a specific pose. Defaults to 0.
   *
   * @return the proportional constant for the PID controller for translational motion when driving
   *     to a specific pose
   */
  public double getDriveToPoseDriveYKP() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for translational motion in the x
   * direction in the frame of the target pose when driving to a specific pose. Defaults to 0.
   *
   * @return the integral constant for the PID controller for translational motion when driving to a
   *     specific pose
   */
  public double getDriveToPoseDriveXKI() {
    return 0.0;
  }

  /**
   * Returns the integral constant for the PID controller for translational motion in the x
   * direction in the frame of the target pose when driving to a specific pose. Defaults to 0.
   *
   * @return the integral constant for the PID controller for translational motion when driving to a
   *     specific pose
   */
  public double getDriveToPoseDriveYKI() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for translational motion in the x
   * direction in the frame of the target pose when driving to a specific pose. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for translational motion when driving to
   *     a specific pose
   */
  public double getDriveToPoseDriveXKD() {
    return 0.0;
  }

  /**
   * Returns the derivative constant for the PID controller for translational motion in the x
   * direction in the frame of the target pose when driving to a specific pose. Defaults to 0.
   *
   * @return the derivative constant for the PID controller for translational motion when driving to
   *     a specific pose
   */
  public double getDriveToPoseDriveYKD() {
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
   * drive-to-pose command. Defaults to the robot's maximum velocity.
   *
   * @return the maximum translational speed, in meters per second, for the robot during the
   *     drive-to-pose command
   */
  public LinearVelocity getDriveToPoseDriveMaxVelocity() {
    return getRobotMaxVelocity();
  }

  /**
   * Returns the maximum translational acceleration, in meters per second squared, for the robot
   * during the drive-to-pose command. Defaults to twice the robot's maximum velocity.
   *
   * @return the maximum translational acceleration, in meters per second squared, for the robot
   *     during the drive-to-pose command
   */
  public LinearAcceleration getDriveToPoseDriveMaxAcceleration() {
    return MetersPerSecondPerSecond.of(getRobotMaxVelocity().in(MetersPerSecond) * 2.0);
  }

  /**
   * Returns the maximum rotational speed, in radians per second, for the robot during the
   * drive-to-pose command. Defaults to the velocity derived from the maximum translational speed
   * and hte robot's geometry.
   *
   * @return the maximum rotational speed, in radians per second, for the robot during the
   *     drive-to-pose command
   */
  public AngularVelocity getDriveToPoseTurnMaxVelocity() {
    return RadiansPerSecond.of(
        getDriveToPoseDriveMaxVelocity().in(MetersPerSecond)
            / Math.hypot(getTrackwidth().in(Meters) / 2.0, getWheelbase().in(Meters) / 2.0));
  }

  /**
   * Returns the maximum rotational acceleration, in radians per second squared, for the robot
   * during the drive-to-pose command. Defaults to the acceleration derived from the maximum
   * translational acceleration and the robot's geometry.
   *
   * @return the maximum rotational acceleration, in radians per second squared, for the robot
   *     during the drive-to-pose command
   */
  public AngularAcceleration getDriveToPoseTurnMaxAcceleration() {
    return RadiansPerSecondPerSecond.of(
        getDriveToPoseDriveMaxAcceleration().in(MetersPerSecondPerSecond)
            / Math.hypot(getTrackwidth().in(Meters) / 2.0, getWheelbase().in(Meters) / 2.0));
  }

  /**
   * Returns the tolerance, in meters, for which the robot's position is considered at the specified
   * pose during the drive-to-pose command. Defaults to 0.
   *
   * @return the tolerance, in meters, for which the robot's position is considered at the specified
   *     pose during the drive-to-pose command
   */
  public Distance getDriveToPoseDriveTolerance() {
    return Meters.of(0.0);
  }

  /**
   * Returns the tolerance, in radians, for which the robot's heading is considered at the specified
   * pose during the drive-to-pose command. Defaults to 0.
   *
   * @return the tolerance, in radians, for which the robot's heading is considered at the specified
   *     pose during the drive-to-pose command
   */
  public Angle getDriveToPoseThetaTolerance() {
    return Radians.of(0.0);
  }

  /**
   * Returns the velocity, in meters per second, of the robot when driving into a field element
   * during a move-to-pose command. Defaults to 0.
   *
   * @return the velocity, in meters per second, of the robot when driving into a field element
   *     during a move-to-pose command
   */
  public LinearVelocity getMoveToPathFinalVelocity() {
    return MetersPerSecond.of(0);
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
   * Returns the Pigeon2 configuration for the swerve drivetrain. Defaults to an empty
   * configuration, which means the Pigeon2 is mounted in the default orientation.
   *
   * @return the Pigeon2 configuration for the swerve drivetrain
   */
  public Pigeon2Configuration getPigeonConfigForSwerveDrivetrain() {
    return new Pigeon2Configuration();
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

  /**
   * Returns the azimuth steer coupling ratio for the swerve drivetrain. Defaults to 0.
   *
   * @return the azimuth steer coupling ratio for the swerve drivetrain
   */
  public double getAzimuthSteerCouplingRatio() {
    return 0.0;
  }

  public enum DRIVETRAIN_TYPE {
    DIFFERENTIAL,
    SWERVE
  }

  public enum SWERVE_CONTROL_MODE {
    VOLTAGE,
    TORQUE_CURRENT_FOC
  }

  /**
   * Configuration for a camera used by the vision subsystem.
   *
   * <p>It contains the following fields:
   *
   * <ul>
   *   <li>robotToCameraTransform: The transform from the robot to the camera. (required)
   *   <li>poseForRobotToCameraTransformCalibration: The pose used for calibrating the robot to
   *       camera transform empirically.
   *   <li>id: The identifier for the camera. (required; specify the serial number of Basler cameras
   *       and the camera name for PhotonVision cameras)
   *   <li>location: The physical location of the camera on the robot; also used as part of the path
   *       in Network Tables. (required)
   *   <li>width: The width of the camera image in pixels. (required for Basler cameras)
   *   <li>height: The height of the camera image in pixels. (required for Basler cameras)
   *   <li>autoExposure: The auto exposure setting for the camera. (default: 0)
   *   <li>exposure: The exposure setting for the camera. (required for Basler cameras)
   *   <li>gain: The gain setting for the camera. (required for Basler cameras)
   *   <li>denoise: The denoise setting for the camera. (required for Basler cameras)
   *   <li>stdDevFactor: The standard deviation factor for filtering. (required; suggested: 1.0)
   * </ul>
   */
  @Builder
  public record CameraConfig(
      Transform3d robotToCameraTransform,
      Pose3d poseForRobotToCameraTransformCalibration,
      String id,
      String location,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor) {}
}
