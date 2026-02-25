package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class NorthstarTestPlatformConfig extends RobotConfig {
  private static final String BR_CAMERA_SERIAL_NUMBER = "40686739";
  private static final String BL_CAMERA_SERIAL_NUMBER = "40708556";
  private static final String CENTER_CAMERA_SERIAL_NUMBER = "25249734";

  private static final int MONO_EXPOSURE = 2200;
  private static final double MONO_GAIN = 17.5;
  private static final double MONO_DENOISE = 1.0;

  private static final int COLOR_EXPOSURE = 4500;
  private static final double COLOR_GAIN = 5.0;

  // Back right camera
  // x, y, z, pitch, yaw
  private static final Transform3d ROBOT_TO_BR_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.969),
              Units.inchesToMeters(-11.729),
              Units.inchesToMeters(7.434)),
          new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(-90.0)));

  // Back left camera
  // x, y, z, pitch, yaw
  private static final Transform3d ROBOT_TO_BL_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.969),
              Units.inchesToMeters(11.729),
              Units.inchesToMeters(7.434)),
          new Rotation3d(0, Units.degreesToRadians(-25), Units.degreesToRadians(90.0)));

  // Center camera
  // x, y, z, pitch, yaw
  private static final Transform3d ROBOT_TO_CENTER_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
          new Rotation3d(0, Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)));

  @Override
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BR_CAMERA)
          .id(BR_CAMERA_SERIAL_NUMBER)
          .location("BR")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BL_CAMERA)
          .id(BL_CAMERA_SERIAL_NUMBER)
          .location("BL")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CENTER_CAMERA)
          .id(CENTER_CAMERA_SERIAL_NUMBER)
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
  public double getOdometryUpdateFrequency() {
    return 250.0;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }

  @Override
  public LinearVelocity getRobotMaxVelocity() {
    return MetersPerSecond.of(0.0);
  }

  @Override
  public SwerveConstants getSwerveConstants() {
    return SwerveConstants.MK4I_L3_PLUS_CONSTANTS;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {};
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {};
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {};
  }

  @Override
  public Angle[] getSwerveSteerOffsets() {
    return new Angle[] {};
  }

  @Override
  public int getGyroCANID() {
    return 0;
  }

  @Override
  public Distance getTrackwidth() {
    return Meters.of(0.0);
  }

  @Override
  public Distance getWheelbase() {
    return Meters.of(0.0);
  }

  @Override
  public Distance getWheelRadius() {
    return Meters.of(0.0);
  }
}
