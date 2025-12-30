package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  private static final String CAMERA_NAME_0 = "40686739";
  private static final String CAMERA_NAME_1 = "40708569";
  private static final String CAMERA_NAME_2 = "40708556";
  private static final String CAMERA_NAME_3 = "40708542";
  private static final String CAMERA_NAME_4 = "25249734";

  private static final int MONO_EXPOSURE = 2200;
  private static final double MONO_GAIN = 17.5;
  private static final double MONO_DENOISE = 1.0;

  private static final int COLOR_EXPOSURE = 4500;
  private static final double COLOR_GAIN = 5.0;

  // Front right camera
  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(
          new Translation3d(0.0, 0.0, 0.0), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 0.0)));

  // Back right camera
  private static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          new Translation3d(0.0, 0.0, 0.0), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 0.0)));

  // Back right camera
  private static final Transform3d ROBOT_TO_CAMERA_2 =
      new Transform3d(
          new Translation3d(0.0, 0.0, 0.0), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 0.0)));

  // Back right camera
  private static final Transform3d ROBOT_TO_CAMERA_3 =
      new Transform3d(
          new Translation3d(0.0, 0.0, 0.0), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 0.0)));

  // color camera
  private static final Transform3d ROBOT_TO_CAMERA_4 =
      new Transform3d(
          new Translation3d(0.0, 0.0, 0.0), new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 0.0)));

  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

  @Override
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_0)
          .id(CAMERA_NAME_0)
          .location("FR")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_1)
          .id(CAMERA_NAME_1)
          .location("BR")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_2)
          .id(CAMERA_NAME_2)
          .location("FL")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_3)
          .id(CAMERA_NAME_3)
          .location("BL")
          .width(1920)
          .height(1200)
          .exposure(MONO_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(MONO_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_4)
          .id(CAMERA_NAME_4)
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
