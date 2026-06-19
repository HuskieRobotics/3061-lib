package frc.robot.configs;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;
import frc.lib.team6328.util.FieldConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class NorthstarTestPlatformConfig extends RobotConfig {
  private static final String BR_CAMERA_SERIAL_NUMBER = "40708542";
  private static final String BL_CAMERA_SERIAL_NUMBER = "40708556";
  private static final String BCL_CAMERA_SERIAL_NUMBER = "24608727";
  private static final String BCH_CAMERA_SERIAL_NUMBER = "40708569";

  private static final int DAA1920_160UM_EXPOSURE = 2200;
  private static final int DAA1280_54UM_EXPOSURE = 1100;
  private static final double MONO_GAIN = 15;
  private static final double DAA1920_160UM_DENOISE = 1.0;
  private static final double DAA1280_54UM_DENOISE = 0.0;

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

  // Back center left camera
  // x, y, z, pitch, yaw
  private static final Transform3d ROBOT_TO_BCL_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11.5),
              Units.inchesToMeters(6.1725),
              Units.inchesToMeters(11.569)),
          new Rotation3d(0, Units.degreesToRadians(-10.00), Units.degreesToRadians(160.0)));

  // Back center right camera
  // x, y, z, pitch, yaw
  private static final Transform3d ROBOT_TO_BCH_CAMERA =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11.5),
              Units.inchesToMeters(4.0415),
              Units.inchesToMeters(11.569)),
          new Rotation3d(0, Units.degreesToRadians(-10.00), Units.degreesToRadians(200.0)));

  // use AprilTag ID 13 for empirical determination of the robot-to-camera transform
  private static final Pose3d ROBOT_TO_TAG_13_BACK_CAMERAS =
      FieldConstants.defaultAprilTagType
          .getLayout()
          .getTagPose(13)
          .get()
          .transformBy(
              new Transform3d(
                  Units.inchesToMeters(15.125),
                  Units.inchesToMeters(0.0),
                  -Units.inchesToMeters(9.625),
                  new Rotation3d()));

  private static final Pose3d ROBOT_TO_TAG_13_LEFT_CAMERA =
      FieldConstants.defaultAprilTagType
          .getLayout()
          .getTagPose(13)
          .get()
          .transformBy(
              new Transform3d(
                  Units.inchesToMeters(15.125),
                  Units.inchesToMeters(18.125),
                  -Units.inchesToMeters(9.625),
                  new Rotation3d(0, 0, Units.degreesToRadians(90))));

  private static final Pose3d ROBOT_TO_TAG_13_RIGHT_CAMERA =
      FieldConstants.defaultAprilTagType
          .getLayout()
          .getTagPose(13)
          .get()
          .transformBy(
              new Transform3d(
                  Units.inchesToMeters(15.125),
                  -Units.inchesToMeters(18.125),
                  -Units.inchesToMeters(21.75),
                  new Rotation3d(0, 0, Units.degreesToRadians(-90))));

  @Override
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BR_CAMERA)
          .poseForRobotToCameraTransformCalibration(ROBOT_TO_TAG_13_RIGHT_CAMERA)
          .id(BR_CAMERA_SERIAL_NUMBER)
          .location("BR")
          .width(1800)
          .height(1200)
          .exposure(DAA1920_160UM_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(DAA1920_160UM_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BL_CAMERA)
          .poseForRobotToCameraTransformCalibration(ROBOT_TO_TAG_13_LEFT_CAMERA)
          .id(BL_CAMERA_SERIAL_NUMBER)
          .location("BL")
          .width(1800)
          .height(1200)
          .exposure(DAA1920_160UM_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(DAA1920_160UM_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BCL_CAMERA)
          .poseForRobotToCameraTransformCalibration(ROBOT_TO_TAG_13_BACK_CAMERAS)
          .id(BCL_CAMERA_SERIAL_NUMBER)
          .location("BCL")
          .width(1280)
          .height(960)
          .exposure(DAA1280_54UM_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(DAA1280_54UM_DENOISE)
          .stdDevFactor(1.0)
          .build(),
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_BCH_CAMERA)
          .poseForRobotToCameraTransformCalibration(ROBOT_TO_TAG_13_BACK_CAMERAS)
          .id(BCH_CAMERA_SERIAL_NUMBER)
          .location("BCH")
          .width(1800)
          .height(1200)
          .exposure(DAA1920_160UM_EXPOSURE)
          .gain(MONO_GAIN)
          .denoise(DAA1920_160UM_DENOISE)
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
  public double getRobotMaxVelocityMPS() {
    return 0.0;
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
  public double[] getSwerveSteerOffsetsRots() {
    return new double[] {};
  }

  @Override
  public int getGyroCANID() {
    return 0;
  }

  @Override
  public double getTrackwidthMeters() {
    return 0.0;
  }

  @Override
  public double getWheelbaseMeters() {
    return 0.0;
  }

  @Override
  public double getWheelRadiusMeters() {
    return 0.0;
  }
}
