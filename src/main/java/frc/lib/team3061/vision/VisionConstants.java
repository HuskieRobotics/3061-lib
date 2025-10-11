package frc.lib.team3061.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.io.File;
import java.nio.file.Path;
import java.util.function.Supplier;
import lombok.Builder;

public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final Path APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2025-reefscape.json").toPath();
  public static final Path OFFICIAL_APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2025-reefscape.json").toPath();
  public static final int MAX_NUMBER_TAGS = 30;

  public static final String SUBSYSTEM_NAME = "Vision";

  public static final boolean ENABLE_DETAILED_LOGGING = false;
  public static final boolean CALIBRATE_CAMERA_TRANSFORMS = false;

  // an accepted pose must be within this many seconds of the current time in order to qualify for
  // resetting the robot's pose with the reset pose to vision command
  public static final double BEST_POSE_TIME_THRESHOLD_SECS = 0.5;

  // AprilTags that were seen more than this many seconds ago are considered stale and are not
  // logged
  public static final double TAG_LOG_TIME_SECS = 0.1;

  // Vision poses that were estimated more than this many seconds ago are considered stale and are
  // not logged
  public static final double POSE_LOG_TIME_SECS = 0.1;

  // the pose ambiguity must be less than this value for the target to be considered valid
  public static final double AMBIGUITY_THRESHOLD = 0.5;

  // the maximum difference, in degrees, between the robot's current rotation and the rotation
  // calculated from the vision target for the pose to be considered valid
  public static final double ROTATION_THRESHOLD_DEGREES = 10.0;

  // the reprojection error must be less than this value for the target to be considered valid
  public static final double REPROJECTION_ERROR_THRESHOLD = 5.0;

  // the maximum error in the z component of the robot's pose for the pose to be considered valid
  // (assumes that the robot is always on the carpet)
  public static final double MAX_Z_ERROR_METERS = 0.25;

  // the maximum distance off the field for the robot's pose for the pose to be considered valid
  public static final double FIELD_BORDER_MARGIN_METERS = 0.5;

  // this factor is applied to the pose ambiguity when calculating the standard deviation to pass to
  // the pose estimator (only used with single-tag estimation)
  public static final double AMBIGUITY_SCALE_FACTOR = 5.0;

  // this factor is applied to the pose reprojection error when calculating the standard deviation
  // to pass to the pose estimator (only used with multi-tag estimation)
  public static final double REPROJECTION_ERROR_SCALE_FACTOR = 3.33;

  // the coefficient from which the standard deviation for the x and y components is initiated
  public static final double X_Y_STD_DEV_COEFFICIENT = 0.08;

  // the coefficient from which the standard deviation for the theta component is initiated (only
  // used with multi-tag estimation)
  public static final double THETA_STD_DEV_COEFFICIENT = 0.1;

  // the average error in pixels for the simulated camera
  public static final double SIM_AVERAGE_ERROR_PIXELS = 0.1;

  // te standard deviation of the error in pixels for the simulated camera
  public static final double SIM_ERROR_STD_DEV_PIXELS = 0.05;

  private static final boolean forceEnableInstanceLogging = false;
  public static final boolean enableInstanceLogging =
      forceEnableInstanceLogging || Constants.getMode() == Mode.REPLAY;

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double demoTagPosePersistenceSecs = 0.5;
  public static final double coralDetectConfidenceThreshold = 0.35;
  public static final double algaeDetectConfidenceThreshold = 0.35;
  public static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", 0.0);

  private static int monoExposure = 2200;
  private static double monoGain = 0.0;
  private static double monoDenoise = 1.0;
  private static int colorExposure = 4500;
  private static double colorGain = 5.0;

  public static LoggedTunableNumber[] pitchAdjustments =
      switch (Constants.getRobot()) {
        case ROBOT_NORTHSTAR_TEST_PLATFORM -> new LoggedTunableNumber[] {
          new LoggedTunableNumber("Vision/PitchAdjust1", 0.0)
        };
        case ROBOT_PRACTICE -> new LoggedTunableNumber[] {
          new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
          new LoggedTunableNumber("Vision/PitchAdjust1", 0.0)
        };
        case ROBOT_COMPETITION -> new LoggedTunableNumber[] {
          new LoggedTunableNumber("Vision/PitchAdjust0", 0.0),
          new LoggedTunableNumber("Vision/PitchAdjust1", 0.0),
          new LoggedTunableNumber("Vision/PitchAdjust2", 0.0),
        };
        default -> new LoggedTunableNumber[] {};
      };
  public static CameraConfig[] cameras =
      switch (Constants.getRobot()) {
        case ROBOT_NORTHSTAR_TEST_PLATFORM -> new CameraConfig[] {
          CameraConfig.builder()
              .pose(() -> new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)))
              .id("40686739")
              .width(1600)
              .height(1200)
              .autoExposure(0)
              .exposure(monoExposure)
              .gain(monoGain)
              .stdDevFactor(1.0)
              .build(),
          CameraConfig.builder()
              .pose(() -> new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)))
              .id("25249734")
              .width(1600)
              .height(1200)
              .autoExposure(0)
              .exposure(monoExposure)
              .gain(monoGain)
              .stdDevFactor(1.0)
              .build()
        };
        case ROBOT_PRACTICE -> new CameraConfig[] {
          CameraConfig.builder()
              .pose(
                  () ->
                      new Pose3d(
                          0.254,
                          0.2032,
                          0.21113,
                          new Rotation3d(
                              0.0,
                              Units.degreesToRadians(-25.0 + pitchAdjustments[0].get()),
                              Units.degreesToRadians(-20.0))))
              .id("40265450")
              .width(1600)
              .height(1200)
              .exposure(monoExposure)
              .gain(monoGain)
              .stdDevFactor(1.0)
              .build(),
          CameraConfig.builder()
              .pose(
                  () ->
                      new Pose3d(
                          0.254,
                          -0.2032,
                          0.21113,
                          new Rotation3d(
                              0.0,
                              Units.degreesToRadians(-25.0 + pitchAdjustments[1].get()),
                              Units.degreesToRadians(20.0))))
              .id("40265453")
              .width(1600)
              .height(1200)
              .exposure(monoExposure)
              .gain(monoGain)
              .stdDevFactor(1.0)
              .build()
        };
        case ROBOT_COMPETITION -> new CameraConfig[] {
          // Front Left
          CameraConfig.builder()
              .pose(
                  () ->
                      new Pose3d(
                          0.2794,
                          0.2286,
                          0.21113,
                          new Rotation3d(
                              0.0,
                              Units.degreesToRadians(-25.0 + pitchAdjustments[0].get()),
                              Units.degreesToRadians(-20.0))))
              .id("40530395")
              .width(1600)
              .height(1200)
              .exposure(monoExposure)
              .gain(monoGain)
              .denoise(monoDenoise)
              .stdDevFactor(1.0)
              .build(),
          // Front Right
          CameraConfig.builder()
              .pose(
                  () ->
                      new Pose3d(
                          0.2794,
                          -0.2286,
                          0.21113,
                          new Rotation3d(
                              0.0,
                              Units.degreesToRadians(-25.0 + pitchAdjustments[1].get()),
                              Units.degreesToRadians(20.0))))
              .id("40552081")
              .width(1600)
              .height(1200)
              .exposure(monoExposure)
              .gain(monoGain)
              .denoise(monoDenoise)
              .stdDevFactor(1.0)
              .build(),
          // Back Up
          CameraConfig.builder()
              .pose(
                  () ->
                      new Pose3d(
                          Units.inchesToMeters(3.209),
                          Units.inchesToMeters(-10),
                          Units.inchesToMeters(26.485),
                          new Rotation3d(
                              0.0,
                              Units.degreesToRadians(25.0 + pitchAdjustments[2].get()),
                              Units.degreesToRadians(163.5))))
              .id("24737133")
              .width(1280)
              .height(960)
              .exposure(colorExposure)
              .gain(colorGain)
              .stdDevFactor(1.25)
              .build(),
        };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Supplier<Pose3d> pose,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor) {}
}
