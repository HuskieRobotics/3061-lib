package frc.lib.team3061.vision;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.nio.file.Path;

public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final Path APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2026-rebuilt-welded.json").toPath();
  public static final Path OFFICIAL_APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2026-rebuilt-welded.json").toPath();
  public static final int MAX_NUMBER_TAGS = 32;

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

  // the standard deviation of the error in pixels for the simulated camera
  public static final double SIM_ERROR_STD_DEV_PIXELS = 0.05;

  // the minimum confidence for a coral detection to be considered valid
  public static final double OBJECT_DETECT_CONFIDENCE_THRESHOLD = 0.80;
}
