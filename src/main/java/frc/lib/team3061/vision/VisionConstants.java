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
      new File(Filesystem.getDeployDirectory(), "home.json").toPath();
  // FIXME: update for the new season's file
  public static final Path OFFICIAL_APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2024-crescendo.json").toPath();
  public static final int MAX_NUMBER_TAGS = 30;

  public static final String SUBSYSTEM_NAME = "Vision";

  public static final double BEST_POSE_TIME_THRESHOLD_SECS = 0.5;

  public static final double TARGET_LOG_TIME_SECS = 0.1;

  // the pose ambiguity must be less than this value for the target to be considered valid
  public static final double AMBIGUITY_THRESHOLD = 0.5;

  // arbitrary for now, wait until testing on robot
  public static final double REPROJECTION_ERROR_THRESHOLD = 100.0;

  public static final double MAX_Z_ERROR_METERS = 0.75; // FIXME: tune

  public static final double FIELD_BORDER_MARGIN = 0.5;

  // this factor is applied to the pose ambiguity when calculating the standard deviation to pass to
  // the pose estimator
  public static final double AMBIGUITY_SCALE_FACTOR = 0.2;

  public static final double REPROJECTION_SCALE_FACTOR = 0.3;

  public static final double X_Y_STD_DEV_COEFFICIENT = 0.005;
  public static final double THETA_STD_DEV_COEFFICIENT = 0.01;

  // the maximum distance between the robot's pose derived from the target and the current robot's
  // estimated pose for the target to be used to update the robot's pose (essentially, always use a
  // valid target to update the robot's pose)
  public static final double MAX_POSE_DIFFERENCE_METERS = 2.5;

  // the maximum distance between the robot and the target, for the target to be used to update the
  // robot's pose
  public static final double MAX_DISTANCE_TO_TARGET_METERS = 6.0;

  // the maximum distance between the robot's current estimated pose and the robot's pose derived
  // from the target to consider the two poses as having converged.
  public static final double POSE_DIFFERENCE_THRESHOLD_METERS = 0.5;
}
