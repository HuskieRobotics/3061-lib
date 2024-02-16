package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * The Vision subsystem is responsible for updating the robot's estimated pose based on a collection
 * of cameras capturing AprilTags. The Vision subsystem is comprised of multiple VisionIO objects,
 * each of which is responsible for producing a single PhotonPipelineResult. There is a one-to-one
 * relationship between each VisionIO object and each co-processor (e.g., Raspberry Pi) running
 * PhotonVision.
 */
public class Vision extends SubsystemBase {
  private static final int EXPIRATION_COUNT = 5;

  private VisionIO[] visionIOs;
  private final VisionIOInputsAutoLogged[] ios;
  private double[] lastTimestamps;
  private final Pose2d[] detectedAprilTags;
  private int[] cyclesWithNoResults;

  private AprilTagFieldLayout layout;
  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  private boolean isEnabled = true;
  private boolean isVisionUpdating = false;

  private RobotOdometry odometry;
  private final TunableNumber poseDifferenceThreshold =
      new TunableNumber("Vision/VisionPoseThreshold", POSE_DIFFERENCE_THRESHOLD_METERS);
  private final TunableNumber stdDevSlope = new TunableNumber("Vision/stdDevSlope", 0.10);
  private final TunableNumber stdDevPower = new TunableNumber("Vision/stdDevPower", 2.0);
  private final TunableNumber stdDevMultiTagFactor =
      new TunableNumber("Vision/stdDevMultiTagFactor", 0.2);

  /**
   * Create a new Vision subsystem. The number of VisionIO objects passed to the constructor must
   * match the number of robot-to-camera transforms returned by the RobotConfig singleton.
   *
   * @param visionIO One or more VisionIO objects, each of which is responsible for producing a
   *     single single PhotonPipelineResult. There is a one-to-one relationship between each
   *     VisionIO object and each co-processor (e.g., Raspberry Pi) running PhotonVision.
   */
  public Vision(VisionIO[] visionIOs) {
    this.visionIOs = visionIOs;
    this.lastTimestamps = new double[visionIOs.length];
    this.cyclesWithNoResults = new int[visionIOs.length];
    this.ios = new VisionIOInputsAutoLogged[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      this.ios[i] = new VisionIOInputsAutoLogged();
    }

    // retrieve a reference to the pose estimator singleton
    this.odometry = RobotOdometry.getInstance();

    // add an indicator to the main Shuffleboard tab to indicate whether vision is updating in order
    // to alert the drive team if it is not.
    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    tabMain
        .addBoolean("isVisionUpdating", () -> isVisionUpdating)
        .withPosition(7, 2)
        .withSize(1, 2);

    // load and log all of the AprilTags in the field layout file
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      noAprilTagLayoutAlert.set(false);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      noAprilTagLayoutAlert.set(true);
    }

    for (AprilTag tag : layout.getTags()) {
      Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTags/" + tag.ID, tag.pose);
    }

    // index corresponds to tag ID; so, add 1 since there is no tag ID 0
    this.detectedAprilTags = new Pose2d[this.layout.getTags().size() + 1];
    for (int i = 0; i < this.detectedAprilTags.length; i++) {
      this.detectedAprilTags[i] = new Pose2d();
    }
  }

  /**
   * This method is invoked each iteration of the scheduler. It updates the inputs for each of the
   * VisionIO objects and, for each, updates the pose estimator based on the most recent detected
   * AprilTags.
   */
  @Override
  public void periodic() {
    isVisionUpdating = false;
    for (int i = 0; i < visionIOs.length; i++) {

      visionIOs[i].updateInputs(ios[i]);
      Logger.processInputs(SUBSYSTEM_NAME + "/" + i, ios[i]);

      processNewVisionData(i);
    }

    // set the pose of all the tags to the current robot pose such that no vision target lines are
    // displayed in AdvantageScope
    for (int tagIndex = 0; tagIndex < this.detectedAprilTags.length; tagIndex++) {
      this.detectedAprilTags[tagIndex] = odometry.getEstimatedPosition();
    }

    for (int visionIndex = 0; visionIndex < visionIOs.length; visionIndex++) {
      for (int tagID = 1; tagID < ios[visionIndex].tagsSeen.length; tagID++) {
        if (ios[visionIndex].tagsSeen[tagID]) {
          this.detectedAprilTags[tagID] =
              this.layout.getTagPose(tagID).orElse(new Pose3d()).toPose2d();
        }
      }
    }
    Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTags", this.detectedAprilTags);

    Logger.recordOutput(SUBSYSTEM_NAME + "/IsEnabled", isEnabled);
  }

  private void processNewVisionData(int i) {
    // only process the vision data if the timestamp is newer than the last one
    if (this.lastTimestamps[i] < ios[i].lastCameraTimestamp) {
      this.lastTimestamps[i] = ios[i].lastCameraTimestamp;
      Pose3d estimatedRobotPose3d =
          ios[i].estimatedCameraPose.plus(
              RobotConfig.getInstance().getRobotToCameraTransforms()[i].inverse());
      Pose2d estimatedRobotPose2d = estimatedRobotPose3d.toPose2d();

      // only update the pose estimator if the vision subsystem is enabled
      if (isEnabled) {
        // when updating the pose estimator, specify standard deviations based on the distance
        // from the robot to the AprilTag (the greater the distance, the less confident we are
        // in the measurement)
        odometry.addVisionMeasurement(
            estimatedRobotPose2d,
            ios[i].estimatedCameraPoseTimestamp,
            getStandardDeviations(i, estimatedRobotPose2d));
        isVisionUpdating = true;
      }

      Logger.recordOutput(SUBSYSTEM_NAME + "/" + i + "/CameraPose3d", ios[i].estimatedCameraPose);
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + i + "/CameraPose2d", ios[i].estimatedCameraPose.toPose2d());
      Logger.recordOutput(SUBSYSTEM_NAME + "/" + i + "/RobotPose3d", estimatedRobotPose3d);
      Logger.recordOutput(SUBSYSTEM_NAME + "/" + i + "/RobotPose2d", estimatedRobotPose2d);
      this.cyclesWithNoResults[i] = 0;
    } else {
      this.cyclesWithNoResults[i] += 1;
    }

    // if no tags have been seen for the specified number of cycles, "zero" the robot pose
    // such that old data is not seen in AdvantageScope
    if (cyclesWithNoResults[i] == EXPIRATION_COUNT) {
      Logger.recordOutput(SUBSYSTEM_NAME + "/" + i + "/RobotPose", new Pose2d());
    }
  }

  /**
   * Returns true if the vision subsystem is enabled.
   *
   * @return true if the vision subsystem is enabled
   */
  public boolean isEnabled() {
    return isEnabled;
  }

  /**
   * Returns the estimated robot pose based on the most recent vision data. This method is used to
   * reset the robot's odometry based solely on the vision data.
   *
   * @return the estimated robot pose based on the most recent vision data
   */
  public Pose3d getBestRobotPose() {
    Pose3d robotPoseFromMostRecentData = null;
    double mostRecentTimestamp = 0.0;
    for (int i = 0; i < visionIOs.length; i++) {
      if (ios[i].estimatedCameraPoseTimestamp > mostRecentTimestamp) {
        robotPoseFromMostRecentData = ios[i].estimatedCameraPose;
        mostRecentTimestamp = ios[i].estimatedCameraPoseTimestamp;
      }
    }
    return robotPoseFromMostRecentData;
  }

  /**
   * Enable or disable the vision subsystem.
   *
   * @param enable enables the vision subsystem if true; disables if false
   */
  public void enable(boolean enable) {
    isEnabled = enable;
  }

  /**
   * Returns true if the robot's pose based on vision data is within the specified threshold of the
   * robot's pose based on the pose estimator. This method can be used to trigger a transition from
   * driver control to automated control once confident that the estimated pose is accurate.
   *
   * @return true if the robot's pose based on vision data is within the specified threshold of the
   *     robot's pose based on the pose estimator
   */
  public boolean posesHaveConverged() {
    for (int i = 0; i < visionIOs.length; i++) {
      Pose3d robotPose = ios[i].estimatedCameraPose;
      if (odometry.getEstimatedPosition().minus(robotPose.toPose2d()).getTranslation().getNorm()
          < poseDifferenceThreshold.get()) {
        Logger.recordOutput(SUBSYSTEM_NAME + "/posesInLine", true);
        return true;
      }
    }
    Logger.recordOutput(SUBSYSTEM_NAME + "/posesInLine", false);
    return false;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  private Matrix<N3, N1> getStandardDeviations(int index, Pose2d estimatedPose) {
    Matrix<N3, N1> estStdDevs = VecBuilder.fill(1, 1, 2);
    int numTags = 0;
    double avgDist = 0;
    for (int tagID = 0; tagID < ios[index].tagsSeen.length; tagID++) {
      Optional<Pose3d> tagPose = layout.getTagPose(tagID);
      if (!ios[index].tagsSeen[tagID] || tagPose.isEmpty()) {
        continue;
      }
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = estStdDevs.times(stdDevMultiTagFactor.get());
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > MAX_DISTANCE_TO_TARGET_METERS) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      estStdDevs = estStdDevs.times(stdDevSlope.get() * (Math.pow(avgDist, stdDevPower.get())));
    }

    return estStdDevs;
  }
}
