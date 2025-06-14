package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.VisionIO.PoseObservation;
import frc.lib.team3061.vision.VisionIO.PoseObservationType;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
  private VisionIO[] visionIOs;
  private final VisionIOInputsAutoLogged[] inputs;
  private double[] lastTimestamps;
  private int[] cyclesWithNoResults;
  private int[] updatePoseCount;
  private Alert[] disconnectedAlerts;

  private List<Integer> camerasToConsider = new ArrayList<>();

  private AprilTagFieldLayout layout;
  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.kWarning);
  private final Alert unofficialAprilTagLayoutAlert = new Alert("", AlertType.kInfo);

  private boolean isEnabled = true;
  private boolean isVisionUpdating = false;
  private final Debouncer isVisionUpdatingDebounce =
      new Debouncer(0.1, Debouncer.DebounceType.kFalling);

  private RobotOdometry odometry;

  private Pose3d mostRecentBestPose = new Pose3d();
  private double mostRecentBestPoseTimestamp = 0.0;
  private double mostRecentBestPoseStdDev = 0.0;

  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  private final Map<Pose3d, Double> lastPoseEstimationAcceptedTimes = new HashMap<>();
  private final Map<Pose3d, Double> lastPoseEstimationRejectedTimes = new HashMap<>();

  private List<Pose3d> allRobotPoses = new ArrayList<>();
  private List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
  private List<Pose3d> allRobotPosesRejected = new ArrayList<>();
  private List<Pose3d> allTagPoses = new ArrayList<>();

  private List<List<Pose3d>> tagPoses;
  private List<List<Pose3d>> cameraPoses;
  private List<List<Pose3d>> robotPoses;
  private List<List<Pose3d>> robotPosesAccepted;
  private List<List<Pose3d>> robotPosesRejected;

  private final Pose3d[] robotPosesForCalibration;

  private final LoggedTunableNumber latencyAdjustmentSeconds =
      new LoggedTunableNumber("Vision/LatencyAdjustmentSeconds", 0.0);
  private final LoggedTunableNumber ambiguityScaleFactor =
      new LoggedTunableNumber("Vision/AmbiguityScaleFactor", AMBIGUITY_SCALE_FACTOR);
  private final LoggedTunableNumber reprojectionErrorScaleFactor =
      new LoggedTunableNumber(
          "Vision/ReprojectionErrorScaleFactor", REPROJECTION_ERROR_SCALE_FACTOR);
  private final LoggedTunableNumber xyStdDevCoefficient =
      new LoggedTunableNumber("Vision/XYStdDevCoefficient", X_Y_STD_DEV_COEFFICIENT);
  private final LoggedTunableNumber thetaStdDevCoefficient =
      new LoggedTunableNumber("Vision/ThetaStdDevCoefficient", THETA_STD_DEV_COEFFICIENT);

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
    this.updatePoseCount = new int[visionIOs.length];
    this.inputs = new VisionIOInputsAutoLogged[visionIOs.length];
    this.disconnectedAlerts = new Alert[visionIOs.length];
    this.camerasToConsider = new ArrayList<>();

    tagPoses = new ArrayList<List<Pose3d>>(visionIOs.length);
    cameraPoses = new ArrayList<List<Pose3d>>(visionIOs.length);
    robotPoses = new ArrayList<List<Pose3d>>(visionIOs.length);
    robotPosesAccepted = new ArrayList<List<Pose3d>>(visionIOs.length);
    robotPosesRejected = new ArrayList<List<Pose3d>>(visionIOs.length);

    for (int i = 0; i < visionIOs.length; i++) {
      this.inputs[i] = new VisionIOInputsAutoLogged();
      this.disconnectedAlerts[i] = new Alert("camera" + i + " is disconnected", AlertType.kError);
      this.camerasToConsider.add(i);

      tagPoses.add(new ArrayList<>());
      cameraPoses.add(new ArrayList<>());
      robotPoses.add(new ArrayList<>());
      robotPosesAccepted.add(new ArrayList<>());
      robotPosesRejected.add(new ArrayList<>());
    }

    // retrieve a reference to the pose estimator singleton
    this.odometry = RobotOdometry.getInstance();

    // load and log all of the AprilTags in the field layout file
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      noAprilTagLayoutAlert.set(false);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      noAprilTagLayoutAlert.set(true);
    }

    // AprilTag layout alert
    if (!APRILTAG_FIELD_LAYOUT_PATH.equals(OFFICIAL_APRILTAG_FIELD_LAYOUT_PATH)) {
      unofficialAprilTagLayoutAlert.set(true);
      unofficialAprilTagLayoutAlert.setText(
          "Unofficial AprilTag layout in use ("
              + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH.toString()
              + ").");
    }

    Pose3d[] aprilTagsPoses = new Pose3d[this.layout.getTags().size()];
    for (int i = 0; i < aprilTagsPoses.length; i++) {
      aprilTagsPoses[i] = this.layout.getTags().get(i).pose;
    }
    Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTagsPoses", aprilTagsPoses);

    // robot to camera transformation calibration
    robotPosesForCalibration =
        RobotConfig.getInstance().getPosesForRobotToCameraTransformCalibration();
  }

  /**
   * This method is invoked each iteration of the scheduler. It updates the inputs for each of the
   * VisionIO objects and, for each, updates the pose estimator based on the most recent detected
   * AprilTags.
   */
  @Override
  public void periodic() {
    isVisionUpdating = false;

    for (int cameraIndex = 0; cameraIndex < visionIOs.length; cameraIndex++) {
      visionIOs[cameraIndex].updateInputs(inputs[cameraIndex]);
      Logger.processInputs(SUBSYSTEM_NAME + "/" + cameraIndex, inputs[cameraIndex]);
    }

    this.allRobotPoses.clear();
    this.allRobotPosesAccepted.clear();
    this.allRobotPosesRejected.clear();
    this.allTagPoses.clear();

    for (int cameraIndex = 0; cameraIndex < visionIOs.length; cameraIndex++) {

      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);
      this.cyclesWithNoResults[cameraIndex] += 1;

      // Initialize logging values
      tagPoses.get(cameraIndex).clear();
      cameraPoses.get(cameraIndex).clear();
      robotPoses.get(cameraIndex).clear();
      robotPosesAccepted.get(cameraIndex).clear();
      robotPosesRejected.get(cameraIndex).clear();

      for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
        // only process the vision data if the timestamp is newer than the last one
        if (this.lastTimestamps[cameraIndex] < observation.timestamp()) {

          if (CALIBRATE_CAMERA_TRANSFORMS) {
            logCameraTransforms(cameraIndex, observation);
          }

          // Initialize logging values
          this.lastTimestamps[cameraIndex] = observation.timestamp();
          cameraPoses.get(cameraIndex).add(observation.cameraPose());
          Pose3d estimatedRobotPose3d =
              observation
                  .cameraPose()
                  .plus(
                      RobotConfig.getInstance()
                          .getRobotToCameraTransforms()[cameraIndex]
                          .inverse());
          Pose2d estimatedRobotPose2d = estimatedRobotPose3d.toPose2d();
          robotPoses.get(cameraIndex).add(estimatedRobotPose3d);

          // only update the pose estimator if the vision subsystem is enabled and the vision's
          // estimated pose is on (or very close to) the field
          // for multi-tag strategies, ensure the reprojection error is less than the threshold; for
          // single-tag, ensure the ambiguity is less than the threshold.
          boolean acceptPose =
              isEnabled
                  && this.camerasToConsider.contains(cameraIndex)
                  && (observation.type() == PoseObservationType.MULTI_TAG
                      || observation.averageAmbiguity() < AMBIGUITY_THRESHOLD)
                  && (observation.type() == PoseObservationType.SINGLE_TAG
                      || Math.abs(observation.reprojectionError()) < REPROJECTION_ERROR_THRESHOLD)
                  && poseIsOnField(estimatedRobotPose3d)
                  && arePoseRotationsReasonable(estimatedRobotPose3d);

          if (acceptPose) {
            // get tag poses and update last detection times
            final int finalCameraIndex = cameraIndex;
            for (int tagID = 1; tagID < MAX_NUMBER_TAGS; tagID++) {
              if ((observation.tagsSeenBitMap() & (1L << tagID)) != 0) {
                if (ENABLE_DETAILED_LOGGING) {
                  lastTagDetectionTimes.put(tagID, Timer.getTimestamp());
                }
                Optional<Pose3d> tagPose = this.layout.getTagPose(tagID);
                tagPose.ifPresent(
                    (e) -> {
                      tagPoses.get(finalCameraIndex).add(e);
                    });
              }
            }
            robotPosesAccepted.get(cameraIndex).add(estimatedRobotPose3d);
            if (ENABLE_DETAILED_LOGGING) {
              lastPoseEstimationAcceptedTimes.put(estimatedRobotPose3d, Timer.getTimestamp());
            }

            Matrix<N3, N1> stdDev = getStandardDeviations(cameraIndex, observation);
            odometry.addVisionMeasurement(
                estimatedRobotPose2d,
                observation.timestamp(),
                latencyAdjustmentSeconds.get(),
                stdDev);

            // if the most-recent "best pose" is too old, capture a new one regardless of its
            // standard deviation values; otherwise, only capture a new one if its standard
            // deviation is lower than the current best pose
            if (mostRecentBestPoseTimestamp < Timer.getTimestamp() - BEST_POSE_TIME_THRESHOLD_SECS
                || mostRecentBestPoseStdDev > stdDev.get(0, 0)) {
              mostRecentBestPose = estimatedRobotPose3d;
              mostRecentBestPoseTimestamp = observation.timestamp();
              mostRecentBestPoseStdDev = stdDev.get(0, 0);
            }

            this.updatePoseCount[cameraIndex]++;

            // if there are multiple observations, only the last will be logged, which is fine
            Logger.recordOutput(
                SUBSYSTEM_NAME + "/" + cameraIndex + "/UpdatePoseCount",
                this.updatePoseCount[cameraIndex]);
            Logger.recordOutput(SUBSYSTEM_NAME + "/" + cameraIndex + "/StdDevX", stdDev.get(0, 0));
            Logger.recordOutput(SUBSYSTEM_NAME + "/" + cameraIndex + "/StdDevY", stdDev.get(1, 0));
            Logger.recordOutput(SUBSYSTEM_NAME + "/" + cameraIndex + "/StdDevT", stdDev.get(2, 0));
          } else {
            robotPosesRejected.get(cameraIndex).add(estimatedRobotPose3d);
            if (ENABLE_DETAILED_LOGGING) {
              lastPoseEstimationRejectedTimes.put(estimatedRobotPose3d, Timer.getTimestamp());
            }
          }
          this.cyclesWithNoResults[cameraIndex] = 0;
          isVisionUpdating = true;
        }
      }

      // Log data for this camera
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + cameraIndex + "/LatencySecs",
          Timer.getTimestamp() - this.lastTimestamps[cameraIndex]);
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + cameraIndex + "/CyclesWithNoResults",
          this.cyclesWithNoResults[cameraIndex]);
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + cameraIndex + "/TagPoses",
          tagPoses.get(cameraIndex).toArray(Pose3d[]::new));
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + cameraIndex + "/CameraPoses",
          cameraPoses.get(cameraIndex).toArray(new Pose3d[cameraPoses.get(cameraIndex).size()]));
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + cameraIndex + "/RobotPoses",
          robotPoses.get(cameraIndex).toArray(new Pose3d[robotPoses.get(cameraIndex).size()]));
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted
              .get(cameraIndex)
              .toArray(new Pose3d[robotPosesAccepted.get(cameraIndex).size()]));
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected
              .get(cameraIndex)
              .toArray(new Pose3d[robotPosesRejected.get(cameraIndex).size()]));
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/" + cameraIndex + "/CameraAxes",
          new Pose3d(RobotOdometry.getInstance().getEstimatedPose())
              .plus(RobotConfig.getInstance().getRobotToCameraTransforms()[cameraIndex]));

      if (!ENABLE_DETAILED_LOGGING) {
        allRobotPosesAccepted.addAll(robotPosesAccepted.get(cameraIndex));
        allRobotPosesRejected.addAll(robotPosesRejected.get(cameraIndex));
        allTagPoses.addAll(tagPoses.get(cameraIndex));
      }
    }

    // Log summary data
    if (ENABLE_DETAILED_LOGGING) {
      for (Map.Entry<Pose3d, Double> entry : lastPoseEstimationAcceptedTimes.entrySet()) {
        if (Timer.getTimestamp() - entry.getValue() < POSE_LOG_TIME_SECS) {
          allRobotPosesAccepted.add(entry.getKey());
        }
      }
    }
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));

    if (ENABLE_DETAILED_LOGGING) {
      for (Map.Entry<Pose3d, Double> entry : lastPoseEstimationRejectedTimes.entrySet()) {
        if (Timer.getTimestamp() - entry.getValue() < POSE_LOG_TIME_SECS) {
          allRobotPosesRejected.add(entry.getKey());
        }
      }
    }
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    allRobotPoses.addAll(allRobotPosesAccepted);
    allRobotPoses.addAll(allRobotPosesRejected);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));

    // Log tag poses
    if (ENABLE_DETAILED_LOGGING) {
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (Timer.getTimestamp() - detectionEntry.getValue() < TAG_LOG_TIME_SECS) {
          layout.getTagPose(detectionEntry.getKey()).ifPresent(allTagPoses::add);
        }
      }
    }
    Logger.recordOutput(SUBSYSTEM_NAME + "/AprilTags", allTagPoses.toArray(Pose3d[]::new));

    Logger.recordOutput(SUBSYSTEM_NAME + "/IsEnabled", isEnabled);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/IsUpdating", isVisionUpdatingDebounce.calculate(isVisionUpdating));

    Logger.recordOutput(SUBSYSTEM_NAME + "/CamerasToConsider", camerasToConsider.toString());

    // Record cycle time
    LoggedTracer.record("Vision");
  }

  public void specifyCamerasToConsider(List<Integer> cameraIndices) {
    this.camerasToConsider = cameraIndices;
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
    if (mostRecentBestPoseTimestamp > Timer.getTimestamp() - BEST_POSE_TIME_THRESHOLD_SECS) {
      return mostRecentBestPose;
    } else {
      return null;
    }
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
   * Returns true if the specified pose is on the field (or within FIELD_BORDER_MARGIN_METERS in the
   * x and y directions and MAX_Z_ERROR_METERS in the z direction).
   *
   * @param pose the pose to check
   * @return true if the specified pose is on the field (or very close to it)
   */
  private boolean poseIsOnField(Pose3d pose) {
    return pose.getX() > -FIELD_BORDER_MARGIN_METERS
        && pose.getX() < layout.getFieldLength() + FIELD_BORDER_MARGIN_METERS
        && pose.getY() > -FIELD_BORDER_MARGIN_METERS
        && pose.getY() < layout.getFieldWidth() + FIELD_BORDER_MARGIN_METERS
        && Math.abs(pose.getZ()) < MAX_Z_ERROR_METERS;
  }

  private boolean arePoseRotationsReasonable(Pose3d pose) {
    return Math.abs(
            RobotOdometry.getInstance()
                .getEstimatedPose()
                .getRotation()
                .minus(pose.getRotation().toRotation2d())
                .getRadians())
        < ROTATION_THRESHOLD_DEGREES;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  private Matrix<N3, N1> getStandardDeviations(int index, PoseObservation observation) {
    double xyStdDev =
        xyStdDevCoefficient.get()
            * Math.pow(observation.averageTagDistance(), 2.0)
            / observation.numTags()
            * RobotConfig.getInstance().getCameraStdDevFactors()[index];

    // for multi-tag strategies, scale the standard deviation by the reprojection error; for
    // single-tag, scale by the ambiguity
    if (observation.type() == PoseObservationType.MULTI_TAG) {
      xyStdDev *= (reprojectionErrorScaleFactor.get() * observation.reprojectionError());
    } else {
      xyStdDev *= (ambiguityScaleFactor.get() * (observation.averageAmbiguity() + 0.1));
    }

    // only trust the rotation component for multi-tag strategies; for single-tag, set the standard
    // deviation to infinity
    double thetaStdDev =
        observation.type() == PoseObservationType.MULTI_TAG
            ? thetaStdDevCoefficient.get()
                * Math.pow(observation.averageTagDistance(), 2.0)
                * (reprojectionErrorScaleFactor.get() * observation.reprojectionError())
                / observation.numTags()
                * RobotConfig.getInstance().getCameraStdDevFactors()[index]
            : Double.POSITIVE_INFINITY;

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  private void logCameraTransforms(int cameraIndex, PoseObservation observation) {
    // this is the pose of the robot when centered on the reef face that faces the driver station
    Pose3d cameraPose = observation.cameraPose();
    Transform3d robotToCameraTransform = cameraPose.minus(robotPosesForCalibration[cameraIndex]);

    Logger.recordOutput(
        SUBSYSTEM_NAME + "/" + cameraIndex + "/RobotToCameraTransform", robotToCameraTransform);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/" + cameraIndex + "/RobotToCameraPose",
        robotPosesForCalibration[cameraIndex]);
  }
}
