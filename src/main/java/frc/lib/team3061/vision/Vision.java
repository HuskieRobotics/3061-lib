package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.VisionIO.VisionIOInputs;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The Vision subsystem is responsible for updating the robot's estimated pose based on a collection
 * of cameras capturing AprilTags. The Vision subsystem is comprised of multiple VisionIO objects,
 * each of which is responsible for producing a single PhotonPipelineResult. There is a one-to-one
 * relationship between each VisionIO object and each co-processor (e.g., Raspberry Pi) running
 * PhotonVision.
 */
public class Vision extends SubsystemBase {
  private VisionIO[] visionIOs;
  private Transform3d[] camerasToRobots;
  private final VisionIOInputs[] ios;
  private double[] lastTimestamps;

  private AprilTagFieldLayout layout;
  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  private boolean isEnabled = true;
  private boolean isVisionUpdating = false;

  private SwerveDrivePoseEstimator poseEstimator;
  private final TunableNumber poseDifferenceThreshold =
      new TunableNumber("Vision/VisionPoseThreshold", POSE_DIFFERENCE_THRESHOLD_METERS);
  private final TunableNumber stdDevSlope = new TunableNumber("Vision/stdDevSlope", 0.10);
  private final TunableNumber stdDevPower = new TunableNumber("Vision/stdDevPower", 2.0);

  private static class RobotPoseFromAprilTag {
    public final Pose3d robotPose;
    public final double distanceToAprilTag;

    public RobotPoseFromAprilTag(Pose3d robotPose, double distance) {
      this.robotPose = robotPose;
      this.distanceToAprilTag = distance;
    }
  }

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
    this.camerasToRobots = RobotConfig.getInstance().getRobotToCameraTransforms();
    this.lastTimestamps = new double[visionIOs.length];
    this.ios = new VisionIOInputs[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      this.ios[i] = new VisionIOInputs();
    }

    // retrieve a reference to the pose estimator singleton
    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

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
      Logger.getInstance().recordOutput("Vision/AprilTags/" + tag.ID, tag.pose);
    }
  }

  /**
   * This method should be invoked once the alliance color is known. Refer to the RobotContainer's
   * checkAllianceColor method for best practices on when to check the alliance's color. It updates
   * the AprilTag field layout, logs the new location of the tags, and updates all of the VisionIO
   * objects with the new alliance color.
   *
   * @param newAlliance the new alliance color
   */
  public void updateAlliance(DriverStation.Alliance newAlliance) {

    if (newAlliance == DriverStation.Alliance.Red) {
      layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      for (VisionIO visionIO : visionIOs) {
        visionIO.setLayoutOrigin(OriginPosition.kRedAllianceWallRightSide);
      }
    } else {
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      for (VisionIO visionIO : visionIOs) {
        visionIO.setLayoutOrigin(OriginPosition.kBlueAllianceWallRightSide);
      }
    }

    for (AprilTag tag : layout.getTags()) {
      layout
          .getTagPose(tag.ID)
          .ifPresent(pose -> Logger.getInstance().recordOutput("Vision/AprilTags/" + tag.ID, pose));
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
      Logger.getInstance().processInputs("Vision" + i, ios[i]);

      // only process the vision data if the timestamp is newer than the last one
      if (lastTimestamps[i] < ios[i].lastTimestamp) {
        lastTimestamps[i] = ios[i].lastTimestamp;
        RobotPoseFromAprilTag poseAndDistance = getRobotPose(i);
        Pose3d robotPose = poseAndDistance.robotPose;

        if (robotPose == null) return;

        // only update the pose estimator if the pose from the vision data is close to the estimated
        // robot pose
        if (poseEstimator
                .getEstimatedPosition()
                .minus(robotPose.toPose2d())
                .getTranslation()
                .getNorm()
            < MAX_POSE_DIFFERENCE_METERS) {

          // only update the pose estimator if the vision subsystem is enabled
          if (isEnabled) {
            // when updating the pose estimator, specify standard deviations based on the distance
            // from the robot to the AprilTag (the greater the distance, the less confident we are
            // in the measurement)
            poseEstimator.addVisionMeasurement(
                robotPose.toPose2d(),
                ios[i].lastTimestamp,
                getStandardDeviations(poseAndDistance.distanceToAprilTag));
            isVisionUpdating = true;
          }

          Logger.getInstance().recordOutput("Vision/RobotPose" + i, robotPose.toPose2d());
          Logger.getInstance().recordOutput("Vision/IsEnabled", isEnabled);
        }
      }
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

  public Pose3d getBestRobotPose() {
    Pose3d robotPoseFromClosestTarget = null;
    double closestTargetDistance = Double.MAX_VALUE;
    for (int i = 0; i < visionIOs.length; i++) {
      RobotPoseFromAprilTag poseAndDistance = getRobotPose(i);
      Pose3d robotPose = poseAndDistance.robotPose;
      double distanceToAprilTag = poseAndDistance.distanceToAprilTag;
      if (robotPose != null && distanceToAprilTag < closestTargetDistance) {
        robotPoseFromClosestTarget = robotPose;
        closestTargetDistance = distanceToAprilTag;
      }
    }
    return robotPoseFromClosestTarget;
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
      Pose3d robotPose = getRobotPose(i).robotPose;
      if (robotPose != null
          && poseEstimator
                  .getEstimatedPosition()
                  .minus(robotPose.toPose2d())
                  .getTranslation()
                  .getNorm()
              < poseDifferenceThreshold.get()) {
        Logger.getInstance().recordOutput("Vision/posesInLine", true);
        return true;
      }
    }
    Logger.getInstance().recordOutput("Vision/posesInLine", false);
    return false;
  }

  private Matrix<N3, N1> getStandardDeviations(double targetDistance) {
    // the standard deviation of the vision measurement is a function of the distance from the robot
    // to the AprilTag and can be tuned
    double stdDevTrust = stdDevSlope.get() * (Math.pow(targetDistance, stdDevPower.get()));
    return VecBuilder.fill(stdDevTrust, stdDevTrust, stdDevTrust);
  }

  /**
   * Returns the robot pose based on vision data and distance to the AprilTag that is closest to the
   * robot. The current algorithm simply uses the AprilTag that is closest to the robot from which
   * to determine the robot's pose. In the future, this method could be updated to use multiple
   * tags.
   *
   * @param index the index of the VisionIO object to use
   * @return the robot pose based on vision data and distance to the AprilTag that is closest to the
   *     robot
   */
  private RobotPoseFromAprilTag getRobotPose(int index) {
    int targetCount = 0;
    Pose3d robotPoseFromClosestTarget = null;
    double closestTargetDistance = Double.MAX_VALUE;

    // "zero" the tag and robot poses such that old data is not used if no new data is available; in
    // terms of logging, we are assuming that a given VisionIO object won't see more than 2 tags at
    // once
    for (int i = 0; i < 2; i++) {
      Logger.getInstance().recordOutput("Vision/TagPose" + index + "_" + i, new Pose2d());
      Logger.getInstance().recordOutput("Vision/NVRobotPose" + index + "_" + i, new Pose2d());
    }

    for (PhotonTrackedTarget target : ios[index].lastResult.getTargets()) {
      if (isValidTarget(target)) {
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Optional<Pose3d> tagPoseOptional = layout.getTagPose(target.getFiducialId());
        if (tagPoseOptional.isPresent()) {
          Pose3d tagPose = tagPoseOptional.get();
          Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());
          Pose3d robotPose = cameraPose.transformBy(camerasToRobots[index].inverse());

          Logger.getInstance()
              .recordOutput("Vision/TagPose" + index + "_" + targetCount, tagPose.toPose2d());
          Logger.getInstance()
              .recordOutput("Vision/NVRobotPose" + index + "_" + targetCount, robotPose.toPose2d());

          double targetDistance =
              target.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm();

          // only consider tags that are within a certain distance of the robot
          if (targetDistance < VisionConstants.MAX_DISTANCE_TO_TARGET_METERS
              && targetDistance < closestTargetDistance) {
            closestTargetDistance = targetDistance;
            robotPoseFromClosestTarget = robotPose;
          }
        }
      }
      targetCount++;
    }

    return new RobotPoseFromAprilTag(robotPoseFromClosestTarget, closestTargetDistance);
  }

  private boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && layout.getTagPose(target.getFiducialId()).isPresent();
  }
}
