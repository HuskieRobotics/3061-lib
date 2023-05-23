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

public class Vision extends SubsystemBase {
  private VisionIO[] visionIOs;
  private Transform3d[] camerasToRobots;
  private final VisionIOInputs[] ios;
  private double[] lastTimestamps;

  private AprilTagFieldLayout layout;

  private SwerveDrivePoseEstimator poseEstimator;
  private boolean isEnabled = true;
  private boolean isVisionUpdating = false;
  private static double POSE_DIFFERENCE_THRESHOLD = 0.5;
  private TunableNumber tunablePoseDifferenceThreshold;

  private Alert noAprilTagLayoutAlert =
      new Alert(
          "No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java",
          AlertType.WARNING);

  private static final TunableNumber SDslope = new TunableNumber("Vision/stddev_Slope", 0.10);
  private static final TunableNumber SDpower = new TunableNumber("AutoBalance/stddev_Power", 2.0);

  private static class RobotPoseFromAprilTag {
    public final Pose3d robotPose;
    public final double distanceToAprilTag;

    public RobotPoseFromAprilTag(Pose3d robotPose, double distance) {
      this.robotPose = robotPose;
      this.distanceToAprilTag = distance;
    }
  }

  public Vision(VisionIO... visionIO) {
    this.visionIOs = visionIO;
    this.camerasToRobots = RobotConfig.getInstance().getRobotToCameraTransforms();
    this.lastTimestamps = new double[visionIO.length];
    this.ios = new VisionIOInputs[visionIO.length];
    for (int i = 0; i < visionIO.length; i++) {
      this.ios[i] = new VisionIOInputs();
    }

    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    this.tunablePoseDifferenceThreshold =
        new TunableNumber("Vision/VisionPoseThreshold", POSE_DIFFERENCE_THRESHOLD);

    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    tabMain
        .addBoolean("isVisionUpdating", () -> isVisionUpdating)
        .withPosition(7, 2)
        .withSize(1, 2);

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
      if (layout.getTagPose(tag.ID).isPresent()) {
        Logger.getInstance()
            .recordOutput("Vision/AprilTags/" + tag.ID, layout.getTagPose(tag.ID).get());
      }
    }
  }

  @Override
  public void periodic() {
    isVisionUpdating = false;
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(ios[i]);
      Logger.getInstance().processInputs("Vision" + i, ios[i]);

      if (lastTimestamps[i] < ios[i].lastTimestamp) {
        lastTimestamps[i] = ios[i].lastTimestamp;
        RobotPoseFromAprilTag poseAndDistance = getRobotPose(i);
        Pose3d robotPose = poseAndDistance.robotPose;

        if (robotPose == null) return;

        if (poseEstimator
                .getEstimatedPosition()
                .minus(robotPose.toPose2d())
                .getTranslation()
                .getNorm()
            < MAX_POSE_DIFFERENCE_METERS) {
          if (isEnabled) {
            // old
            // poseEstimator.addVisionMeasurement(robotPose.toPose2d(), ios[i].lastTimestamp);

            // new
            poseEstimator.addVisionMeasurement(
                robotPose.toPose2d(),
                ios[i].lastTimestamp,
                getStdDevs(poseAndDistance.distanceToAprilTag));
            isVisionUpdating = true;
          }

          Logger.getInstance().recordOutput("Vision/RobotPose" + i, robotPose.toPose2d());
          Logger.getInstance().recordOutput("Vision/isEnabled", isEnabled);
        }
      }
    }
  }

  private Matrix<N3, N1> getStdDevs(double targetDistance) {
    double stdDevTrust = SDslope.get() * (Math.pow(targetDistance, SDpower.get()));
    return VecBuilder.fill(stdDevTrust, stdDevTrust, stdDevTrust);
  }

  // make a tunable number for the equation for standard deviation
  // 1 meter is .2, 3 meters is .9, so then

  public boolean isEnabled() {
    return isEnabled;
  }

  private RobotPoseFromAprilTag getRobotPose(int index) {
    int targetCount = 0;
    Pose3d robotPoseFromClosestTarget = null;
    double closestTargetDistance = Double.MAX_VALUE;

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
          if (targetDistance < VisionConstants.MAX_DISTANCE_TO_TARGET
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

  public void enable(boolean enable) {
    isEnabled = enable;
  }

  public boolean isValidTarget(PhotonTrackedTarget target) {
    return target.getFiducialId() != -1
        && target.getPoseAmbiguity() != -1
        && target.getPoseAmbiguity() < VisionConstants.MAXIMUM_AMBIGUITY
        && layout.getTagPose(target.getFiducialId()).isPresent();
  }

  public boolean posesInLine() {
    for (int i = 0; i < visionIOs.length; i++) {
      Pose3d robotPose = getRobotPose(i).robotPose;
      if (robotPose != null
          && poseEstimator
                  .getEstimatedPosition()
                  .minus(robotPose.toPose2d())
                  .getTranslation()
                  .getNorm()
              < tunablePoseDifferenceThreshold.get()) {
        Logger.getInstance().recordOutput("Vision/posesInLine", true);
        return true;
      }
    }
    Logger.getInstance().recordOutput("Vision/posesInLine", false);
    return false;
  }
}
