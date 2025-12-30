// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.FieldConstants;
import frc.lib.team6328.util.SystemTimeValidReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionIONorthstar implements VisionIO {
  private final String deviceId;
  private final DoubleArraySubscriber observationSubscriber;
  private final DoubleArraySubscriber objDetectObservationSubscriber;
  private final IntegerSubscriber fpsAprilTagsSubscriber;
  private final IntegerSubscriber fpsObjDetectSubscriber;
  private final StringPublisher eventNamePublisher;
  private final IntegerPublisher matchTypePublisher;
  private final IntegerPublisher matchNumberPublisher;
  private final IntegerPublisher timestampPublisher;
  private final BooleanPublisher isRecordingPublisher;

  private final Timer slowPeriodicTimer = new Timer();

  private final List<VisionIO.PoseObservation> observations = new ArrayList<>();
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final Transform3d robotToCameraTransform;

  public VisionIONorthstar(AprilTagFieldLayout layout, RobotConfig.CameraConfig camera) {
    this.deviceId = "northstar_" + camera.location();
    this.robotToCameraTransform = camera.robotToCameraTransform();
    this.aprilTagFieldLayout = layout;
    String layoutString = "";
    var northstarTable = NetworkTableInstance.getDefault().getTable(this.deviceId);
    var configTable = northstarTable.getSubTable("config");

    try {
      layoutString = new ObjectMapper().writeValueAsString(layout);
    } catch (JsonProcessingException e) {
      throw new RuntimeException(
          "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
    }

    configTable.getStringTopic("camera_id").publish().set(camera.id());
    configTable.getIntegerTopic("camera_resolution_width").publish().set(camera.width());
    configTable.getIntegerTopic("camera_resolution_height").publish().set(camera.height());
    configTable.getIntegerTopic("camera_auto_exposure").publish().set(camera.autoExposure());
    configTable.getIntegerTopic("camera_exposure").publish().set(camera.exposure());
    configTable.getDoubleTopic("camera_gain").publish().set(camera.gain());
    configTable.getDoubleTopic("camera_denoise").publish().set(camera.denoise());
    configTable.getDoubleTopic("fiducial_size_m").publish().set(FieldConstants.aprilTagWidth);
    configTable.getStringTopic("tag_layout").publish().set(layoutString);
    isRecordingPublisher = configTable.getBooleanTopic("is_recording").publish();
    isRecordingPublisher.set(false);
    timestampPublisher = configTable.getIntegerTopic("timestamp").publish();
    eventNamePublisher = configTable.getStringTopic("event_name").publish();
    matchTypePublisher = configTable.getIntegerTopic("match_type").publish();
    matchNumberPublisher = configTable.getIntegerTopic("match_number").publish();

    var outputTable = northstarTable.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    objDetectObservationSubscriber =
        outputTable
            .getDoubleArrayTopic("objdetect_observations")
            .subscribe(
                new double[] {},
                PubSubOption.keepDuplicates(true),
                PubSubOption.sendAll(true),
                PubSubOption.pollStorage(5),
                PubSubOption.periodic(0.01));
    fpsAprilTagsSubscriber = outputTable.getIntegerTopic("fps_apriltags").subscribe(0);
    fpsObjDetectSubscriber = outputTable.getIntegerTopic("fps_objdetect").subscribe(0);

    slowPeriodicTimer.start();
  }

  public void updateInputs(
      VisionIOInputs inputs,
      AprilTagVisionIOInputs aprilTagInputs,
      ObjDetectVisionIOInputs objDetectInputs) {
    boolean slowPeriodic = slowPeriodicTimer.advanceIfElapsed(1.0);

    observations.clear();

    // Update NT connection status
    inputs.connected = false;
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      if (client.remote_id.startsWith(this.deviceId)) {
        inputs.connected = true;
        break;
      }
    }

    // Publish timestamp
    if (slowPeriodic && SystemTimeValidReader.isValid()) {
      timestampPublisher.set(WPIUtilJNI.getSystemTime() / 1000000);
    }

    if (slowPeriodic) {
      eventNamePublisher.set(DriverStation.getEventName());
      matchTypePublisher.set(DriverStation.getMatchType().ordinal());
      matchNumberPublisher.set(DriverStation.getMatchNumber());
    }

    // Get AprilTag data
    var aprilTagQueue = observationSubscriber.readQueue();
    aprilTagInputs.timestamps = new double[aprilTagQueue.length];
    aprilTagInputs.frames = new double[aprilTagQueue.length][];
    for (int i = 0; i < aprilTagQueue.length; i++) {
      aprilTagInputs.timestamps[i] = aprilTagQueue[i].timestamp / 1000000.0;
      aprilTagInputs.frames[i] = aprilTagQueue[i].value;

      processAprilTagFrame(aprilTagInputs.timestamps[i], aprilTagInputs.frames[i], observations);
    }
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    if (slowPeriodic) {
      aprilTagInputs.fps = fpsAprilTagsSubscriber.get();
    }

    // Get object detection data
    var objDetectQueue = objDetectObservationSubscriber.readQueue();
    objDetectInputs.timestamps = new double[objDetectQueue.length];
    objDetectInputs.frames = new double[objDetectQueue.length][];
    for (int i = 0; i < objDetectQueue.length; i++) {
      objDetectInputs.timestamps[i] = objDetectQueue[i].timestamp / 1000000.0;
      objDetectInputs.frames[i] = objDetectQueue[i].value;
    }
    if (slowPeriodic) {
      objDetectInputs.fps = fpsObjDetectSubscriber.get();
    }
  }

  public void setRecording(boolean active) {
    isRecordingPublisher.set(active);
  }

  private void processAprilTagFrame(
      double timestamp, double[] values, List<PoseObservation> observations) {

    // Exit if blank frame
    if (values.length == 0 || values[0] == 0) {
      return;
    }

    // Switch based on number of poses
    Pose3d cameraPose = null;
    PoseObservationType observationType = PoseObservationType.SINGLE_TAG;
    double ambiguity = 0.0;
    double reprojectionError = 0.0;
    long tagsSeenBitMap = 0;

    switch ((int) values[0]) {
      case 1:
        reprojectionError = values[1];
        // One pose (multi-tag), use directly
        cameraPose =
            new Pose3d(
                values[2],
                values[3],
                values[4],
                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
        observationType = PoseObservationType.MULTI_TAG;
        break;
      case 2:
        // Two poses (one tag), disambiguate
        double error0 = values[1];
        double error1 = values[9];
        Pose3d cameraPose0 =
            new Pose3d(
                values[2],
                values[3],
                values[4],
                new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
        Pose3d cameraPose1 =
            new Pose3d(
                values[10],
                values[11],
                values[12],
                new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));

        Transform3d cameraToRobot = this.robotToCameraTransform.inverse();
        Pose3d robotPose0 = cameraPose0.plus(cameraToRobot);
        Pose3d robotPose1 = cameraPose1.plus(cameraToRobot);

        // Check for ambiguity and select based on estimated rotation
        if (error0 < error1 * AMBIGUITY_THRESHOLD || error1 < error0 * AMBIGUITY_THRESHOLD) {
          Rotation2d currentRotation = RobotOdometry.getInstance().getEstimatedPose().getRotation();
          Rotation2d visionRotation0 = robotPose0.toPose2d().getRotation();
          Rotation2d visionRotation1 = robotPose1.toPose2d().getRotation();
          if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
              < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
            cameraPose = cameraPose0;
            ambiguity = error0;
          } else {
            cameraPose = cameraPose1;
            ambiguity = error1;
          }
        }
        break;
    }

    // Exit if no data
    if (cameraPose == null) {
      return;
    }

    List<Pose3d> tagPoses = new ArrayList<>();
    for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i += 10) {
      int tagId = (int) values[i];
      tagsSeenBitMap |= 1L << tagId;
      Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagId);
      tagPose.ifPresent(tagPoses::add);
    }
    if (tagPoses.isEmpty()) return;

    // Calculate average distance to tag
    double totalDistance = 0.0;
    for (Pose3d tagPose : tagPoses) {
      totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
    }
    double avgDistance = totalDistance / tagPoses.size();

    observations.add(
        new PoseObservation(
            timestamp,
            cameraPose,
            Timer.getFPGATimestamp() - timestamp,
            ambiguity,
            reprojectionError,
            tagsSeenBitMap,
            tagPoses.size(),
            avgDistance,
            observationType));
  }
}
