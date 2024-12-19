package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * PhotonVision-compatible simulated implementation of the VisionIO interface. Only a single
 * VisionIOSim object may be instantiated. It uses the PhotonVision SimVisionSystem to simulates the
 * AprilTag targets that would be seen by a camera based on the robot's pose, which is determined
 * based on its odometry.
 */
public class VisionIOSim implements VisionIO {
  private static final String CAMERA_NAME = "simCamera";
  private static final double DIAGONAL_FOV = 160; // FOV in degrees
  private static final int IMG_WIDTH = 1280; // image width in px
  private static final int IMG_HEIGHT = 960; // image heigh in px
  private static final int EXPIRATION_COUNT = 5;

  private final PhotonCamera camera = new PhotonCamera(CAMERA_NAME);
  private final PhotonPoseEstimator photonEstimator;
  private final boolean[] tagsSeen;
  private int cyclesWithNoResults = 0;

  private Supplier<Pose2d> poseSupplier;
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOSim object.
   *
   * @param layout the AprilTag field layout
   * @param poseSupplier a Pose2d supplier that returns the robot's pose based on its odometry
   * @param robotToCamera the transform from the robot's center to the simulated camera
   */
  public VisionIOSim(
      AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
    this.photonEstimator =
        new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera.inverse());
    this.poseSupplier = poseSupplier;

    this.visionSim = new VisionSystemSim(CAMERA_NAME);
    this.visionSim.addAprilTags(layout);
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(IMG_WIDTH, IMG_HEIGHT, Rotation2d.fromDegrees(DIAGONAL_FOV));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);

    this.cameraSim = new PhotonCameraSim(camera, cameraProp);

    visionSim.addCamera(cameraSim, new Transform3d());
    cameraSim.enableDrawWireframe(true);

    // the index of the array corresponds to the tag ID; so, add one since there is no tag ID 0
    this.tagsSeen = new boolean[layout.getTags().size() + 1];
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    this.visionSim.update(poseSupplier.get());

    this.cyclesWithNoResults += 1;

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> visionEstimate = this.photonEstimator.update(result);

      visionEstimate.ifPresent(
          estimate -> {
            inputs.estimatedCameraPose = estimate.estimatedPose;
            inputs.estimatedCameraPoseTimestamp = estimate.timestampSeconds;
            for (int i = 0; i < this.tagsSeen.length; i++) {
              this.tagsSeen[i] = false;
            }
            for (int i = 0; i < estimate.targetsUsed.size(); i++) {
              this.tagsSeen[estimate.targetsUsed.get(i).getFiducialId()] = true;
            }
            inputs.tagsSeen = this.tagsSeen;
            inputs.poseFromMultiTag =
                estimate.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

            inputs.ambiguity = 0;
            for (int i = 0; i < estimate.targetsUsed.size(); i++) {
              inputs.ambiguity += estimate.targetsUsed.get(i).getPoseAmbiguity();
            }
            inputs.ambiguity /= estimate.targetsUsed.size();

            if (inputs.poseFromMultiTag) {
              inputs.ambiguity = 0.2;
            }

            this.cyclesWithNoResults = 0;
          });
    }

    // if no tags have been seen for the specified number of cycles, clear the array
    if (this.cyclesWithNoResults == EXPIRATION_COUNT) {
      for (int i = 0; i < this.tagsSeen.length; i++) {
        this.tagsSeen[i] = false;
      }
      inputs.tagsSeen = this.tagsSeen;
    }
  }
}
