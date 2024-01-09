package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * PhotonVision-based implementation of the VisionIO interface.
 *
 * <p>Adapted from
 * https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java
 */
public class VisionIOPhotonVision implements VisionIO {
  private Alert noCameraConnectedAlert =
      new Alert("specified camera not connected", AlertType.WARNING);
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private double lastTimestamp = 0;

  /**
   * Creates a new VisionIOPhotonVision object.
   *
   * @param cameraName the name of the PhotonVision camera to use; the name must be unique
   */
  public VisionIOPhotonVision(
      String cameraName, AprilTagFieldLayout layout, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.photonEstimator =
        new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Optional<EstimatedRobotPose> visionEstimate = this.photonEstimator.update();
    double latestTimestamp = camera.getLatestResult().getTimestampSeconds();

    boolean newResult = Math.abs(latestTimestamp - lastTimestamp) > 1e-5;
    if (newResult) {
      visionEstimate.ifPresent(
          estimate -> {
            inputs.estimatedRobotPose = estimate.estimatedPose;
            inputs.estimatedRobotPoseTimestamp = estimate.timestampSeconds;
            int[] tags = new int[estimate.targetsUsed.size()];
            for (int i = 0; i < estimate.targetsUsed.size(); i++) {
              tags[i] = estimate.targetsUsed.get(i).getFiducialId();
            }
            inputs.estimatedRobotPoseTags = tags;
            inputs.lastCameraTimestamp = latestTimestamp;
            lastTimestamp = latestTimestamp;
          });
    }

    noCameraConnectedAlert.set(!camera.isConnected());
  }
}
