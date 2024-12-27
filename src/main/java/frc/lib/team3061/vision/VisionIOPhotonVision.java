package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.MAX_NUMBER_TAGS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * PhotonVision-based implementation of the VisionIO interface.
 *
 * <p>Adapted from
 * https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java
 */
public class VisionIOPhotonVision implements VisionIO {
  private static final int EXPIRATION_COUNT = 5;

  protected final PhotonCamera camera;
  protected PhotonPoseEstimator photonEstimator;
  private final boolean[] tagsSeen;
  private int cyclesWithNoResults = 0;

  /**
   * Creates a new VisionIOPhotonVision object.
   *
   * @param cameraName the name of the PhotonVision camera to use; the name must be unique
   */
  public VisionIOPhotonVision(String cameraName, AprilTagFieldLayout layout) {
    this.camera = new PhotonCamera(cameraName);
    this.photonEstimator =
        new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d());

    // the index of the array corresponds to the tag ID; so, add one since there is no tag ID 0
    this.tagsSeen = new boolean[MAX_NUMBER_TAGS + 1];
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    this.cyclesWithNoResults += 1;

    inputs.connected = camera.isConnected();
    List<VisionIO.PoseObservation> observations = new ArrayList<>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> visionEstimate = this.photonEstimator.update(result);

      visionEstimate.ifPresent(
          estimate -> {
            long tagsSeenBitMap = 0;
            double averageAmbiguity = 0.0;
            double averageTagDistance = 0.0;

            for (int i = 0; i < estimate.targetsUsed.size(); i++) {
              tagsSeenBitMap |= 1L << estimate.targetsUsed.get(i).getFiducialId();
              averageAmbiguity += estimate.targetsUsed.get(i).getPoseAmbiguity();
              averageTagDistance +=
                  estimate.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getNorm();
            }
            averageAmbiguity /= estimate.targetsUsed.size();
            averageTagDistance /= estimate.targetsUsed.size();

            observations.add(
                new PoseObservation(
                    result.getTimestampSeconds(),
                    estimate.estimatedPose,
                    NetworkTablesJNI.now() - result.getTimestampSeconds(),
                    averageAmbiguity,
                    result.multitagResult.isPresent()
                        ? result.multitagResult.get().estimatedPose.bestReprojErr
                        : 0.0,
                    tagsSeenBitMap,
                    estimate.targetsUsed.size(),
                    averageTagDistance,
                    estimate.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                        ? PoseObservationType.MULTI_TAG
                        : PoseObservationType.SINGLE_TAG));

            this.cyclesWithNoResults = 0;
          });
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }
}
