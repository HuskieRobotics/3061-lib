package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected PhotonPoseEstimator photonEstimator;
  private final List<VisionIO.PoseObservation> observations = new ArrayList<>();

  /**
   * Creates a new VisionIOPhotonVision object.
   *
   * @param cameraName the name of the PhotonVision camera to use; the name must be unique
   */
  public VisionIOPhotonVision(String cameraName, AprilTagFieldLayout layout) {
    this.camera = new PhotonCamera(cameraName);

    // Don't pass the robot to camera transform as we will work with the estimated camera poses and
    // later transform them to the robot's frame
    this.photonEstimator = new PhotonPoseEstimator(layout, new Transform3d());

    // flush any old results from previous results
    this.camera.getAllUnreadResults();
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public void updateInputs(
      VisionIOInputs inputs,
      AprilTagVisionIOInputs aprilTagInputs,
      ObjDetectVisionIOInputs objDetectInputs) {
    inputs.connected = camera.isConnected();
    observations.clear();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> visionEstimate =
          this.photonEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()) {
        visionEstimate = this.photonEstimator.estimateLowestAmbiguityPose(result);
      }

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
                    Timer.getFPGATimestamp() - result.getTimestampSeconds(),
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
          });
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }
}
