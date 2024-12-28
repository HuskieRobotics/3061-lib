package frc.lib.team3061.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * The hardware abstraction interface for a PhotonVision-based co-processor that provides
 * PhotonPipelineResult objects. There is a one-to-one relationship between each VisionIO object and
 * each co-processor (e.g., Raspberry Pi) running PhotonVision.
 *
 * <p>In the future, this interface may be further abstracted to not be coupled to PhotonVision.
 * Currently, the abstraction is used to simulate vision.
 */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    boolean connected;
    PoseObservation[] poseObservations = new PoseObservation[0];
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d cameraPose,
      double latencySecs,
      double averageAmbiguity,
      double reprojectionError,
      long tagsSeenBitMap,
      int numTags,
      double averageTagDistance,
      PoseObservationType type) {}

  public enum PoseObservationType {
    SINGLE_TAG,
    MULTI_TAG
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(VisionIOInputs inputs) {}
}
