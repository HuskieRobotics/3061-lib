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
    Pose3d estimatedCameraPose = new Pose3d();
    double estimatedCameraPoseTimestamp = 0.0;
    double latencySecs = 0.0;
    boolean[] tagsSeen = new boolean[] {};
    double ambiguity = 0.0;
    double reprojectionError = 0.0;
    boolean poseFromMultiTag = false;
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(VisionIOInputs inputs) {}
}
