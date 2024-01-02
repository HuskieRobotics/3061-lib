package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
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
    Pose3d estimatedRobotPose = new Pose3d();
    double estimatedRobotPoseTimestamp = 0.0;
    int[] estimatedRobotPoseTags = new int[] {};
    double lastCameraTimestamp = 0.0;
  }

  /**
   * Updates the set of loggable inputs.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(VisionIOInputs inputs) {}

  /**
   * Sets the origin of the AprilTag field layout. This is invoked once the alliance color is known.
   *
   * @param origin the origin of the AprilTag field layout
   */
  public default void setLayoutOrigin(OriginPosition origin) {}
}
