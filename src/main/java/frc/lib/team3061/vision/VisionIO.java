package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The hardware abstraction interface for a PhotonVision-based co-processor that provides
 * PhotonPipelineResult objects. There is a one-to-one relationship between each VisionIO object and
 * each co-processor (e.g., Raspberry Pi) running PhotonVision.
 *
 * <p>In the future, this interface may be further abstracted to not be coupled to PhotonVision.
 * Currently, the abstraction is used to simulate vision.
 */
public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    PhotonPipelineResult lastResult = new PhotonPipelineResult(0, new ArrayList<>());
    double lastTimestamp = 0.0;

    public void toLog(LogTable table) {
      // since AdvantageKit doesn't support the PhotonPipelineResult type, log it as a byte array
      byte[] photonPacketBytes = new byte[lastResult.getPacketSize()];
      lastResult.populatePacket(new Packet(photonPacketBytes));
      table.put("photonPacketBytes", photonPacketBytes);

      table.put("lastTimestamp", lastTimestamp);

      // log targets in a human-readable way
      List<PhotonTrackedTarget> targets = lastResult.getTargets();
      String[] stringifiedTargets = new String[targets.size()];

      for (int i = 0; i < targets.size(); i++) {
        stringifiedTargets[i] = targets.get(i).toString();
      }
      table.put("stringifiedTargets", stringifiedTargets);
    }

    public void fromLog(LogTable table) {
      byte[] photonPacketBytes = table.getRaw("photonPacketBytes", new byte[0]);
      lastResult = new PhotonPipelineResult();
      lastResult.createFromPacket(new Packet(photonPacketBytes));

      lastTimestamp = table.getDouble("lastTimestamp", 0.0);
    }
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
