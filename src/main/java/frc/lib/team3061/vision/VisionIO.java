package frc.lib.team3061.vision;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    PhotonPipelineResult lastResult = new PhotonPipelineResult(0, new ArrayList<>());
    double lastTimestamp = 0.0;
    boolean hasNewResult = false;

    public void toLog(LogTable table) {
      byte[] photonPacketBytes = new byte[lastResult.getPacketSize()];
      lastResult.populatePacket(new Packet(photonPacketBytes));
      table.put("photonPacketBytes", photonPacketBytes);

      table.put("lastTimestamp", lastTimestamp);
      table.put("hasNewResult", hasNewResult);

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
      hasNewResult = table.getBoolean("hasNewResult", false);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
