package frc.lib.team3061.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import java.util.EnumSet;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** PhotonVision-based implementation of the VisionIO interface. */
public class VisionIOPhotonVision implements VisionIO {
  private Alert noCameraConnectedAlert =
      new Alert("specified camera not connected", AlertType.WARNING);
  private final PhotonCamera camera;

  private double lastTimestamp = 0;
  private PhotonPipelineResult lastResult = new PhotonPipelineResult();

  /**
   * Creates a new VisionIOPhotonVision object.
   *
   * @param cameraName the name of the PhotonVision camera to use; the name must be unique
   */
  public VisionIOPhotonVision(String cameraName) {
    camera = new PhotonCamera(cameraName);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /*
     * based on https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html#listening-for-changes
     * and https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java
     */
    DoubleArraySubscriber targetPoseSub =
        inst.getTable("/photonvision/" + cameraName)
            .getDoubleArrayTopic("targetPose")
            .subscribe(new double[0]);

    inst.addListener(
        targetPoseSub,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        event -> {
          PhotonPipelineResult result = camera.getLatestResult();

          double timestamp = result.getTimestampSeconds();
          synchronized (VisionIOPhotonVision.this) {
            lastTimestamp = timestamp;
            lastResult = result;
          }
        });
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.lastTimestamp = this.lastTimestamp;
    inputs.lastResult = this.lastResult;

    noCameraConnectedAlert.set(!camera.isConnected());
  }
}
