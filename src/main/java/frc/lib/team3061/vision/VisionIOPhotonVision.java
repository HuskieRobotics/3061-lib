package frc.lib.team3061.vision;

import java.util.EnumSet;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOPhotonVision implements VisionIO {
    private static final String cameraName = "photonvision"; //Camera name of the photonvision camera
    private final PhotonCamera camera = new PhotonCamera(cameraName);

    private double lastTimestamp = 0;
    private PhotonPipelineResult lastResult = null;
    private boolean hasNewResult = false;

    public VisionIOPhotonVision() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        
        /* 
         * based on https://docs.wpilib.org/en/latest/docs/software/networktables/listening-for-change.html#listening-for-changes
         * and https://github.com/Mechanical-Advantage/RobotCode2022/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVision.java
        */
        DoubleSubscriber latencySub = inst.getTable("/photonvision/" + cameraName)
            .getDoubleTopic("latencyMillis")
            .subscribe(0.0);
        
        inst.addListener(latencySub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), event -> {
            PhotonPipelineResult result = camera.getLatestResult();
            double timestamp = Logger.getInstance().getRealTimestamp() - (result.getLatencyMillis() / 1000.0);
            synchronized (VisionIOPhotonVision.this) {
                lastTimestamp = timestamp;
                lastResult = result;
                hasNewResult = true;
            }
        });
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        inputs.lastTimestamp = lastTimestamp;
        inputs.lastResult = lastResult;
        inputs.hasNewResult = hasNewResult;
    }

    @Override
    public boolean hasNewResult() {
        return hasNewResult;
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        hasNewResult = false;
        return lastResult;
    }
}
