package frc.lib.team3061.vision;

import java.util.function.Supplier;

import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOSim implements VisionIO {
    private final String CAMERA_NAME = "simCamera";
    private final double DIAGONAL_FOV = 70; // FOV in degrees
    private final int IMG_WIDTH = 1280; // image width in px
    private final int IMG_HEIGHT = 720; // image heigh in px

    private Supplier<Pose2d> poseSupplier;
    private SimVisionSystem simVision;
    private AprilTagFieldLayout layout;

    public VisionIOSim(AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
        this.layout = layout;
        this.poseSupplier = poseSupplier;

        this.simVision = new SimVisionSystem(
            CAMERA_NAME, 
            DIAGONAL_FOV,
            robotToCamera.inverse(), 
            9000, 
            IMG_WIDTH, 
            IMG_HEIGHT, 
            0);
        
        for (AprilTag tag : layout.getTags()) {
            this.simVision.addSimVisionTarget(
                new SimVisionTarget(tag.pose, Units.inchesToMeters(6), Units.inchesToMeters(6), tag.ID));
        }
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        this.simVision.processFrame(poseSupplier.get());
        return null;
    }

    @Override
    public double getLatestTimestamp() {
        return Timer.getFPGATimestamp();
    }
}