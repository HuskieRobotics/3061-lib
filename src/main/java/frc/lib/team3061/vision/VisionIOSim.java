package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOSim implements VisionIO {
  private static final String CAMERA_NAME = "simCamera";
  private static final double DIAGONAL_FOV = 70; // FOV in degrees
  private static final int IMG_WIDTH = 1280; // image width in px
  private static final int IMG_HEIGHT = 720; // image heigh in px

  private Supplier<Pose2d> poseSupplier;
  private SimVisionSystem simVision;

  public VisionIOSim(
      AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
    this.poseSupplier = poseSupplier;

    this.simVision =
        new SimVisionSystem(
            CAMERA_NAME, DIAGONAL_FOV, robotToCamera.inverse(), 9000, IMG_WIDTH, IMG_HEIGHT, 0);

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
