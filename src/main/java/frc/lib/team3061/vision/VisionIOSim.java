package frc.lib.team3061.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/**
 * PhotonVision-compatible simulated implementation of the VisionIO interface. Only a single
 * VisionIOSim object may be instantiated. It uses the PhotonVision SimVisionSystem to simulates the
 * AprilTag targets that would be seen by a camera based on the robot's pose, which is determined
 * based on its odometry.
 */
public class VisionIOSim extends VisionIOPhotonVision {
  private static final String CAMERA_NAME = "simCamera";
  private static final double DIAGONAL_FOV = 160; // FOV in degrees
  private static final int IMG_WIDTH = 1280; // image width in px
  private static final int IMG_HEIGHT = 960; // image heigh in px

  private Supplier<Pose2d> poseSupplier;
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOSim object.
   *
   * @param layout the AprilTag field layout
   * @param poseSupplier a Pose2d supplier that returns the robot's pose based on its odometry
   * @param robotToCamera the transform from the robot's center to the simulated camera
   */
  public VisionIOSim(
      AprilTagFieldLayout layout, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
    super(CAMERA_NAME, layout);

    this.poseSupplier = poseSupplier;

    this.visionSim = new VisionSystemSim(CAMERA_NAME);
    this.visionSim.addAprilTags(layout);
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(IMG_WIDTH, IMG_HEIGHT, Rotation2d.fromDegrees(DIAGONAL_FOV));
    // cameraProp.setCalibError(0.35, 0.10);
    // cameraProp.setFPS(15);
    // cameraProp.setAvgLatencyMs(50);
    // cameraProp.setLatencyStdDevMs(15);

    this.cameraSim = new PhotonCameraSim(camera, cameraProp);

    visionSim.addCamera(cameraSim, robotToCamera);
    cameraSim.enableDrawWireframe(false);
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public void updateInputs(VisionIOInputs inputs) {
    this.visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }
}
