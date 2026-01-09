package frc.lib.team3061.vision;

import static frc.lib.team3061.vision.VisionConstants.*;

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
 *
 * <p>Due to a current bug in PhotonVision, the simulated camera assumes that the 2024 field layout
 * is being used.
 */
public class VisionIOSim extends VisionIOPhotonVision {
  private static final double DIAGONAL_FOV = 96.0; // FOV in degrees
  private static final int IMG_WIDTH = 1600; // image width in px
  private static final int IMG_HEIGHT = 1200; // image heigh in px

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
      String cameraName,
      AprilTagFieldLayout layout,
      Supplier<Pose2d> poseSupplier,
      Transform3d robotToCamera) {
    super(cameraName, layout);

    this.poseSupplier = poseSupplier;

    this.visionSim = new VisionSystemSim(cameraName);
    this.visionSim.addAprilTags(layout);
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(IMG_WIDTH, IMG_HEIGHT, Rotation2d.fromDegrees(DIAGONAL_FOV));
    cameraProp.setCalibError(SIM_AVERAGE_ERROR_PIXELS, SIM_ERROR_STD_DEV_PIXELS);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(100);
    cameraProp.setLatencyStdDevMs(30);

    this.cameraSim = new PhotonCameraSim(camera, cameraProp, layout);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  /**
   * Updates the specified VisionIOInputs object with the latest data from the camera.
   *
   * @param inputs the VisionIOInputs object to update with the latest data from the camera
   */
  @Override
  public void updateInputs(
      VisionIOInputs inputs,
      AprilTagVisionIOInputs aprilTagInputs,
      ObjDetectVisionIOInputs objDetectInputs) {
    this.visionSim.update(poseSupplier.get());
    super.updateInputs(inputs, aprilTagInputs, objDetectInputs);
  }
}
