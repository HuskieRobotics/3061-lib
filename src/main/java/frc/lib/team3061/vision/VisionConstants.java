package frc.lib.team3061.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.nio.file.Path;

public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final Path APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2022-rapidreact.json").toPath();
  public static final double MAXIMUM_AMBIGUITY = 0.2;

  // FIXME: update this with the real transform from the robot to the camera
  public static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
}