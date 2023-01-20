package frc.lib.team3061.vision;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.nio.file.Path;

public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // FIXME: add the current year's AprilTag field layout file to the deploy directory and update
  // this constant with the current year's AprilTag field layout file
  public static final Path APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2023-chargedup.json").toPath();

  public static final double MAXIMUM_AMBIGUITY = 0.2;
  public static final double MAX_POSE_DIFFERENCE_METERS = 1.0;
}
