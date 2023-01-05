package frc.lib.team3061.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String APRILTAG_FIELD_LAYOUT_PATH = "";
  public static final double MAXIMUM_AMBIGUITY = .2;
  public static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(0, 0, 0),
          new Rotation3d(0, 0, 0)); // FIXME: update this with the real transform
}
