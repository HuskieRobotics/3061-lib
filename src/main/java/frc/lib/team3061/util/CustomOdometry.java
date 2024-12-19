package frc.lib.team3061.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;

public interface CustomOdometry {
  Pose2d getCustomEstimatedPose();

  Optional<Pose2d> samplePoseAt(double timestamp);

  void resetCustomPose(Pose2d pose);

  void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs);
}
