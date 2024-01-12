// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.team6328.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Map;
import java.util.Optional;

@java.lang.SuppressWarnings({"java:S1118", "java:S115", "java:S2386"})

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link #flipForRedSide(Translation2d)} and {@link #flipForRedSide(Pose2d)}
 * methods to flip these values to get the red alliance version.
 */
public final class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth = Units.inchesToMeters(323.25);
  public static final double tapeWidth = Units.inchesToMeters(2.0);
  

  /**
   * Flips a translation to the correct side of the field based on the current alliance color. By
   * default, all translations and poses in {@link FieldConstants} are stored with the origin at the
   * rightmost point on the BLUE ALLIANCE wall.
   */
  public static Translation2d flipForRedSide(Translation2d translation) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return new Translation2d(fieldLength - translation.getX(), translation.getY());
    } else {
      return translation;
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance color. By default,
   * all translations and poses in {@link FieldConstants} are stored with the origin at the
   * rightmost point on the BLUE ALLIANCE wall.
   */
  public static Pose2d flipForRedSide(Pose2d pose, boolean rotate180) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      int flipSign = rotate180 ? -1 : 0;
      return new Pose2d(
          fieldLength - pose.getX(),
          pose.getY(),
          new Rotation2d(flipSign * pose.getRotation().getCos(), pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }
}
