// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.team6328.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

@java.lang.SuppressWarnings({"java:S1118", "java:S115", "java:S2386"})

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(651.223);
  public static final double fieldWidth = Units.inchesToMeters(323.277);

  public static final double blueWingX = Units.inchesToMeters(229.201);
  public static final double redWingX = fieldLength - blueWingX;
  public static final double bluePodiumX = Units.inchesToMeters(126.75);
  public static final double redPodiumX = fieldLength - bluePodiumX;
  public static final double blueStartingLineX = Units.inchesToMeters(74.111);
  public static final double redStartingLineX = fieldLength - blueStartingLineX;

  public static final Translation2d blueAmpCenter =
      new Translation2d(Units.inchesToMeters(72.455), Units.inchesToMeters(322.996));
  public static final Translation2d redAmpCenter = FieldConstants.flipForRedSide(blueAmpCenter);

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static final double centerlineX = fieldLength / 2.0;

    public static final double centerlineFirstY = Units.inchesToMeters(29.638);
    private static final double centerlineSeparationY = Units.inchesToMeters(66);
    private static final double blueSpikeX = Units.inchesToMeters(114.0);
    // should be right
    private static final double blueSpikeFirstY = Units.inchesToMeters(161.638);
    private static final double blueSpikeSeparationY = Units.inchesToMeters(57);

    // Notes in center
    public static final Translation2d[] centerlineTranslations = new Translation2d[5];
    public static final Translation2d[] blueSpikeTranslations = new Translation2d[3];
    public static final Translation2d[] redSpikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < blueSpikeTranslations.length; i++) {
        blueSpikeTranslations[i] =
            new Translation2d(blueSpikeX, blueSpikeFirstY + (i * blueSpikeSeparationY));
      }
    }

    static {
      for (int i = 0; i < redSpikeTranslations.length; i++) {
        redSpikeTranslations[i] = FieldConstants.flipForRedSide(blueSpikeTranslations[i]);
      }
    }
  }

  /** Each corner of the speaker * */
  public static final class BlueSpeaker {

    /** Center of the speaker opening (blue alliance) */
    public static final Pose2d blueCenterSpeakerOpening =
        new Pose2d(0.0, fieldWidth - Units.inchesToMeters(104.0), new Rotation2d());

    // corners (blue alliance origin)
    public static final Translation3d blueTopRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(13.091));

    public static final Translation3d blueTopLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static final Translation3d blueBottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d blueBottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));
  }

  public static final class RedSpeaker {
    /** Center of the speaker opening (blue alliance) */
    public static final Pose2d redCenterSpeakerOpening =
        flipForRedSide(BlueSpeaker.blueCenterSpeakerOpening);

    // corners (red alliance origin)
    public static final Translation3d redTopRightSpeaker =
        FieldConstants.flipForRedSide(BlueSpeaker.blueTopLeftSpeaker);
    public static final Translation3d redTopLeftSpeaker =
        FieldConstants.flipForRedSide(BlueSpeaker.blueTopRightSpeaker);
    public static final Translation3d redBottomRightSpeaker =
        FieldConstants.flipForRedSide(BlueSpeaker.blueBottomLeftSpeaker);
    public static final Translation3d redBottomLeftSpeaker =
        FieldConstants.flipForRedSide(BlueSpeaker.blueBottomRightSpeaker);
  }

  public static class RedSource {
    public static final Translation2d redSourceCenter =
        new Translation2d(Units.inchesToMeters(35.6247), Units.inchesToMeters(21.9704));
    public static final Pose2d redSourcePose =
        new Pose2d(redSourceCenter, new Rotation2d(2.8163 - (Math.PI) / 2));
  }

  public static class BlueSource {
    public static final Translation2d blueSourceCenter = flipForRedSide(RedSource.redSourceCenter);
    public static final Pose2d blueSourcePose = flipForRedSide(RedSource.redSourcePose);
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);

  public static Translation3d flipForRedSide(Translation3d translation) {
    return new Translation3d(
        fieldLength - translation.getX(), translation.getY(), translation.getZ());
  }

  public static Translation2d flipForRedSide(Translation2d translation) {
    return new Translation2d(fieldLength - translation.getX(), translation.getY());
  }

  public static Pose2d flipForRedSide(Pose2d pose) {
    return new Pose2d(
        fieldLength - pose.getX(), pose.getY(), new Rotation2d(Math.PI).minus(pose.getRotation()));
  }

  public static Rotation2d flipForRedSide(Rotation2d rotation) {
    return new Rotation2d(Math.PI).minus(rotation);
  }
}
