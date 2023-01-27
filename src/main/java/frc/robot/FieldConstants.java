package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The FieldConstants class contains various constants relating to the FRC Game Arena for the
 * Region2d and Field2d classes.
 */
public final class FieldConstants {
  // Current FieldConstants are for the FRC 2023 Game "Charged Up"
  // NOTE TO SELF: All constants to be accessed globally should be initialized using (public static)

  public static final Translation2d COMMUNITY_POINT_1 = new Translation2d(0, 0);
  public static final Translation2d COMMUNITY_POINT_2 = new Translation2d(0, 5.49);
  public static final Translation2d COMMUNITY_POINT_3 = new Translation2d(3, 5.49);
  public static final Translation2d COMMUNITY_POINT_4 = new Translation2d(3.36, 5.49);
  public static final Translation2d COMMUNITY_POINT_5 = new Translation2d(3.36, 3.98);
  public static final Translation2d COMMUNITY_POINT_6 = new Translation2d(3, 3.98);
  public static final Translation2d COMMUNITY_POINT_7 = new Translation2d(3, 1.51);
  public static final Translation2d COMMUNITY_POINT_8 = new Translation2d(4.91, 1.51);
  public static final Translation2d COMMUNITY_POINT_9 = new Translation2d(4.91, 0);
  public static final Translation2d COMMUNITY_POINT_10 = new Translation2d(3, 0);

  public static final Translation2d[] COMMUNITY_REGION_POINTS_1 =
      new Translation2d[] {
        COMMUNITY_POINT_1, COMMUNITY_POINT_2, COMMUNITY_POINT_3, COMMUNITY_POINT_10
      };
  public static final Translation2d[] COMMUNITY_REGION_POINTS_2 =
      new Translation2d[] {
        COMMUNITY_POINT_6, COMMUNITY_POINT_3, COMMUNITY_POINT_4, COMMUNITY_POINT_5
      };
  public static final Translation2d[] COMMUNITY_REGION_POINTS_3 =
      new Translation2d[] {
        COMMUNITY_POINT_10, COMMUNITY_POINT_7, COMMUNITY_POINT_8, COMMUNITY_POINT_9
      };

  public static final Region2d COMMUNITY_REGION_1 = new Region2d(COMMUNITY_REGION_POINTS_1);
  public static final Region2d COMMUNITY_REGION_2 = new Region2d(COMMUNITY_REGION_POINTS_2);
  public static final Region2d COMMUNITY_REGION_3 = new Region2d(COMMUNITY_REGION_POINTS_3);

  public static final Field2d COMMUNITY_ZONE =
      new Field2d(new Region2d[] {COMMUNITY_REGION_1, COMMUNITY_REGION_2, COMMUNITY_REGION_3});

  // TODO: Replace placeholder X values for each grid node location
  public static final Pose2d GRID_1_NODE_1 = new Pose2d(2, 0.51, Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_1_NODE_2 = new Pose2d(2, 1.05, Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_1_NODE_3 = new Pose2d(2, 1.63, Rotation2d.fromDegrees(180));

  public static final Pose2d GRID_2_NODE_1 = new Pose2d(2, 2.19, Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_2_NODE_2 = new Pose2d(2, 2.75, Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_2_NODE_3 = new Pose2d(2, 3.31, Rotation2d.fromDegrees(180));

  public static final Pose2d GRID_3_NODE_1 = new Pose2d(2, 3.87, Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_3_NODE_2 = new Pose2d(2, 4.42, Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_3_NODE_3 = new Pose2d(2, 4.98, Rotation2d.fromDegrees(180));
}
