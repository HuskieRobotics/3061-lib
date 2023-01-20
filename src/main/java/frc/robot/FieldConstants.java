package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The FieldConstants class contains various constants relating to the FRC Game Arena for
 * the Region2d and Field2d classes.
 */

public final class FieldConstants{
    // Current FieldConstants are for the FRC 2023 Game "Charged Up"
    //NOTE TO SELF: All constants to be accessed globally should be initialized using (public static)

    private static final Translation2d COMMUNITY_POINT_1 = new Translation2d(0,0);
    private static final Translation2d COMMUNITY_POINT_2 = new Translation2d(0,5.49);
    private static final Translation2d COMMUNITY_POINT_3 = new Translation2d(3,5.49);
    private static final Translation2d COMMUNITY_POINT_4 = new Translation2d(3.36,5.49);
    private static final Translation2d COMMUNITY_POINT_5 = new Translation2d(3.36,3.98);
    private static final Translation2d COMMUNITY_POINT_6 = new Translation2d(3,3.98);
    private static final Translation2d COMMUNITY_POINT_7 = new Translation2d(3,1.51);
    private static final Translation2d COMMUNITY_POINT_8 = new Translation2d(4.91, 1.51);
    private static final Translation2d COMMUNITY_POINT_9 = new Translation2d(4.91, 0);
    private static final Translation2d COMMUNITY_POINT_10 = new Translation2d(3,0);

    private static final Translation2d[] COMMUNITY_REGION_POINTS_1 = new Translation2d[] {
        COMMUNITY_POINT_1, COMMUNITY_POINT_2, COMMUNITY_POINT_3, COMMUNITY_POINT_10
    };
    private static final Translation2d[] COMMUNITY_REGION_POINTS_2 = new Translation2d[] {
        COMMUNITY_POINT_6, COMMUNITY_POINT_3, COMMUNITY_POINT_4, COMMUNITY_POINT_5
    };
    private static final Translation2d[] COMMUNITY_REGION_POINTS_3 = new Translation2d[] {
        COMMUNITY_POINT_10, COMMUNITY_POINT_7, COMMUNITY_POINT_8, COMMUNITY_POINT_9
    };

    public final static Region2d COMMUNITY_REGION_1 = new Region2d(COMMUNITY_REGION_POINTS_1);
    public final static Region2d COMMUNITY_REGION_2 = new Region2d(COMMUNITY_REGION_POINTS_2);
    public final static Region2d COMMUNITY_REGION_3 = new Region2d(COMMUNITY_REGION_POINTS_3);

    public final static Field2d COMMUNITY_ZONE = new Field2d(new Region2d[] {
        COMMUNITY_REGION_1, COMMUNITY_REGION_2, COMMUNITY_REGION_3}
    );

    //TODO: Replace placeholder X values for each grid node location
    public final static Pose2d GRID_1_NODE_1 = new Pose2d(2, 0.51, new Rotation2d(Math.PI));    //should be 180 in radians
    public final static Pose2d GRID_1_NODE_2 = new Pose2d(2, 1.05, new Rotation2d(Math.PI));
    public final static Pose2d GRID_1_NODE_3 = new Pose2d(2, 1.63, new Rotation2d(Math.PI));

    public final static Pose2d GRID_2_NODE_1 = new Pose2d(2, 2.19, new Rotation2d(Math.PI));
    public final static Pose2d GRID_2_NODE_2 = new Pose2d(2, 2.75, new Rotation2d(Math.PI));
    public final static Pose2d GRID_2_NODE_3 = new Pose2d(2, 3.31, new Rotation2d(Math.PI));

    public final static Pose2d GRID_3_NODE_1 = new Pose2d(2, 3.87, new Rotation2d(Math.PI));
    public final static Pose2d GRID_3_NODE_2 = new Pose2d(2, 4.42, new Rotation2d(Math.PI));
    public final static Pose2d GRID_3_NODE_3 = new Pose2d(2, 4.98, new Rotation2d(Math.PI));
}