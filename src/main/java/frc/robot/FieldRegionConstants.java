package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.team6328.util.FieldConstants;

/**
 * This class defines the points for all of the regions that defines the field along with the
 * transitions points between these regions. For more information refer to the Field2d and Region2d
 * classes.
 */
public final class FieldRegionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private FieldRegionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // Points making up the Community Zone (not including Charging Station)
  public static final Translation2d COMMUNITY_POINT_1 = FieldConstants.Community.regionCorners[0];
  public static final Translation2d COMMUNITY_POINT_2 = FieldConstants.Community.regionCorners[1];
  public static final Translation2d COMMUNITY_POINT_3 =
      new Translation2d(
          FieldConstants.Community.chargingStationInnerX, FieldConstants.Community.leftY);
  public static final Translation2d COMMUNITY_POINT_4 =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX, FieldConstants.Community.leftY);
  public static final Translation2d COMMUNITY_POINT_5 =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX,
          FieldConstants.Community.chargingStationLeftY);
  public static final Translation2d COMMUNITY_POINT_6 =
      new Translation2d(
          FieldConstants.Community.chargingStationInnerX,
          FieldConstants.Community.chargingStationLeftY);
  public static final Translation2d COMMUNITY_POINT_7 =
      new Translation2d(
          FieldConstants.Community.chargingStationInnerX,
          FieldConstants.Community.chargingStationRightY);
  public static final Translation2d COMMUNITY_POINT_8 =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX,
          FieldConstants.Community.chargingStationRightY);
  public static final Translation2d COMMUNITY_POINT_9 =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX, FieldConstants.Community.rightY);
  public static final Translation2d COMMUNITY_POINT_10 =
      new Translation2d(
          FieldConstants.Community.chargingStationInnerX, FieldConstants.Community.rightY);

  // Points making up the Loading Zone
  public static final Translation2d LOADING_ZONE_POINT_1 =
      FieldConstants.LoadingZone.regionCorners[0];
  public static final Translation2d LOADING_ZONE_POINT_2 =
      FieldConstants.LoadingZone.regionCorners[1];
  public static final Translation2d LOADING_ZONE_POINT_3 =
      FieldConstants.LoadingZone.regionCorners[2];
  public static final Translation2d LOADING_ZONE_POINT_4 =
      FieldConstants.LoadingZone.regionCorners[3];
  public static final Translation2d LOADING_ZONE_POINT_5 =
      new Translation2d(FieldConstants.LoadingZone.midX, FieldConstants.LoadingZone.leftY);
  public static final Translation2d LOADING_ZONE_POINT_6 =
      FieldConstants.LoadingZone.regionCorners[4];
  public static final Translation2d LOADING_ZONE_POINT_7 =
      FieldConstants.LoadingZone.regionCorners[5];

  // Points making up the rest of the Game Field (not including opposite Alliance zones)
  public static final Translation2d FIELD_POINT_1 =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX, FieldConstants.Community.rightY);
  public static final Translation2d FIELD_POINT_2 =
      new Translation2d(
          FieldConstants.fieldLength - FieldConstants.Community.chargingStationOuterX,
          FieldConstants.Community.rightY);
  public static final Translation2d FIELD_POINT_3 =
      new Translation2d(
          FieldConstants.fieldLength - FieldConstants.Community.chargingStationOuterX,
          (FieldConstants.Community.leftY + FieldConstants.Community.rightY) / 2.0);
  public static final Translation2d FIELD_POINT_4 =
      new Translation2d(
          FieldConstants.fieldLength - FieldConstants.Community.chargingStationOuterX,
          FieldConstants.Community.leftY);
  public static final Translation2d FIELD_POINT_5 =
      new Translation2d(FieldConstants.LoadingZone.midX, FieldConstants.Community.leftY);
  public static final Translation2d FIELD_POINT_6 =
      new Translation2d(FieldConstants.LoadingZone.midX, FieldConstants.LoadingZone.midY);
  public static final Translation2d FIELD_POINT_7 =
      new Translation2d(
          FieldConstants.fieldLength - FieldConstants.Community.chargingStationOuterX,
          FieldConstants.LoadingZone.midY);
  public static final Translation2d FIELD_POINT_8 =
      new Translation2d(FieldConstants.LoadingZone.outerX, FieldConstants.LoadingZone.midY);
  public static final Translation2d FIELD_POINT_9 =
      new Translation2d(FieldConstants.LoadingZone.outerX, FieldConstants.LoadingZone.leftY);
  public static final Translation2d FIELD_POINT_10 =
      new Translation2d(
          FieldConstants.fieldLength - FieldConstants.LoadingZone.outerX,
          FieldConstants.LoadingZone.leftY);
  public static final Translation2d FIELD_POINT_11 =
      new Translation2d(
          FieldConstants.fieldLength - FieldConstants.LoadingZone.outerX,
          FieldConstants.LoadingZone.midY);
  public static final Translation2d FIELD_POINT_12 =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX, FieldConstants.LoadingZone.midY);
  public static final Translation2d FIELD_POINT_13 =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX,
          (FieldConstants.Community.leftY + FieldConstants.Community.rightY) / 2.0);

  protected static final Translation2d[] COMMUNITY_REGION_POINTS_1 =
      new Translation2d[] {
        COMMUNITY_POINT_1, COMMUNITY_POINT_2, COMMUNITY_POINT_3, COMMUNITY_POINT_10
      };
  protected static final Translation2d[] COMMUNITY_REGION_POINTS_2 =
      new Translation2d[] {
        COMMUNITY_POINT_6, COMMUNITY_POINT_3, COMMUNITY_POINT_4, COMMUNITY_POINT_5
      };
  protected static final Translation2d[] COMMUNITY_REGION_POINTS_3 =
      new Translation2d[] {
        COMMUNITY_POINT_10, COMMUNITY_POINT_7, COMMUNITY_POINT_8, COMMUNITY_POINT_9
      };

  protected static final Translation2d[] LOADING_ZONE_REGION_POINTS_1 =
      new Translation2d[] {
        LOADING_ZONE_POINT_1, LOADING_ZONE_POINT_7, LOADING_ZONE_POINT_6, LOADING_ZONE_POINT_5
      };
  protected static final Translation2d[] LOADING_ZONE_REGION_POINTS_2 =
      new Translation2d[] {
        LOADING_ZONE_POINT_3, LOADING_ZONE_POINT_2, LOADING_ZONE_POINT_5, LOADING_ZONE_POINT_4
      };

  protected static final Translation2d[] FIELD_ZONE_REGION_POINTS_1 =
      new Translation2d[] {FIELD_POINT_13, FIELD_POINT_3, FIELD_POINT_7, FIELD_POINT_12};
  protected static final Translation2d[] FIELD_ZONE_REGION_POINTS_2 =
      new Translation2d[] {FIELD_POINT_1, FIELD_POINT_2, FIELD_POINT_3, FIELD_POINT_13};
  protected static final Translation2d[] FIELD_ZONE_REGION_POINTS_3 =
      new Translation2d[] {FIELD_POINT_4, FIELD_POINT_5, FIELD_POINT_6, FIELD_POINT_7};
  protected static final Translation2d[] FIELD_ZONE_REGION_POINTS_4 =
      new Translation2d[] {FIELD_POINT_8, FIELD_POINT_9, FIELD_POINT_10, FIELD_POINT_11};

  public static final Region2d COMMUNITY_REGION_1 = new Region2d(COMMUNITY_REGION_POINTS_1);
  public static final Region2d COMMUNITY_REGION_2 = new Region2d(COMMUNITY_REGION_POINTS_2);
  public static final Region2d COMMUNITY_REGION_3 = new Region2d(COMMUNITY_REGION_POINTS_3);
  public static final Region2d LOADING_ZONE_REGION_1 = new Region2d(LOADING_ZONE_REGION_POINTS_1);
  public static final Region2d LOADING_ZONE_REGION_2 = new Region2d(LOADING_ZONE_REGION_POINTS_2);
  public static final Region2d FIELD_ZONE_REGION_1 = new Region2d(FIELD_ZONE_REGION_POINTS_1);
  public static final Region2d FIELD_ZONE_REGION_2 = new Region2d(FIELD_ZONE_REGION_POINTS_2);
  public static final Region2d FIELD_ZONE_REGION_3 = new Region2d(FIELD_ZONE_REGION_POINTS_3);
  public static final Region2d FIELD_ZONE_REGION_4 = new Region2d(FIELD_ZONE_REGION_POINTS_4);

  // Points setting the transition points between each region
  public static final Translation2d COMMUNITY_REGION_1_2_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.Community.chargingStationInnerX,
          (FieldConstants.Community.leftY + FieldConstants.Community.chargingStationLeftY) / 2.0);
  public static final Translation2d COMMUNITY_REGION_2_1_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.Community.chargingStationInnerX - 0.5,
          (FieldConstants.Community.leftY + FieldConstants.Community.chargingStationLeftY) / 2.0
              - 0.5);
  public static final Translation2d COMMUNITY_REGION_1_3_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.Community.chargingStationInnerX,
          FieldConstants.Community.chargingStationRightY / 2.0);
  public static final Translation2d COMMUNITY_REGION_3_1_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.Community.chargingStationInnerX - 0.5,
          FieldConstants.Community.chargingStationRightY / 2.0 + 0.5);

  public static final Translation2d LOADING_ZONE_REGION_1_2_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.LoadingZone.midX - 0.5,
          (FieldConstants.LoadingZone.leftY + FieldConstants.LoadingZone.midY) / 2.0);
  public static final Translation2d LOADING_ZONE_REGION_2_1_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.LoadingZone.midX + 0.5,
          (FieldConstants.LoadingZone.leftY + FieldConstants.LoadingZone.midY) / 2.0);

  public static final Translation2d FIELD_ZONE_REGION_1_2_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.fieldLength / 2.0,
          (FieldConstants.Community.leftY + FieldConstants.Community.rightY) / 2.0 - 0.5);
  public static final Translation2d FIELD_ZONE_REGION_2_1_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.fieldLength / 2.0,
          (FieldConstants.Community.leftY + FieldConstants.Community.rightY) / 2.0 + 0.5);
  public static final Translation2d FIELD_ZONE_REGION_1_3_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.LoadingZone.midX - 0.5,
          (FieldConstants.LoadingZone.midY + FieldConstants.LoadingZone.rightY) / 2.0);
  public static final Translation2d FIELD_ZONE_REGION_3_1_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.fieldLength - FieldConstants.Community.chargingStationOuterX - 0.5,
          (FieldConstants.LoadingZone.midY + FieldConstants.LoadingZone.rightY) / 2.0);
  public static final Translation2d FIELD_ZONE_REGION_1_4_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.LoadingZone.outerX + 0.5,
          (FieldConstants.LoadingZone.leftY + FieldConstants.LoadingZone.midY) / 2.0);
  public static final Translation2d FIELD_ZONE_REGION_4_1_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.fieldLength / 2.0 - 0.5,
          (FieldConstants.LoadingZone.midY + FieldConstants.LoadingZone.rightY) / 2.0);

  public static final Translation2d COMMUNITY_2_TO_FIELD_1_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX + 0.5,
          (FieldConstants.LoadingZone.midY + FieldConstants.Community.chargingStationLeftY) / 2.0);
  public static final Translation2d COMMUNITY_3_TO_FIELD_2_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX + 0.5,
          FieldConstants.Community.chargingStationRightY / 2.0);

  public static final Translation2d FIELD_1_TO_COMMUNITY_2_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX - 0.5,
          (FieldConstants.Community.leftY + FieldConstants.Community.chargingStationLeftY) / 2.0);
  public static final Translation2d FIELD_2_TO_COMMUNITY_3_TRANSITION_POINT =
      new Translation2d(
          FieldConstants.Community.chargingStationOuterX - 0.5,
          FieldConstants.Community.chargingStationRightY / 2.0);
  public static final Translation2d FIELD_3_TO_LOADING_1_TRANSITION_POINT =
      new Translation2d(FieldConstants.LoadingZone.midX + 0.5, FieldConstants.LoadingZone.midY);
  public static final Translation2d FIELD_4_TO_LOADING_2_TRANSITION_POINT =
      new Translation2d(FieldConstants.LoadingZone.outerX + 0.5, FieldConstants.LoadingZone.midY);

  public static final Translation2d LOADING_2_TO_FIELD_4_TRANSITION_POINT =
      new Translation2d(FieldConstants.LoadingZone.outerX - 0.5, FieldConstants.LoadingZone.midY);

  // Points setting the node locations
  public static final Pose2d GRID_1_NODE_1 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_1_NODE_2 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_1_NODE_3 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * 2,
          Rotation2d.fromDegrees(180));

  public static final Pose2d GRID_2_NODE_1 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * 3,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_2_NODE_2 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * 4,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_2_NODE_3 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * 5,
          Rotation2d.fromDegrees(180));

  public static final Pose2d GRID_3_NODE_1 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * 6,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_3_NODE_2 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * 7,
          Rotation2d.fromDegrees(180));
  public static final Pose2d GRID_3_NODE_3 =
      new Pose2d(
          FieldConstants.Grids.outerX,
          FieldConstants.Grids.nodeFirstY + FieldConstants.Grids.nodeSeparationY * 8,
          Rotation2d.fromDegrees(180));

  // Points setting the substation locations
  public static final Pose2d SINGLE_SUBSTATION =
      new Pose2d(
          FieldConstants.LoadingZone.singleSubstationTranslation, Rotation2d.fromDegrees(90));
  public static final Pose2d DOUBLE_SUBSTATION_UPPER =
      new Pose2d(
          FieldConstants.LoadingZone.doubleSubstationX,
          FieldConstants.LoadingZone.leftY
              - ((FieldConstants.LoadingZone.doubleSubstationWidth / 4) - 0.1016),
          Rotation2d.fromDegrees(0));
  public static final Pose2d DOUBLE_SUBSTATION_LOWER =
      new Pose2d(
          FieldConstants.LoadingZone.doubleSubstationX,
          FieldConstants.LoadingZone.leftY
              - ((3 * FieldConstants.LoadingZone.doubleSubstationWidth / 4) + 0.178),
          Rotation2d.fromDegrees(0));
}
