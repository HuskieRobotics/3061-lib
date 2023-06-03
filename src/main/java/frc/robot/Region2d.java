package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.team6328.util.FieldConstants;
import java.awt.geom.*;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * This class models a region of the field. It is defined by its vertices and the transition points
 * to neighboring regions.
 */
public class Region2d {
  private Path2D shape;
  private HashMap<Region2d, Translation2d> transitionMap;
  private DriverStation.Alliance alliance = DriverStation.Alliance.Invalid;

  /**
   * Create a Region2d, a polygon, from an array of Translation2d specifying vertices of a polygon.
   * The polygon is created using the even-odd winding rule.
   *
   * @param points the array of Translation2d that define the vertices of the region.
   */
  public Region2d(Translation2d[] points) {
    this.transitionMap = new HashMap<>();
    this.shape = new Path2D.Double(Path2D.WIND_EVEN_ODD, points.length);
    this.shape.moveTo(points[0].getX(), points[0].getY());

    for (int i = 1; i < points.length; i++) {
      this.shape.lineTo(points[i].getX(), points[i].getY());
    }

    this.shape.closePath();
  }

  /**
   * Log the bounding rectangle of the region and the transition points to neighboring regions.
   * These can be visualized using AdvantageScope to confirm that the regions are properly defined.
   */
  public void logPoints() {
    // log the bounding rectangle of the region
    Rectangle2D rect = this.shape.getBounds2D();
    Logger.getInstance()
        .recordOutput("Region2d/point0", new Pose2d(rect.getX(), rect.getY(), new Rotation2d()));
    Logger.getInstance()
        .recordOutput(
            "Region2d/point1",
            new Pose2d(rect.getX() + rect.getWidth(), rect.getY(), new Rotation2d()));
    Logger.getInstance()
        .recordOutput(
            "Region2d/point2",
            new Pose2d(rect.getX(), rect.getY() + rect.getHeight(), new Rotation2d()));
    Logger.getInstance()
        .recordOutput(
            "Region2d/point3",
            new Pose2d(
                rect.getX() + rect.getWidth(), rect.getY() + rect.getHeight(), new Rotation2d()));

    // assume that there are at most 4 neighbors
    for (int i = 0; i < 4; i++) {
      Logger.getInstance().recordOutput("Region2d/transition" + i, new Pose2d());
    }
    int i = 0;
    for (Entry<Region2d, Translation2d> entry : transitionMap.entrySet()) {
      Translation2d point = entry.getValue();
      Logger.getInstance()
          .recordOutput(
              "Region2d/transition" + i, new Pose2d(point.getX(), point.getY(), new Rotation2d()));
      i++;
    }
  }

  /**
   * Returns true if the region contains a given Pose2d accounting for the alliance color.
   *
   * @param other the given pose2d
   * @return if the pose is inside the region
   */
  public boolean contains(Pose2d other) {

    if (this.alliance == DriverStation.Alliance.Red) {
      return this.shape.contains(
          new Point2D.Double(other.getX(), FieldConstants.fieldWidth - other.getY()));
    } else {
      return this.shape.contains(new Point2D.Double(other.getX(), other.getY()));
    }
  }

  /**
   * Add a neighboring Region2d and the ideal point through which to transition from this region to
   * the other region. Normally, this method will be invoked once per transition. The transition
   * point doesn't need to be on the boundary between the two regions. It may be advantageous for
   * the transition point to be located within the other region to influence the generated path.
   * Therefore, the transition point from one region to another may not be the same when traversing
   * the regions in the opposite direction.
   *
   * @param other the other region
   * @param point a Translation2d representing the transition point
   */
  public void addNeighbor(Region2d other, Translation2d point) {
    transitionMap.put(other, point);
  }

  /**
   * Returns a Set of this region's neighbors
   *
   * @return
   */
  public Set<Region2d> getNeighbors() {
    return transitionMap.keySet();
  }

  /**
   * Get the transition pont between this region and another region. Returns null if they aren't a
   * neighbor.
   *
   * @param other the other region
   * @return the transition point, represented as a Translation2d
   */
  public Translation2d getTransitionPoint(Region2d other) {
    if (getNeighbors().contains(other)) {

      if (this.alliance == DriverStation.Alliance.Red) {
        return new Translation2d(
            transitionMap.get(other).getX(),
            FieldConstants.fieldWidth - transitionMap.get(other).getY());
      } else {
        return transitionMap.get(other);
      }

    } else {
      return null;
    }
  }

  /**
   * Sets the current alliance color which is used to transform the coordinates of the region.
   *
   * @param newAlliance the new alliance color
   */
  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
  }
}
