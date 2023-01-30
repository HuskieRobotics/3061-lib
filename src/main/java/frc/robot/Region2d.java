package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.awt.geom.*;
import java.util.HashMap;
import java.util.Set;

public class Region2d {
  private Path2D shape;
  private HashMap<Region2d, Translation2d> transitionMap;
  /**
   * Create a Region2d, a polygon, from an array of Translation2d specifiying verticies of a
   * polygon. The polygon is created using the even-odd winding rule.
   *
   * @param points the array of Translation2d that define the verticies of the region.
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
   * Returns true if the region contains a given Pose2d
   *
   * @param other the given pose2d
   * @return if the pose is inside the region
   */
  public boolean contains(Pose2d other) {
    return this.shape.contains(new Point2D.Double(other.getX(), other.getY()));
  }

  /**
   * Add a neighboring Region2d and the ideal point through which to transition between the two.
   * Normally, this operation will be performed once per transition with the transition point on at
   * the center of the boundary between the two regions.
   *
   * @param other the other region
   * @param point a Translation2d representing the transition point
   */
  public void addNeighbor(Region2d other, Translation2d point) {
    transitionMap.put(other, point);
    // FIXME: should this mehtod add both regions as neighbors to each other? No, we want the
    // flexibility to set the transition point inside of one of the regions to avoid obstacles
    // (e.g., charging station).
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
      return transitionMap.get(other);
    } else {
      return null;
    }
  }
}
