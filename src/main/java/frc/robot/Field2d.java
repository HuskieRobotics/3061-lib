package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;

/**
 * This singleton class models the field as a collection of regions. This class is used to create a
 * path from a starting pose in one region to an ending pose in another region that passes through
 * the transition points defined for those regions.
 *
 * <p>The coordinate system of the field is oriented such that the origin is in the lower left
 * corner when the blue alliance is to the left (i.e., to the blue alliance driver's right). It can
 * map poses to reflect the current alliance.
 */
public class Field2d {
  private static Field2d instance = null;

  private Region2d[] regions;
  private DriverStation.Alliance alliance = DriverStation.Alliance.Invalid;

  /**
   * Get the singleton instance of the Field2d class.
   *
   * @return the singleton instance of the Field2d class
   */
  public static Field2d getInstance() {
    if (instance == null) {
      instance = new Field2d();
    }
    return instance;
  }
  /**
   * Construct a Field2d from an array of regions. These regions should not be overlapping (aside
   * from edges) and any regions with overlapping edges should be neighbors (see
   * Region2d::addNeighbor).
   *
   * @param regions the regions that define the field
   */
  public void setRegions(Region2d[] regions) {
    this.regions = regions;
  }

  /**
   * Set the current alliance. This will adjust the coordinate system of the field and its regions.
   *
   * @param newAlliance the new alliance
   */
  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
    for (Region2d region : regions) {
      region.updateAlliance(newAlliance);
    }
  }

  /**
   * Translates the specified pose based on the current alliance.
   *
   * @param pose the pose to translate based on the current alliance
   * @return the translated pose based on the current alliance
   */
  public Pose2d mapPoseToCurrentAlliance(Pose2d pose) {
    if (this.alliance == DriverStation.Alliance.Red) {
      return new Pose2d(
          new Translation2d(
              pose.getTranslation().getX(),
              FieldConstants.fieldWidth - pose.getTranslation().getY()),
          new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
    } else {
      return pose;
    }
  }

  /**
   * Create a path from a starting pose in one region to an ending pose in another region that
   * passes through the transition points defined for those regions.
   *
   * @param start the starting pose
   * @param end the ending pose
   * @param pathConstants the path constraints (i.e., max velocity, max acceleration)
   * @param subsystem the drivetrain subsystem
   * @return the path from the starting pose to the ending pose; null if no path exists
   */
  public PathPlannerTrajectory makePath(
      Pose2d start, Pose2d end, PathConstraints pathConstants, Drivetrain subsystem) {
    Region2d startRegion = null;
    Region2d endRegion = null;

    // find the starting and ending regions
    for (Region2d region : regions) {
      if (region.contains(start)) {
        startRegion = region;
      }
      if (region.contains(end)) {
        endRegion = region;
      }
    }

    // make sure both start and end are on the field
    if (startRegion == null || endRegion == null) return null;

    // BFS to find the shortest path to the end
    List<Region2d> path = breadthFirstSearch(startRegion, endRegion);
    if (path.isEmpty()) return null;

    // create point locations
    ArrayList<Translation2d> pointLocations = new ArrayList<>();

    // add the starting point
    pointLocations.add(start.getTranslation());

    // add all the transition points
    for (int i = 0; i < path.size() - 1; i++) {
      Region2d from = path.get(i);
      Region2d to = path.get(i + 1);
      pointLocations.add(from.getTransitionPoint(to));
    }

    // add a transition point if starting region & ending region same
    if (startRegion == endRegion) {
      pointLocations.add(
          new Translation2d((start.getX() + end.getX()) / 2, (start.getY() + end.getY()) / 2));
    }

    // add the ending point
    pointLocations.add(end.getTranslation());

    List<PathPoint> pathPoints = createPathPoints(start, end, subsystem, pointLocations);
    return PathPlanner.generatePath(pathConstants, pathPoints);
  }

  /**
   * Create the path points based on the starting and ending poses and the point locations. The path
   * will be created such that the first path point matches the robot's current heading and velocity
   * to ensure a smooth transition to the path. The the starting and ending poses have different
   * rotations, the change in rotation will occur between the first and second points. The final
   * speed of the robot will be as specified by the robot's configuration class'
   * getMoveToPathFinalVelocity method.
   *
   * @param start the starting pose
   * @param end the ending pose
   * @param subsystem the drivetrain subsystem
   * @param pointLocations the locations of the points in the path
   * @return the path points
   */
  private List<PathPoint> createPathPoints(
      Pose2d start, Pose2d end, Drivetrain subsystem, ArrayList<Translation2d> pointLocations) {
    List<PathPoint> pathPoints = new ArrayList<>();
    Rotation2d lastHeading = null;
    for (int i = 0; i < pointLocations.size() - 1; i++) {
      double deltaX = pointLocations.get(i + 1).getX() - pointLocations.get(i).getX();
      double deltaY = pointLocations.get(i + 1).getY() - pointLocations.get(i).getY();
      lastHeading = new Rotation2d(deltaX, deltaY);
      if (i == 0) {
        // if the robot is not currently moving, orient the heading towards the next point
        if (subsystem.getVelocityX() == 0 && subsystem.getVelocityY() == 0) {
          pathPoints.add(
              new PathPoint(
                  pointLocations.get(i),
                  lastHeading,
                  start.getRotation(),
                  Math.sqrt(
                      Math.pow(subsystem.getVelocityX(), 2)
                          + Math.pow(subsystem.getVelocityY(), 2))));
        }
        // if the robot is currently moving, maintain the current heading and velocity in order to
        // have a smooth transition to the start of the path
        else {
          pathPoints.add(
              new PathPoint(
                  pointLocations.get(i),
                  new Rotation2d(subsystem.getVelocityX(), subsystem.getVelocityY()),
                  start.getRotation(),
                  Math.sqrt(
                      Math.pow(subsystem.getVelocityX(), 2)
                          + Math.pow(subsystem.getVelocityY(), 2))));
        }
      } else {
        pathPoints.add(new PathPoint(pointLocations.get(i), lastHeading, end.getRotation()));
      }
    }

    // the final path point will match the ending pose's rotation and the velocity as specified by
    // the robot's configuration class' getMoveToPathFinalVelocity method.
    pathPoints.add(
        new PathPoint(
            pointLocations.get(pointLocations.size() - 1),
            end.getRotation(),
            end.getRotation(),
            RobotConfig.getInstance().getMoveToPathFinalVelocity()));

    return pathPoints;
  }

  private List<Region2d> breadthFirstSearch(Region2d start, Region2d end) {
    Queue<ArrayList<Region2d>> todo = new LinkedList<>();
    Set<Region2d> explored = new HashSet<>();

    // add the starting region to the set of explored regions
    explored.add(start);

    // if the path starts and ends in the same region, return that region
    if (start == end) {
      return new ArrayList<>(Arrays.asList(start));
    }

    todo.add(
        new ArrayList<>(
            Arrays.asList(start))); // add a path starting with startRegion to the todo list

    while (!todo.isEmpty()) { // while the todo list isn't empty, keep looking over the todo list.
      ArrayList<Region2d> path = todo.poll();
      Region2d region = path.get(path.size() - 1); // last region in the path

      for (Region2d other : region.getNeighbors()) {
        if (!explored.contains(other)) {
          ArrayList<Region2d> newPath = new ArrayList<>(path);
          newPath.add(other);

          if (other == end) {
            return newPath;
          }

          explored.add(other);
          todo.add(newPath);
        }
      }
    }
    return new ArrayList<>();
  }
}
