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

public class Field2d {
  private static Field2d instance = null;

  private Region2d[] regions;
  private DriverStation.Alliance alliance = DriverStation.Alliance.Invalid;

  public static Field2d getInstance() {
    if (instance == null) {
      instance = new Field2d();
    }
    return instance;
  }
  /**
   * Construct a field2d from an array of regions. These regions should not be overlapping (aside
   * from edges) and any regions with overlapping edges should be neighbors (see
   * Region2d::addNeighbor).
   *
   * @param regions
   */
  public void setRegions(Region2d[] regions) {
    this.regions = regions;
  }

  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
    for (Region2d region : regions) {
      region.updateAlliance(newAlliance);
    }
  }

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
    pointLocations.add(start.getTranslation());

    // add all the transition points
    for (int i = 0; i < path.size() - 1; i++) {
      Region2d from = path.get(i);
      Region2d to = path.get(i + 1);
      pointLocations.add(from.getTransitionPoint(to));
    }

    // add transition point if starting region & ending region same
    if (startRegion == endRegion) {
      pointLocations.add(
          new Translation2d((start.getX() + end.getX()) / 2, (start.getY() + end.getY()) / 2));
    }

    pointLocations.add(end.getTranslation());

    // create points
    List<PathPoint> points = new ArrayList<>();
    Rotation2d lastHeading = null;
    for (int i = 0; i < pointLocations.size() - 1; i++) {
      double deltaX = pointLocations.get(i + 1).getX() - pointLocations.get(i).getX();
      double deltaY = pointLocations.get(i + 1).getY() - pointLocations.get(i).getY();
      lastHeading = new Rotation2d(deltaX, deltaY);
      if (i == 0) {
        if (subsystem.getVelocityX() == 0 && subsystem.getVelocityY() == 0) {
          points.add(
              new PathPoint(
                  pointLocations.get(i),
                  lastHeading,
                  start.getRotation(),
                  Math.sqrt(
                      Math.pow(subsystem.getVelocityX(), 2)
                          + Math.pow(subsystem.getVelocityY(), 2))));
        } else {
          points.add(
              new PathPoint(
                  pointLocations.get(i),
                  new Rotation2d(subsystem.getVelocityX(), subsystem.getVelocityY()),
                  start.getRotation(),
                  Math.sqrt(
                      Math.pow(subsystem.getVelocityX(), 2)
                          + Math.pow(subsystem.getVelocityY(), 2))));
        }
      } else {
        points.add(new PathPoint(pointLocations.get(i), lastHeading, end.getRotation()));
      }
    }

    points.add(
        new PathPoint(
            pointLocations.get(pointLocations.size() - 1),
            end.getRotation(),
            end.getRotation(),
            RobotConfig.getInstance().getStallAgainstElementVelocity()));

    return PathPlanner.generatePath(pathConstants, points);
  }

  public List<Region2d> breadthFirstSearch(Region2d start, Region2d end) {
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
