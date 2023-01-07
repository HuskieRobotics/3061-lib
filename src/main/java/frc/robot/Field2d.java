package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.counter.EdgeConfiguration;

public class Field2d {
    private Region2d[] regions;

    /**
     * Construct a field2d from an array of regions. These regions should not be overlapping (aside from edges)
     * and any regions with overlapping edges should be neighbors (see Region2d::addNeighbor). 
     * @param regions
     */
    public Field2d(Region2d[] regions) {
        this.regions = regions;
    }
    public PathPlannerTrajectory makePath(Pose2d start, Pose2d end, PathConstraints pathConstants) {
        Region2d startRegion = null;
        Region2d endRegion = null;

        //find the starting and ending regions
        for (Region2d region : regions) {
            if (region.contains(start)) {
                startRegion = region;
            }
            if (region.contains(end)) {
                endRegion = region;
            }
        }

        //make sure both start and end are on the field
        if (startRegion == null || endRegion == null) return null;

        //BFS to find the shortest path to the end
        ArrayList<Region2d> path = breadthFirstSearch(startRegion, endRegion);
        if (path == null) return null;

        //create point locations
        ArrayList<Translation2d> pointLocations = new ArrayList<>();
        pointLocations.add(start.getTranslation());

        //add all the transition points
        for (int i = 0; i < path.size() - 1; i++) {
            Region2d from = path.get(i);
            Region2d to = path.get(i+1);
            pointLocations.add(from.getTransitionPoint(to));
        }

        pointLocations.add(end.getTranslation());

        //find the correct heading for each 
        


        //create points
        List<PathPoint> points = new ArrayList<>();
        Rotation2d lastHeading = null;
        for (int i = 0; i < pointLocations.size() - 2; i++) {
            double deltaX = pointLocations.get(i+1).getX() - pointLocations.get(i).getX();
            double deltaY = pointLocations.get(i+1).getY() - pointLocations.get(i).getY();
            lastHeading = new Rotation2d(deltaX, deltaY);
            if (i == 0) {
                points.add(new PathPoint(
                    pointLocations.get(i),
                    lastHeading,
                    start.getRotation()
                ));
            } else {
                points.add(new PathPoint(
                    pointLocations.get(i),
                    lastHeading
                ));
            }
        }

        points.add(new PathPoint(
            pointLocations.get(pointLocations.size() - 1),
            lastHeading,
            end.getRotation()
        ));
            

        // TODO: swap to new method once it's released
        return PathPlanner.generatePath(pathConstants, points.get(0), points.get(1), points.subList(2,points.size()).toArray(new PathPoint[0]));
        //return PathPlanner.generatePath(pathConstants, points); //list constructor in main, not yet in a release 



    }

    public ArrayList<Region2d> breadthFirstSearch(Region2d start, Region2d end) {
        Queue<ArrayList<Region2d>> todo = new LinkedList<>();
        Set<Region2d> explored = new HashSet<>();
        
        todo.add(new ArrayList<Region2d>(Arrays.asList(start))); //add a path starting with startRegion to the todo list

        while (todo.size() > 0) { // while the todo list isn't empty, keep looking over the todo list.
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
        return null;
    }
}
