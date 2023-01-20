package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

public class FieldRegionTester {
    
    public static void main(String[] args)
    {
        FieldConstants.COMMUNITY_ZONE.makePath(new Pose2d(2.7,5, new Rotation2d()), FieldConstants.GRID_1_NODE_3, new PathConstraints(
            DrivetrainConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, DrivetrainConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

        
        //TODO: Find out why Drivetrain.getPose() is static
        /*
        FieldConstants.COMMUNITY_ZONE.makePath(Drivetrain.getPose(), FieldConstants.GRID_1_NODE_3, new PathConstraints(
            DrivetrainConstants.AUTO_MAX_SPEED_METERS_PER_SECOND, DrivetrainConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));
        */
    }
}
