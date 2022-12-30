package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;


/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by subsystems (drivetrain and vision)
 */
public class RobotOdometry {
    private static RobotOdometry robotOdometry;
    private static SwerveDrivePoseEstimator estimator;
    private SwerveModulePosition[] defaultPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private RobotOdometry() {
        estimator = new SwerveDrivePoseEstimator(DrivetrainConstants.KINEMATICS, new Rotation2d(), defaultPositions, new Pose2d());
    }
    public static RobotOdometry getInstance() {
        if (robotOdometry == null) {
            robotOdometry = new RobotOdometry();
        }
        return robotOdometry;
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return estimator;
    }
}