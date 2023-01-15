package frc.robot.commands;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This command, when executed, instructs the drivetrain subsystem to follow the specified
 * trajectory, persumably during the autonomous period. The superclass' execute method invokes the
 * drivetrain subsystem's setSwerveModuleStates method to follow the trajectory.
 *
 * <p>Requires: the Drivetrain subsystem (handled by superclass)
 *
 * <p>Finished When: the time of the specified path has elapsed (handled by superclass)
 *
 * <p>At End: stops the drivetrain
 */
public class FollowRobotRelativePath extends PPSwerveControllerCommand {
  private PIDController thetaController;
  private Drivetrain drivetrain;
  private PathPlannerTrajectory trajectory;
  private Transform2d poseOffset;

  /**
   * Constructs a new FollowPath object.
   *
   * @param trajectory the specified trajectory created by PathPlanner
   * @param thetaController the PID controller for the drivetrain's rotation
   * @param subsystem the drivetrain subsystem required by this command
   */
  public FollowRobotRelativePath(
      PathPlannerTrajectory trajectory, PIDController thetaController, Drivetrain subsystem) {
    super(
        trajectory,
        subsystem::getPose,
        KINEMATICS,
        subsystem.getAutoXController(),
        subsystem.getAutoYController(),
        subsystem.getAutoThetaController(),
        subsystem::setSwerveModuleStates,
        subsystem);

    this.thetaController = thetaController;
    this.drivetrain = subsystem;
    this.trajectory = trajectory;
  }

  /**
   * This method is invoked once when this command is scheduled. If the trajectory is the first in a
   * sequence of trajectories or the only trajector, initialize the gyro and odometry to match the
   * start of trajectory. PathPlanner sets the origin of the field to the lower left corner (i.e.,
   * the corner of the field to the driver's right). Zero degrees is away from the driver and
   * increases in the CCW direction. It is critical that this initialization occurs in this method
   * and not the constructor as this object is constructed well before the command is scheduled.
   */
  @Override
  public void initialize() {
    super.initialize();

    // set the pose offset such that getPose() returns the starting pose of the trajectory
    // (realPose + poseOffset = initialPose)
    poseOffset = trajectory.getInitialPose().minus(drivetrain.getPose());

    // reset the theta controller such that old accumuldated ID values aren't used with the new path
    //      this doesn't matter if only the P value is non-zero, which is the current behavior
    this.thetaController.reset();
  }

  /** Get the robot's pose with the pose offset applied */
  public Pose2d getPose() {
    return this.drivetrain.getPose().plus(this.poseOffset);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    this.drivetrain.stop();
    super.end(interrupted);
  }
}
