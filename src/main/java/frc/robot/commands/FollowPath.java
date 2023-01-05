package frc.robot.commands;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to follow the specified
 * trajectory, presumably during the autonomous period. The superclass' execute method invokes the
 * drivetrain subsystem's setSwerveModuleStates method to follow the trajectory.
 *
 * <p>Requires: the Drivetrain subsystem (handled by superclass)
 *
 * <p>Finished When: the time of the specified path has elapsed (handled by superclass)
 *
 * <p>At End: stops the drivetrain
 */
public class FollowPath extends PPSwerveControllerCommand {
  private Drivetrain drivetrain;
  private PathPlannerTrajectory trajectory;
  private boolean initialPath;

  /**
   * Constructs a new FollowPath object.
   *
   * @param trajectory the specified trajectory created by PathPlanner
   * @param subsystem the drivetrain subsystem required by this command
   * @param initialPath true, if this trajectory is the first in a sequence of trajectories or the
   *     only trajectory, in which case the gyro and odometry will be initialized to match the start
   *     of trajectory; false, if this trajectory is a subsequent trajectory in which case the gyro
   *     and odometry will not be re-initialized in order to ensure a smooth transition between
   *     trajectories
   */
  public FollowPath(PathPlannerTrajectory trajectory, Drivetrain subsystem, boolean initialPath) {
    super(
        trajectory,
        subsystem::getPose,
        KINEMATICS,
        subsystem.getAutoXController(),
        subsystem.getAutoYController(),
        subsystem.getAutoThetaController(),
        subsystem::setSwerveModuleStates,
        subsystem);

    this.drivetrain = subsystem;
    this.trajectory = trajectory;
    this.initialPath = initialPath;
  }

  /**
   * This method is invoked once when this command is scheduled. If the trajectory is the first in a
   * sequence of trajectories or the only trajectory, initialize the gyro and odometry to match the
   * start of trajectory. PathPlanner sets the origin of the field to the lower left corner (i.e.,
   * the corner of the field to the driver's right). Zero degrees is away from the driver and
   * increases in the CCW direction. It is critical that this initialization occurs in this method
   * and not the constructor as this object is constructed well before the command is scheduled.
   */
  @Override
  public void initialize() {
    super.initialize();

    if (initialPath) {
      // reset odometry to the starting pose of the trajectory
      this.drivetrain.resetOdometry(this.trajectory.getInitialState());
    }

    // reset the theta controller such that old accumulated ID values aren't used with the new path
    //      this doesn't matter if only the P value is non-zero, which is the current behavior
    this.drivetrain.getAutoXController().reset();
    this.drivetrain.getAutoYController().reset();
    this.drivetrain.getAutoThetaController().reset();

    Logger.getInstance().recordOutput("Odometry/trajectory", trajectory);
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
