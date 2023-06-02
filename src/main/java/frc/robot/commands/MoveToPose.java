package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Field2d;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, generates a trajectory from the current pose (projected into the
 * future to account for the average latency introduced by generating the trajectory) of the robot
 * to a specified pose on the field and instructs the drivetrain subsystem to follow the specified
 * trajectory using a PPSwerveControllerCommand object. The specified pose will be translated based
 * on the alliance color. The initial speed will match the current speed and the final speed will
 * match the speed returned by the RobotConfig's getStallAgainstElementVelocity method. The
 * PPSwerveControllerCommand contained by this command will never be scheduled; this command invoke
 * each of the command methods at the appropriate time.
 *
 * <p>For following a predetermined trajectory, refer to the FollowPath Command class. For driving
 * in a straight line to a pose, refer to the DriveToPose Command class.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: trajectory is null or time of specified trajectory has elapsed (from
 * PPSwerveControllerCommand object)
 *
 * <p>At End: stops the drivetrain, sets trajectory to null
 */
public class MoveToPose extends CommandBase {
  private Supplier<Pose2d> endPoseSupplier;
  private Drivetrain drivetrain;
  private double minTrajectoryTraversalTime;
  private PathPlannerTrajectory trajectory;
  private PPSwerveControllerCommand ppSwerveControllerCommand;
  private Alert noTrajectoryAlert =
      new Alert("No trajectory between start and end pose", AlertType.WARNING);
  private final TunableNumber timeOffset = new TunableNumber("MoveToPose/TIME_OFFSET", 0.1);

  /**
   * Constructs a new MoveToPose command object. A pose supplier is specified instead of a pose
   * since the target pose may not be known when this command is created.
   *
   * @param subsystem the drivetrain subsystem required by this command
   * @param endPoseSupplier a supplier of the end pose of the trajectory
   */
  public MoveToPose(Drivetrain subsystem, Supplier<Pose2d> endPoseSupplier) {
    this(subsystem, endPoseSupplier, 0);
  }

  /**
   * Constructs a new MoveToPose command object. A pose supplier is specified instead of a pose
   * since the target pose may not be known when this command is created. The generated trajectory
   * will be constrained such that the time to traverse the trajectory is at least the specified
   * minimum time. This is useful for coordinating this command with the motion of a mechanism.
   *
   * @param subsystem
   * @param endPoseSupplier
   * @param minTrajectoryTraversalTime
   */
  public MoveToPose(
      Drivetrain subsystem, Supplier<Pose2d> endPoseSupplier, double minTrajectoryTraversalTime) {
    this.endPoseSupplier = endPoseSupplier;
    this.drivetrain = subsystem;
    this.minTrajectoryTraversalTime = minTrajectoryTraversalTime;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/MoveToPose", true);

    // reset the theta controller such that old accumulated ID values aren't used with the new
    // trajectory
    //      this doesn't matter if only the P value is non-zero, which is the current behavior
    this.drivetrain.getAutoXController().reset();
    this.drivetrain.getAutoYController().reset();
    this.drivetrain.getAutoThetaController().reset();

    double beforeCalc = Logger.getInstance().getRealTimestamp();

    Pose2d endPose = endPose();

    // calculate the starting pose by projecting the current pose forward in time (assuming the
    // robot is not rotating)
    Pose2d startingPose =
        new Pose2d(
            this.drivetrain.getPose().getX() + this.drivetrain.getVelocityX() * timeOffset.get(),
            this.drivetrain.getPose().getY() + this.drivetrain.getVelocityY() * timeOffset.get(),
            this.drivetrain.getPose().getRotation());

    // set the maximum velocity to accommodate the minimum trajectory traversal time
    double distance = endPose.minus(this.drivetrain.getPose()).getTranslation().getNorm();
    double maxVelocity = RobotConfig.getInstance().getAutoMaxSpeed();
    if (minTrajectoryTraversalTime != 0) {
      maxVelocity = distance / minTrajectoryTraversalTime;
    }
    maxVelocity = Math.min(maxVelocity, RobotConfig.getInstance().getAutoMaxSpeed());

    // the Field2d singleton generates the trajectory from the poses and constraints
    this.trajectory =
        Field2d.getInstance()
            .makePath(
                startingPose,
                endPose,
                new PathConstraints(
                    maxVelocity, RobotConfig.getInstance().getAutoMaxAcceleration()),
                this.drivetrain);

    noTrajectoryAlert.set(this.trajectory == null);

    if (this.trajectory != null) {
      // create the wrapped PPSwerveControllerCommand object and invoke its initialize method since
      // it will never be scheduled
      this.ppSwerveControllerCommand =
          new PPSwerveControllerCommand(
              this.trajectory,
              this.drivetrain::getPose,
              RobotConfig.getInstance().getSwerveDriveKinematics(),
              this.drivetrain.getAutoXController(),
              this.drivetrain.getAutoYController(),
              this.drivetrain.getAutoThetaController(),
              this.drivetrain::setSwerveModuleStates);
      this.ppSwerveControllerCommand.initialize();

      double afterCalc = Logger.getInstance().getRealTimestamp();
      Logger.getInstance().recordOutput("Odometry/trajectoryCalcTime", afterCalc - beforeCalc);

      Logger.getInstance().recordOutput("Odometry/trajectory", trajectory);
    }
  }

  /**
   * This method is invoked periodically while this command is scheduled. It executes the contained
   * PPSwerveControllerCommand object.
   */
  @Override
  public void execute() {
    if (this.ppSwerveControllerCommand != null) {
      this.ppSwerveControllerCommand.execute();
    }
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). This command is considered finished if the
   * move-to-pose feature is disabled on the drivetrain subsystem, if a trajectory failed to be
   * generated, or if the contained PPSwerveControllerCommand command is finished.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    return !drivetrain.isMoveToPoseEnabled()
        || this.trajectory == null
        || ppSwerveControllerCommand.isFinished();
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain if a trajectory was successfully created.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    if (this.ppSwerveControllerCommand != null) {
      this.ppSwerveControllerCommand.end(interrupted);
    }

    if (this.trajectory != null) {
      this.drivetrain.stop();
    }
    this.trajectory = null;
    this.ppSwerveControllerCommand = null;

    Logger.getInstance().recordOutput("ActiveCommands/MoveToPose", false);
  }

  /**
   * Returns the total time in seconds of the trajectory.
   *
   * @return the total time in seconds of the trajectory
   */
  public double getTotalTimeSeconds() {
    if (this.trajectory != null) {
      return this.trajectory.getTotalTimeSeconds();
    } else {
      return 0;
    }
  }

  /**
   * Calculates the end pose based on the pose from the pose supplier. The end pose is mapped based
   * on the alliance color.
   *
   * <p>This method can be overridden to translate the end pose for a specific scenario (e.g.,
   * account for the robot width).
   */
  protected Pose2d endPose() {
    Pose2d endPose = endPoseSupplier.get();

    Pose2d translatedEndPose = Field2d.getInstance().mapPoseToCurrentAlliance(endPose);

    Logger.getInstance().recordOutput("MoveToPose/endPose", translatedEndPose);

    return translatedEndPose;
  }
}
