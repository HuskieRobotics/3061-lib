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
 * This command, when executed, generates a path from the current pose of the robot to a specified
 * point on the field and instructs the drivetrain subsystem to follow the specified trajectory
 * using a PPSwerveControllerCommand object.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: trajectory is null or time of specified path has elapsed (from
 * PPSwerveControllerCommand object)
 *
 * <p>At End: stops the drivetrain, sets trajectory to null
 */
public class MoveToPose extends CommandBase {
  private Supplier<Pose2d> endPoseSupplier;
  private Drivetrain drivetrain;
  private double minPathTraversalTime;
  private PathPlannerTrajectory trajectory;
  private PPSwerveControllerCommand ppSwerveControllerCommand;
  private Alert noPathAlert = new Alert("No path between start and end pose", AlertType.WARNING);
  private final TunableNumber timeOffset = new TunableNumber("MoveToPose/TIME_OFFSET", 0.1);

  /**
   * Constructs a new MoveToPose command object.
   *
   * @param subsystem the drivetrain subsystem required by this command
   */
  protected MoveToPose(Drivetrain subsystem, Supplier<Pose2d> endPoseSupplier) {
    this(subsystem, endPoseSupplier, 0);
  }

  protected MoveToPose(
      Drivetrain subsystem, Supplier<Pose2d> endPoseSupplier, double minPathTraversalTime) {
    this.endPoseSupplier = endPoseSupplier;
    this.drivetrain = subsystem;
    this.minPathTraversalTime = minPathTraversalTime;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/MoveToPose", true);

    // reset the theta controller such that old accumulated ID values aren't used with the new path
    //      this doesn't matter if only the P value is non-zero, which is the current behavior
    this.drivetrain.getAutoXController().reset();
    this.drivetrain.getAutoYController().reset();
    this.drivetrain.getAutoThetaController().reset();

    double beforeCalc = Logger.getInstance().getRealTimestamp();

    Pose2d endPose = endPose();
    Pose2d startingPose =
        new Pose2d(
            this.drivetrain.getPose().getX() + this.drivetrain.getVelocityX() * timeOffset.get(),
            this.drivetrain.getPose().getY() + this.drivetrain.getVelocityY() * timeOffset.get(),
            this.drivetrain.getPose().getRotation());

    double distance = endPose.minus(this.drivetrain.getPose()).getTranslation().getNorm();
    double maxVelocity = RobotConfig.getInstance().getAutoMaxSpeed();
    if (minPathTraversalTime != 0) {
      maxVelocity = distance / minPathTraversalTime;
    }
    maxVelocity = Math.min(maxVelocity, RobotConfig.getInstance().getAutoMaxSpeed());

    this.trajectory =
        Field2d.getInstance()
            .makePath(
                startingPose,
                endPose,
                new PathConstraints(
                    maxVelocity, RobotConfig.getInstance().getAutoMaxAcceleration()),
                this.drivetrain);

    noPathAlert.set(this.trajectory == null);

    if (this.trajectory != null) {
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

  @Override
  public void execute() {
    if (this.ppSwerveControllerCommand != null) {
      this.ppSwerveControllerCommand.execute();
    }
  }

  @Override
  public boolean isFinished() {
    return !drivetrain.isMoveToGridEnabled()
        || this.trajectory == null
        || ppSwerveControllerCommand.isFinished();
  }

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

  public double getTotalTimeSeconds() {
    return this.trajectory.getTotalTimeSeconds();
  }

  private Pose2d endPose() {
    Pose2d endPose = endPoseSupplier.get();

    Pose2d translatedEndPose =
        new Pose2d(
            endPose.getX() - RobotConfig.getInstance().getRobotWidthWithBumpers() / 2,
            endPose.getY(),
            endPose.getRotation());

    translatedEndPose = Field2d.getInstance().mapPoseToCurrentAlliance(translatedEndPose);

    Logger.getInstance().recordOutput("MoveToPose/endPose", translatedEndPose);

    return translatedEndPose;
  }
}
