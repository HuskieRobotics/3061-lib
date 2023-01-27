package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import org.littletonrobotics.junction.Logger;

public class MoveToGrid extends CommandBase {
  private Drivetrain drivetrain;
  private PathPlannerTrajectory trajectory;
  private PPSwerveControllerCommand ppSwerveControllerCommand;

  public MoveToGrid(Drivetrain subsystem) {
    // no requirements for movetogrid, drivetrain for ppswerve
    this.drivetrain = subsystem;
  }

  public void initialize() {
    // reset the theta controller such that old accumulated ID values aren't used with the new path
    //      this doesn't matter if only the P value is non-zero, which is the current behavior
    this.drivetrain.getAutoXController().reset();
    this.drivetrain.getAutoYController().reset();
    this.drivetrain.getAutoThetaController().reset();

    // Current verison is for grid 1, node 3
    this.trajectory =
        FieldConstants.COMMUNITY_ZONE.makePath(
            this.drivetrain.getPose(),
            FieldConstants.GRID_1_NODE_3,
            new PathConstraints(
                DrivetrainConstants.AUTO_MAX_SPEED_METERS_PER_SECOND,
                DrivetrainConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED));

    this.ppSwerveControllerCommand =
        new PPSwerveControllerCommand(
            this.trajectory,
            this.drivetrain::getPose,
            DrivetrainConstants.KINEMATICS,
            this.drivetrain.getAutoXController(),
            this.drivetrain.getAutoYController(),
            this.drivetrain.getAutoThetaController(),
            this.drivetrain::setSwerveModuleStates,
            this.drivetrain);
    this.ppSwerveControllerCommand.schedule();

    Logger.getInstance().recordOutput("Odometry/trajectory", trajectory);
  }

  public boolean isFinished(){
    return ppSwerveControllerCommand.isFinished();
  }

  public void end(boolean interrupted) {
    this.drivetrain.stop();
  }
}
