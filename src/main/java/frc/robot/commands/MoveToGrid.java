package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import org.littletonrobotics.junction.Logger;

public class MoveToGrid extends CommandBase {
  private Drivetrain drivetrain;
  private PathPlannerTrajectory trajectory;
  private PPSwerveControllerCommand ppSwerveControllerCommand;
  private Alert noPathAlert = new Alert("No path between start and end pose", AlertType.WARNING);

  public MoveToGrid(Drivetrain subsystem) {
    // no requirements for movetogrid, drivetrain for ppswerve
    this.drivetrain = subsystem;

    FieldConstants.COMMUNITY_REGION_1.addNeighbor(
        FieldConstants.COMMUNITY_REGION_2, FieldConstants.REGION_1_2_TRANSITION_POINT);
    FieldConstants.COMMUNITY_REGION_2.addNeighbor(
        FieldConstants.COMMUNITY_REGION_1, FieldConstants.REGION_1_2_TRANSITION_POINT);
    FieldConstants.COMMUNITY_REGION_1.addNeighbor(
        FieldConstants.COMMUNITY_REGION_3, FieldConstants.REGION_1_3_TRANSITION_POINT);
    FieldConstants.COMMUNITY_REGION_3.addNeighbor(
        FieldConstants.COMMUNITY_REGION_1, FieldConstants.REGION_1_3_TRANSITION_POINT);
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

    noPathAlert.set(this.trajectory == null);

    if (this.trajectory != null) {
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
    // FIXME: add alert that no path was found
  }

  public boolean isFinished() {
    return this.trajectory == null || ppSwerveControllerCommand.isFinished();
  }

  public void end(boolean interrupted) {
    if (this.trajectory != null) {
      this.drivetrain.stop();
    }
    this.trajectory = null;
  }
}
