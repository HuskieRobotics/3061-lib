package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.robot.Field2d;
import frc.robot.FieldRegionConstants;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.operator_interface.OperatorInterface.Node;
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
public class MoveToGrid extends MoveToPose {
  private OperatorInterface oi;
  private double marginOfError = Units.inchesToMeters(6);
  private double marginOfErrorEndNodes = Units.inchesToMeters(2);

  /**
   * Constructs a new MoveToGrid command object.
   *
   * @param subsystem the drivetrain subsystem required by this command
   */
  // FIXME: Remove this constructor if we need to account for path traversal times
  public MoveToGrid(Drivetrain subsystem) {
    super(subsystem);
  }

  public MoveToGrid(Drivetrain subsystem, double minPathTraversalTime) {
    super(subsystem, minPathTraversalTime);
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/MoveToGrid", true);

    this.oi = OISelector.getOperatorInterface();
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Logger.getInstance().recordOutput("ActiveCommands/MoveToGrid", false);
  }

  // TODO: add minPathTraversalTime

  public Supplier<Pose2d> endPoseSupplier() {
    return this::endPose;
  }

  /**
   * This method returns a Pose2d object meant for the end pose of the auto-generated paths based on
   * the current values of the operator console switches.
   *
   * @return the end pose for a path
   */
  @Override
  public Pose2d endPose() {
    Pose2d endPose = new Pose2d();
    Node node = this.oi.getNode();
    switch (node) {
      case NODE_INVALID:
        break;
      case NODE_1:
        endPose = FieldRegionConstants.GRID_1_NODE_1;
        endPose =
            Field2d.getInstance()
                .mapPoseToCurrentAlliance(
                    new Pose2d(
                        endPose.getX()
                            + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2
                            + marginOfErrorEndNodes,
                        endPose.getY(),
                        endPose.getRotation()));
        Logger.getInstance().recordOutput("MoveToGrid/endPose", endPose);
        return endPose;
      case NODE_2:
        endPose = FieldRegionConstants.GRID_1_NODE_2;
        break;
      case NODE_3:
        endPose = FieldRegionConstants.GRID_1_NODE_3;
        break;
      case NODE_4:
        endPose = FieldRegionConstants.GRID_2_NODE_1;
        break;
      case NODE_5:
        endPose = FieldRegionConstants.GRID_2_NODE_2;
        break;
      case NODE_6:
        endPose = FieldRegionConstants.GRID_2_NODE_3;
        break;
      case NODE_7:
        endPose = FieldRegionConstants.GRID_3_NODE_1;
        break;
      case NODE_8:
        endPose = FieldRegionConstants.GRID_3_NODE_2;
        break;
      case NODE_9:
        endPose = FieldRegionConstants.GRID_3_NODE_3;
        endPose =
            Field2d.getInstance()
                .mapPoseToCurrentAlliance(
                    new Pose2d(
                        endPose.getX()
                            + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2
                            + marginOfErrorEndNodes,
                        endPose.getY(),
                        endPose.getRotation()));
        Logger.getInstance().recordOutput("MoveToGrid/endPose", endPose);
        return endPose;
    }
    endPose =
        new Pose2d(
            endPose.getX()
                + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2
                + marginOfError,
            endPose.getY(),
            endPose.getRotation());

    endPose = Field2d.getInstance().mapPoseToCurrentAlliance(endPose);

    Logger.getInstance().recordOutput("MoveToGrid/endPose", endPose);

    return endPose;
  }
}
