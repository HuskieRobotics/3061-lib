package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.robot.Field2d;
import frc.robot.FieldRegionConstants;
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
public class MoveToLoadingZone extends MoveToPose {
  private final Pose2d endPose;
  private double marginOfError = Units.inchesToMeters(24);

  /**
   * Constructs a new MoveToGrid command object.
   *
   * @param subsystem the drivetrain subsystem required by this command
   */
  // FIXME: Remove this constructor if we need to account for path traversal times
  public MoveToLoadingZone(Drivetrain subsystem, Pose2d endPose) {
    this(subsystem, endPose, 0);
  }

  public MoveToLoadingZone(Drivetrain subsystem, Pose2d endPose, double minPathTraversalTime) {
    super(subsystem, minPathTraversalTime);
    this.endPose = endPose;
  }

  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/MoveToLoadingZone", true);
    super.initialize();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    Logger.getInstance().recordOutput("ActiveCommands/MoveToLoadingZone", false);
  }

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
    Pose2d translatedEndPose;

    if (this.endPose == FieldRegionConstants.SINGLE_SUBSTATION) {
      translatedEndPose =
          new Pose2d(
              this.endPose.getX(),
              this.endPose.getY()
                  - RobotConfig.getInstance().getRobotWidthWithBumpers() / 2
                  - marginOfError,
              this.endPose.getRotation());
    } else {
      translatedEndPose =
          new Pose2d(
              this.endPose.getX()
                  - RobotConfig.getInstance().getRobotWidthWithBumpers() / 2
                  - marginOfError,
              this.endPose.getY(),
              this.endPose.getRotation());
    }

    translatedEndPose = Field2d.getInstance().mapPoseToCurrentAlliance(translatedEndPose);

    Logger.getInstance().recordOutput("MoveToLoadingZone/endPose", translatedEndPose);

    return translatedEndPose;
  }
}
