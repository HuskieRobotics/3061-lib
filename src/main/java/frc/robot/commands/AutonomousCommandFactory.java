package frc.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.Field2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonomousCommandFactory {

  private static AutonomousCommandFactory autonomousCommandFactory = null;

  // arbitrary, find the actual starting poses
  private Pose2d blueLeftStartingAutoPose =
      new Pose2d(7.017, 6.076, Rotation2d.fromDegrees(-132.957));
  private Pose2d blueRightStartingAutoPose =
      new Pose2d(6.972, 1.884, Rotation2d.fromDegrees(145.333));
  private Pose2d redLeftStartingAutoPose = FlippingUtil.flipFieldPose(blueLeftStartingAutoPose);
  private Pose2d redRightStartingAutoPose = FlippingUtil.flipFieldPose(blueRightStartingAutoPose);

  // set arbitrary tolerance values to 3 inches in each direction and 5 degrees
  private Transform2d autoStartTolerance =
      new Transform2d(
          Units.inchesToMeters(3),
          Units.inchesToMeters(3),
          new Rotation2d(Units.degreesToRadians(5)));

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);

  /**
   * Returns the singleton instance of this class.
   *
   * @return the singleton instance of this class
   */
  public static AutonomousCommandFactory getInstance() {
    if (autonomousCommandFactory == null) {
      autonomousCommandFactory = new AutonomousCommandFactory();
    }
    return autonomousCommandFactory;
  }

  private AutonomousCommandFactory() {}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void configureAutoCommands(Drivetrain drivetrain, Vision vision) {
    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    Command startPoint =
        Commands.runOnce(
            () -> {
              try {
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("Start Point").getStartingDifferentialPose());
              } catch (Exception e) {
                pathFileMissingAlert.setText("Could not find the specified path file: Start Point");
                pathFileMissingAlert.set(true);
              }
            },
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Distance Test ************
     *
     * used for empirically determining the wheel radius
     *
     */
    autoChooser.addOption(
        "Distance Test Slow", createTuningAutoPath("DistanceTestSlow", true, drivetrain));
    autoChooser.addOption(
        "Distance Test Med", createTuningAutoPath("DistanceTestMed", true, drivetrain));
    autoChooser.addOption(
        "Distance Test Fast", createTuningAutoPath("DistanceTestFast", true, drivetrain));

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    autoChooser.addOption(
        "Rotation Test Slow", createTuningAutoPath("RotationTestSlow", false, drivetrain));
    autoChooser.addOption(
        "Rotation Test Fast", createTuningAutoPath("RotationTestFast", false, drivetrain));

    autoChooser.addOption(
        "Oval Test Slow", createTuningAutoPath("OvalTestSlow", false, drivetrain));
    autoChooser.addOption(
        "Oval Test Fast", createTuningAutoPath("OvalTestFast", false, drivetrain));

    /************ Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    autoChooser.addOption("Drive Velocity Tuning", this.getDriveVelocityTuningCommand(drivetrain));

    /************ Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    autoChooser.addOption(
        "Swerve Rotation Tuning", this.getSwerveRotationTuningCommand(drivetrain));

    /************ Drive Wheel Radius Characterization ************
     *
     * useful for characterizing the drive wheel Radius
     *
     */
    autoChooser.addOption( // start by driving slowing in a circle to align wheels
        "Drive Wheel Radius Characterization",
        this.getDriveWheelRadiusCharacterizationCommand(drivetrain));
  }

  private Command getDriveVelocityTuningCommand(Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
        Commands.repeatingSequence(
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(2.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(-0.5, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(3.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain))));
  }

  private Command getSwerveRotationTuningCommand(Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
        Commands.repeatingSequence(
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(0.1, 0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-0.1, 0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(-0.1, -0.1, 0.0, true, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(0.5),
                Commands.run(() -> drivetrain.drive(0.1, -0.1, 0.0, true, false), drivetrain))));
  }

  private Command getDriveWheelRadiusCharacterizationCommand(Drivetrain drivetrain) {
    return CharacterizationCommands.wheelRadiusCharacterization(drivetrain);
  }

  private Command createTuningAutoPath(
      String autoName, boolean measureDistance, Drivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::captureInitialConditions),
        new PathPlannerAuto(autoName),
        Commands.runOnce(() -> drivetrain.captureFinalConditions(autoName, measureDistance)));
  }

  public boolean alignedToStartingPose() {

    // find the target position
    Transform2d difference;

    if (Field2d.getInstance().getAlliance() == Alliance.Blue) {
      difference =
          RobotOdometry.getInstance().getEstimatedPose().getY() > (FieldConstants.fieldWidth / 2.0)
              ? RobotOdometry.getInstance().getEstimatedPose().minus(blueLeftStartingAutoPose)
              : RobotOdometry.getInstance().getEstimatedPose().minus(blueRightStartingAutoPose);
    } else {
      difference =
          RobotOdometry.getInstance().getEstimatedPose().getY() > (FieldConstants.fieldWidth / 2.0)
              ? RobotOdometry.getInstance().getEstimatedPose().minus(redRightStartingAutoPose)
              : RobotOdometry.getInstance().getEstimatedPose().minus(redLeftStartingAutoPose);
    }

    boolean isAligned =
        Math.abs(difference.getX()) < autoStartTolerance.getX()
            && Math.abs(difference.getY()) < autoStartTolerance.getY()
            && Math.abs(difference.getRotation().getRadians())
                < autoStartTolerance.getRotation().getRadians();

    // this method will be invoked in something like our disabledPeriodic method
    Logger.recordOutput("PathFinding/alignedForAuto", isAligned);
    Logger.recordOutput("PathFinding/xDiff", difference.getX());
    Logger.recordOutput("PathFinding/yDiff", difference.getY());
    Logger.recordOutput("PathFinding/rotDiff", difference.getRotation().getDegrees());

    return isAligned;
  }
}
