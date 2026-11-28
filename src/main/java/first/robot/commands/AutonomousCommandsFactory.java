package first.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import first.lib.differential_drivetrain.DifferentialDrivetrain;
import first.lib.swerve_drivetrain.SwerveDrivetrain;
import first.lib.util.RobotOdometry;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import org.wpilib.util.Alert;

public class AutonomousCommandsFactory {

  private static AutonomousCommandsFactory autonomousCommandFactory = null;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final Selectable<Command> autoChooser = new Selectable<>();

  private final Alert pathFileMissingAlert =
      new Alert("autoPathFileMissing", "Could not find the specified path file.", Alert.Level.HIGH);

  private Timer matchTimer;
  private Pose2d currentTargetPose;
  private final Debouncer fallenBehindPathDebouncer = new Debouncer(0.2);

  /**
   * Returns the singleton instance of this class.
   *
   * @return the singleton instance of this class
   */
  public static AutonomousCommandsFactory getInstance() {
    if (autonomousCommandFactory == null) {
      autonomousCommandFactory = new AutonomousCommandsFactory();
    }
    return autonomousCommandFactory;
  }

  private AutonomousCommandsFactory() {
    matchTimer = new Timer();
    currentTargetPose = new Pose2d();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void configureAutoCommands(SwerveDrivetrain drivetrain) {
    // add commands to the auto chooser
    autoChooser.addDefault("Do Nothing", new InstantCommand());

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
    autoChooser.add("Start Point", startPoint);

    /************ Distance Test ************
     *
     * used for empirically determining the wheel radius
     *
     */
    autoChooser.add(
        "Distance Test Slow", createTuningAutoPath("DistanceTestSlow", true, drivetrain));
    autoChooser.add("Distance Test Med", createTuningAutoPath("DistanceTestMed", true, drivetrain));
    autoChooser.add(
        "Distance Test Fast", createTuningAutoPath("DistanceTestFast", true, drivetrain));

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    autoChooser.add(
        "Rotation Test Slow", createTuningAutoPath("RotationTestSlow", false, drivetrain));
    autoChooser.add(
        "Rotation Test Fast", createTuningAutoPath("RotationTestFast", false, drivetrain));

    autoChooser.add("Oval Test Slow", createTuningAutoPath("OvalTestSlow", false, drivetrain));
    autoChooser.add("Oval Test Fast", createTuningAutoPath("OvalTestFast", false, drivetrain));

    /************ Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    autoChooser.add("Drive Velocity Tuning", this.getDriveVelocityTuningCommand(drivetrain));

    /************ Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    autoChooser.add("Swerve Rotation Tuning", this.getSwerveRotationTuningCommand(drivetrain));

    /************ Drive Wheel Radius Characterization ************
     *
     * useful for characterizing the drive wheel Radius
     *
     */
    autoChooser.add( // start by driving slowing in a circle to align wheels
        "Drive Wheel Radius Characterization",
        this.getDriveWheelRadiusCharacterizationCommand(drivetrain));

    /************ Drive Wheel Radius Check ************
     *
     * useful for checking the drive wheel radius
     *
     */
    autoChooser.add( // start by driving slowing in a circle to align wheels
        "Drive Wheel Radius Check", this.getDriveWheelRadiusCheckCommand(drivetrain));

    Tunables.publish("Auto Routine", autoChooser);
  }

  public void configureAutoCommands(DifferentialDrivetrain drivetrain) {
    // add commands to the auto chooser
    autoChooser.addDefault("Do Nothing", new InstantCommand());

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
    autoChooser.add("Start Point", startPoint);

    /************ Differential Auto ************
     *
     * example PathPlanner auto for XRP
     *
     */

    autoChooser.add("Differential Example", new PathPlannerAuto("Differential Auto"));

    Tunables.publish("Auto Routine", autoChooser);
  }

  private Command getDriveVelocityTuningCommand(SwerveDrivetrain drivetrain) {
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
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(3.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(1.0),
                Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, false, false), drivetrain)),
            Commands.deadline(
                Commands.waitSeconds(2.0),
                Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain))));
  }

  private Command getSwerveRotationTuningCommand(SwerveDrivetrain drivetrain) {
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

  private Command getDriveWheelRadiusCharacterizationCommand(SwerveDrivetrain drivetrain) {
    return CharacterizationCommands.wheelRadiusCharacterization(drivetrain);
  }

  private Command getDriveWheelRadiusCheckCommand(SwerveDrivetrain drivetrain) {
    return Commands.sequence(
        Commands.deadline(
            Commands.waitSeconds(1.0),
            Commands.run(() -> drivetrain.drive(0.5, 0.0, 0.0, true, false), drivetrain)),
        Commands.run(() -> drivetrain.drive(0.0, 0.0, 0.0, true, false), drivetrain));
  }

  private Command createTuningAutoPath(
      String autoName, boolean measureDistance, SwerveDrivetrain drivetrain) {
    return Commands.sequence(
        Commands.runOnce(drivetrain::captureInitialConditions),
        new PathPlannerAuto(autoName),
        Commands.runOnce(() -> drivetrain.captureFinalConditions(autoName, measureDistance)));
  }

  private boolean fallenBehindPath() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();

    Transform2d diff = pose.minus(getPathFollowingTargetPose());
    double dist = Math.hypot(diff.getX(), diff.getY());

    // check match timer to make sure we don't trigger this if the target pose isn't updated yet
    boolean fallenBehind =
        fallenBehindPathDebouncer.calculate(matchTimer.get() > 0.5 && dist > 2.0);

    Logger.recordOutput("Auto/FallenBehindPath/Difference", dist);
    Logger.recordOutput("Auto/FallenBehindPath/FallenBehind", fallenBehind);

    return fallenBehind;
  }

  private Command setStartingPoseForAuto(Pose2d startingPose, SwerveDrivetrain drivetrain) {
    return Commands.runOnce(
        () -> {
          Pose2d pose = startingPose;
          if (drivetrain.shouldFlipAutoPath()) {
            pose = FlippingUtil.flipFieldPose(startingPose);
          }
          drivetrain.resetPose(pose);
        });
  }

  public void setPathFollowingTargetPose(Pose2d pose) {
    currentTargetPose = pose;
  }

  public Pose2d getPathFollowingTargetPose() {
    return currentTargetPose;
  }

  public double getCustomMatchTime() {
    return matchTimer.get();
  }
}
