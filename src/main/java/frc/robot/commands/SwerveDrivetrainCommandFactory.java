package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainConstants;
import frc.lib.team3061.vision.Vision;
import frc.lib.team6328.util.FieldConstants;
import frc.robot.Field2d;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import java.util.Optional;
import java.util.function.Supplier;

public class SwerveDrivetrainCommandFactory {

  private static final double WALL_SNAP_TOLERANCE_METERS = Units.inchesToMeters(30);

  private static Trigger driveToPoseCanceledTrigger;

  private SwerveDrivetrainCommandFactory() {}

  public static void registerCommands(
      OperatorInterface oi, SwerveDrivetrain swerveDrivetrain, Vision vision) {
    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    swerveDrivetrain.setDefaultCommand(getDefaultTeleopSwerveCommand(oi, swerveDrivetrain));

    driveToPoseCanceledTrigger = new Trigger(swerveDrivetrain::getDriveToPoseCanceled);
    driveToPoseCanceledTrigger.onTrue(
        Commands.sequence(
                Commands.run(
                        () -> LEDs.getInstance().requestState(LEDs.States.DRIVE_TO_POSE_CANCELED),
                        swerveDrivetrain)
                    .withTimeout(0.5),
                Commands.runOnce(
                    () -> swerveDrivetrain.setDriveToPoseCanceled(false), swerveDrivetrain))
            .withName("cancel drive to pose"));

    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                    Commands.runOnce(swerveDrivetrain::disableFieldRelative, swerveDrivetrain),
                    Commands.runOnce(swerveDrivetrain::enableFieldRelative, swerveDrivetrain),
                    swerveDrivetrain::getFieldRelative)
                .withName("toggle field relative"));

    // slow-mode toggle
    oi.getLimitAccelerationAndVelocityButton()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      swerveDrivetrain.enableTranslationSlowMode();
                      swerveDrivetrain.enableRotationSlowMode();
                    })
                .withName("enable slow mode"));

    oi.getLimitAccelerationAndVelocityButton()
        .onFalse(
            Commands.runOnce(
                    () -> {
                      swerveDrivetrain.disableTranslationSlowMode();
                      swerveDrivetrain.disableRotationSlowMode();
                    })
                .withName("disable slow mode"));

    // reset gyro to 0 degrees
    oi.getResetGyroButton()
        .onTrue(
            Commands.runOnce(swerveDrivetrain::zeroGyroscope, swerveDrivetrain)
                .withName("zero gyro"));

    // reset pose based on vision
    oi.getResetPoseToVisionButton()
        .onTrue(
            Commands.repeatingSequence(Commands.none())
                .until(() -> vision.getBestRobotPose() != null)
                .andThen(
                    Commands.runOnce(
                        () -> swerveDrivetrain.resetPoseToVision(() -> vision.getBestRobotPose())))
                .ignoringDisable(true)
                .withName("reset pose to vision"));

    // x-stance
    oi.getXStanceButton()
        .whileTrue(
            Commands.run(swerveDrivetrain::holdXstance, swerveDrivetrain)
                .withName("hold x-stance"));

    // print pose to console for field calibration
    // format the string so that it shows how to make the pose2d object given our current x
    // (double), current y (double), and current rotation (Rotation2d)
    oi.getCurrentPoseButton()
        .onTrue(
            Commands.runOnce(
                    () ->
                        System.out.println(
                            "new Pose2d("
                                + swerveDrivetrain.getPose().getTranslation().getX()
                                + ", "
                                + swerveDrivetrain.getPose().getTranslation().getY()
                                + ", Rotation2d.fromDegrees("
                                + swerveDrivetrain.getPose().getRotation().getDegrees()
                                + "));"))
                .ignoringDisable(true)
                .withName("print current pose"));

    // new Trigger(
    //         () -> {
    //           return drivetrain.isTilted();
    //         })
    //     .whileTrue(Commands.run(() -> drivetrain.untilt(), drivetrain).withName("untilt"));

    FaultReporter.getInstance()
        .registerSystemCheck(
            SwerveDrivetrainConstants.SUBSYSTEM_NAME,
            swerveDrivetrain.getSystemCheckCommand(),
            oi.getDrivetrainSystemTest());
  }

  public static Command getDefaultTeleopSwerveCommand(
      OperatorInterface oi, SwerveDrivetrain drivetrain) {
    return new TeleopSwerve(
            drivetrain,
            oi::getTranslateX,
            oi::getTranslateY,
            oi::getRotate,
            getTeleopSwerveAngleSupplier(drivetrain))
        .withName("default teleop swerve");
  }

  private static Supplier<Optional<Rotation2d>> getTeleopSwerveAngleSupplier(
      SwerveDrivetrain drivetrain) {
    return () -> {
      if (OISelector.getOperatorInterface().getAutoSnapsEnabledTrigger().getAsBoolean()) {
        double currentRotationDeg = drivetrain.getPose().getRotation().getDegrees();

        // if the robot is near a wall, force the heading to a multiple of 90 degrees to facilitate
        // traversal
        double currentYPose = drivetrain.getPose().getY();

        // see which of the two walls we are closer to
        boolean closerToLeft = currentYPose < WALL_SNAP_TOLERANCE_METERS;
        boolean closerToRight =
            currentYPose > (FieldConstants.fieldWidth - WALL_SNAP_TOLERANCE_METERS);

        // Don't snap if we are in the trench zone since we want the driver to maintain control
        // while traversing the trench
        if ((closerToLeft || closerToRight) && !Field2d.getInstance().inTrenchZone()) {
          double nearest90DegreeAngle =
              Math.round(currentRotationDeg / 90.0) * 90.0; // nearest multiple of 90 degrees
          Rotation2d targetRotation = Rotation2d.fromDegrees(nearest90DegreeAngle);
          return Optional.of(targetRotation);
        }
      }

      return Optional.empty(); // no snap, use joystick rotation
    };
  }
}
