package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.lib.team3061.vision.Vision;
import frc.robot.operator_interface.OperatorInterface;

public class SwerveDrivetrainCommandFactory {

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
    swerveDrivetrain.setDefaultCommand(
        new TeleopSwerve(swerveDrivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    driveToPoseCanceledTrigger = new Trigger(swerveDrivetrain::getDriveToPoseCanceled);
    driveToPoseCanceledTrigger.onTrue(
        Commands.sequence(
                Commands.run(
                        () -> LEDs.getInstance().requestState(LEDs.States.DRIVE_TO_POSE_CANCELED),
                        swerveDrivetrain)
                    .withTimeout(0.5),
                Commands.runOnce(() -> swerveDrivetrain.setDriveToPoseCanceled(false)))
            .withName("cancel drive to pose"));

    // lock rotation to the nearest 180° while driving
    oi.getLock180Button()
        .whileTrue(
            new TeleopSwerve(
                    swerveDrivetrain,
                    oi::getTranslateX,
                    oi::getTranslateY,
                    () ->
                        (swerveDrivetrain.getPose().getRotation().getDegrees() > -90
                                && swerveDrivetrain.getPose().getRotation().getDegrees() < 90)
                            ? Rotation2d.fromDegrees(0.0)
                            : Rotation2d.fromDegrees(180.0))
                .withName("lock 180"));

    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                    Commands.runOnce(swerveDrivetrain::disableFieldRelative, swerveDrivetrain),
                    Commands.runOnce(swerveDrivetrain::enableFieldRelative, swerveDrivetrain),
                    swerveDrivetrain::getFieldRelative)
                .withName("toggle field relative"));

    // slow-mode toggle
    oi.getTranslationSlowModeButton()
        .onTrue(
            Commands.runOnce(swerveDrivetrain::enableTranslationSlowMode, swerveDrivetrain)
                .withName("enable translation slow mode"));
    oi.getTranslationSlowModeButton()
        .onFalse(
            Commands.runOnce(swerveDrivetrain::disableTranslationSlowMode, swerveDrivetrain)
                .withName("disable translation slow mode"));
    oi.getRotationSlowModeButton()
        .onTrue(
            Commands.runOnce(swerveDrivetrain::enableRotationSlowMode, swerveDrivetrain)
                .withName("enable rotation slow mode"));
    oi.getRotationSlowModeButton()
        .onFalse(
            Commands.runOnce(swerveDrivetrain::disableRotationSlowMode, swerveDrivetrain)
                .withName("disable rotation slow mode"));

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
  }
}
