// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team3061.differential_drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.lib.team3061.differential_drivetrain.DifferentialDrivetrainConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.DifferentialRobotOdometry;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.LoggedTracer;
import frc.robot.Field2d;
import org.littletonrobotics.junction.Logger;

public class DifferentialDrivetrain extends SubsystemBase {
  private final DifferentialDrivetrainIO io;
  private final DifferentialDrivetrainIOInputsAutoLogged inputs =
      new DifferentialDrivetrainIOInputsAutoLogged();

  private final DifferentialRobotOdometry odometry;

  /** Creates a new Drivetrain. */
  public DifferentialDrivetrain(DifferentialDrivetrainIO io) {
    this.io = io;

    this.odometry = new DifferentialRobotOdometry();
    RobotOdometry.setInstance(this.odometry);

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::applyRobotSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new PPLTVController(0.02),
        RobotConfig.getInstance().getPathPlannerRobotConfig(),
        this::shouldFlipAutoPath,
        this // Reference to this subsystem to set requirements
        );
  }

  public void arcadeDrive(LinearVelocity xVelocity, AngularVelocity rotationalVelocity) {

    io.driveRobotRelative(xVelocity, rotationalVelocity, true);
  }

  /**
   * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
   * rotation). The origin of the field is always the blue origin (i.e., the positive x-axis points
   * away from the blue alliance wall). Zero degrees is aligned to the positive x axis and increases
   * in the CCW direction.
   *
   * @return the pose of the robot
   */
  private Pose2d getPose() {
    return this.odometry.getEstimatedPose();
  }

  /**
   * Sets the odometry of the robot to the specified pose. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). The origin of
   * the field is always the blue origin (i.e., the positive x-axis points away from the blue
   * alliance wall). Zero degrees is aligned to the positive x axis and increases in the CCW
   * direction.
   *
   * @param pose the specified pose to which is set the odometry
   */
  public void resetPose(Pose2d pose) {
    this.odometry.resetPose(
        Rotation2d.fromRadians(this.inputs.heading.in(Radians)),
        this.inputs.leftPosition.in(Meters),
        this.inputs.rightPosition.in(Meters),
        pose);
  }

  /**
   * Returns the robot-relative speeds of the robot.
   *
   * @return the robot-relative speeds of the robot
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {

    return RobotConfig.getInstance()
        .getDifferentialDriveKinematics()
        .toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(this.inputs.leftVelocity, this.inputs.rightVelocity));
  }
  /**
   * Applies the specified robot-relative speeds to the drivetrain along with the specified feed
   * forward forces.
   *
   * @param chassisSpeeds the robot-relative speeds of the robot
   * @param feedforwards the feed forward forces to apply
   */
  private void applyRobotSpeeds(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
    this.io.applyRobotSpeeds(
        chassisSpeeds,
        feedforwards.robotRelativeForcesX(),
        feedforwards.robotRelativeForcesY(),
        false);
  }

  /**
   * Returns true if the auto path, which is always defined for a blue alliance robot, should be
   * flipped to the red alliance side of the field.
   *
   * @return true if the auto path should be flipped to the red alliance side of the field
   */
  private boolean shouldFlipAutoPath() {
    return Field2d.getInstance().getAlliance() == Alliance.Red;
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    Logger.processInputs(SUBSYSTEM_NAME, this.inputs);

    // update odometry
    this.odometry.updateWithTime(
        Timer.getFPGATimestamp(),
        Rotation2d.fromRadians(inputs.heading.in(Radians)),
        inputs.leftPosition.in(Meters),
        inputs.rightPosition.in(Meters));

    Pose2d pose = this.odometry.getEstimatedPose();
    Logger.recordOutput(SUBSYSTEM_NAME + "/Pose", pose);

    // Record cycle time
    LoggedTracer.record("Drivetrain");
  }
}
