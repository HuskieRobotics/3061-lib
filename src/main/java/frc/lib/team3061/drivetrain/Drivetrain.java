// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team3061.drivetrain;

import static frc.lib.team3061.drivetrain.DrivetrainConstants.*;
import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.DrivetrainIO.SwerveIOInputs;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's drivetrain mechanism. It consists of a four MK4 swerve modules,
 * each with two motors and an encoder. It also consists of a Pigeon which is used to measure the
 * robot's rotation.
 */
public class Drivetrain extends SubsystemBase {

  private final DrivetrainIO io;
  private final DrivetrainIO.DrivetrainIOInputsCollection inputs =
      new DrivetrainIO.DrivetrainIOInputsCollection();

  private final TunableNumber autoDriveKp =
      new TunableNumber("AutoDrive/DriveKp", RobotConfig.getInstance().getAutoDriveKP());
  private final TunableNumber autoDriveKi =
      new TunableNumber("AutoDrive/DriveKi", RobotConfig.getInstance().getAutoDriveKI());
  private final TunableNumber autoDriveKd =
      new TunableNumber("AutoDrive/DriveKd", RobotConfig.getInstance().getAutoDriveKD());
  private final TunableNumber autoTurnKp =
      new TunableNumber("AutoDrive/TurnKp", RobotConfig.getInstance().getAutoTurnKP());
  private final TunableNumber autoTurnKi =
      new TunableNumber("AutoDrive/TurnKi", RobotConfig.getInstance().getAutoTurnKI());
  private final TunableNumber autoTurnKd =
      new TunableNumber("AutoDrive/TurnKd", RobotConfig.getInstance().getAutoTurnKD());

  private final TunableNumber driveCurrent = new TunableNumber("Drivetrain/driveCurrent", 0.0);
  private final TunableNumber steerCurrent = new TunableNumber("Drivetrain/steerCurrent", 0.0);

  private final PIDController autoXController =
      new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoYController =
      new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoThetaController =
      new PIDController(autoTurnKp.get(), autoTurnKi.get(), autoTurnKd.get());

  private boolean isFieldRelative;

  private boolean isTranslationSlowMode = false;
  private boolean isRotationSlowMode = false;

  private boolean brakeMode;
  private Timer brakeModeTimer = new Timer();
  private static final double BREAK_MODE_DELAY_SEC = 10.0;
  private static final double LEDS_FALLEN_ANGLE_DEGREES = 60.0; // Threshold to detect falls

  private DriveMode driveMode = DriveMode.NORMAL;

  private boolean isTurbo;

  private boolean isMoveToPoseEnabled;

  private Alert noPoseAlert =
      new Alert("Attempted to reset pose from vision, but no pose was found.", AlertType.WARNING);

  private ChassisSpeeds prevSpeeds = new ChassisSpeeds();
  private double[] prevSteerVelocitiesRevPerMin = new double[4];

  private DriverStation.Alliance alliance = DriverStation.Alliance.Blue;

  /**
   * Creates a new Drivetrain subsystem.
   *
   * @param io the abstracted interface for the drivetrain
   */
  public Drivetrain(DrivetrainIO io) {
    this.io = io;

    this.autoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.isFieldRelative = false;

    // based on testing we can drive in turbo mode all the time
    this.isTurbo = true;

    this.isMoveToPoseEnabled = true;

    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    tabMain
        .addNumber("Gyroscope Angle", () -> getRotation().getDegrees())
        .withPosition(9, 0)
        .withSize(1, 1);
    tabMain.addBoolean("X-Stance On?", this::isXstance).withPosition(7, 0).withSize(1, 1);
    tabMain
        .addBoolean("Field-Relative Enabled?", () -> this.isFieldRelative)
        .withPosition(8, 0)
        .withSize(1, 1);

    FaultReporter faultReporter = FaultReporter.getInstance();
    faultReporter.registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(
                RobotConfig.getInstance().getAutoDriveKP(),
                RobotConfig.getInstance().getAutoDriveKI(),
                RobotConfig.getInstance().getAutoDriveKD()), // Translation PID constants
            new PIDConstants(
                RobotConfig.getInstance().getAutoTurnKP(),
                RobotConfig.getInstance().getAutoTurnKI(),
                RobotConfig.getInstance().getAutoTurnKD()), // Rotation PID constants
            RobotConfig.getInstance().getAutoMaxSpeed(), // Max module speed, in m/s
            new Translation2d(
                    RobotConfig.getInstance().getWheelbase(),
                    RobotConfig.getInstance().getTrackwidth())
                .getNorm(), // Drive base radius in meters. Distance from robot center to furthest
            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        this::shouldFlipAutoPath,
        this // Reference to this subsystem to set requirements
        );
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return new ChassisSpeeds(
        this.inputs.drivetrain.measuredVXMetersPerSec,
        this.inputs.drivetrain.measuredVYMetersPerSec,
        this.inputs.drivetrain.measuredAngularVelocityRadPerSec);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    this.drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        false,
        false);
  }

  /** Enables "turbo" mode (i.e., acceleration is not limited by software) */
  public void enableTurbo() {
    this.isTurbo = true;
  }

  /** Disables "turbo" mode (i.e., acceleration is limited by software) */
  public void disableTurbo() {
    this.isTurbo = false;
  }

  /**
   * Returns true if the robot is in "turbo" mode (i.e., acceleration is not limited by software);
   * false otherwise
   *
   * @return true if the robot is in "turbo" mode (i.e., acceleration is not limited by software);
   *     false otherwise
   */
  public boolean getTurbo() {
    return this.isTurbo;
  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment between the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */
  public void zeroGyroscope() {
    setGyroOffset(0.0);
  }

  /**
   * Returns the rotation of the robot. Zero degrees is facing away from the driver station; CCW is
   * positive. If the gyro is not connected, the rotation from the estimated pose is returned.
   *
   * @return the rotation of the robot
   */
  public Rotation2d getRotation() {
    return this.inputs.drivetrain.rotation;
  }

  /**
   * Returns the yaw of the drivetrain as reported by the gyro in degrees. Usually, the getRotation
   * method should be invoked instead.
   *
   * @return the yaw of the drivetrain as reported by the gyro in degrees
   */
  public double getYaw() {
    return this.inputs.gyro.yawDeg;
  }

  /**
   * Returns the pitch of the drivetrain as reported by the gyro in degrees.
   *
   * @return the pitch of the drivetrain as reported by the gyro in degrees
   */
  public double getPitch() {
    return this.inputs.gyro.pitchDeg;
  }

  /**
   * Returns the roll of the drivetrain as reported by the gyro in degrees.
   *
   * @return the roll of the drivetrain as reported by the gyro in degrees
   */
  public double getRoll() {
    return this.inputs.gyro.rollDeg;
  }

  /**
   * Sets the rotation of the robot to the specified value. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
   * facing away from the driver station; CCW is positive.
   *
   * @param expectedYaw the rotation of the robot (in degrees)
   */
  public void setGyroOffset(double expectedYaw) {
    this.io.setGyroOffset(expectedYaw);
  }

  /**
   * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
   * rotation). The origin of the field to the lower left corner (i.e., the corner of the field to
   * the driver's right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @return the pose of the robot
   */
  public Pose2d getPose() {
    return this.inputs.drivetrain.robotPose;
  }

  /**
   * Sets the odometry of the robot to the specified PathPlanner state. This method should only be
   * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
   * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
   * right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param state the specified PathPlanner state to which is set the odometry
   */
  public void resetPose(Pose2d pose) {
    this.io.resetPose(pose);
  }

  /**
   * Sets the robot's odometry's rotation based on the gyro. This method is intended to be invoked
   * when the vision subsystem is first disabled and may have negatively impacted the pose
   * estimator.
   */
  public void resetPoseRotationToGyro() {
    this.io.resetPose(new Pose2d(this.getPose().getTranslation(), this.getRotation()));
  }

  /**
   * Sets the odometry of the robot based on the supplied pose (e.g., from the vision subsystem).
   * When testing, the robot can be positioned in front of an AprilTag and this method can be
   * invoked to reset the robot's pose based on tag.
   *
   * @param poseSupplier the supplier of the pose to which set the robot's odometry
   */
  public void resetPoseToVision(Supplier<Pose3d> poseSupplier) {
    Pose3d pose = poseSupplier.get();
    if (pose != null) {
      noPoseAlert.set(false);
      this.io.resetPose(pose.toPose2d());
    } else {
      noPoseAlert.set(true);
    }
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities may be specified from either the robot's frame of
   * reference of the field's frame of reference. In the robot's frame of reference, the positive x
   * direction is forward; the positive y direction, left; position rotation, CCW. In the field
   * frame of reference, the positive x direction is away from the driver and the positive y
   * direction is to the driver's left. This method accounts for the fact that the origin of the
   * field is always the corner to the right of the blue alliance driver station. A positive
   * rotational velocity always rotates the robot in the CCW direction.
   *
   * <p>If the translation or rotation slow mode features are enabled, the corresponding velocities
   * will be scaled to enable finer control.
   *
   * <p>If the drive mode is X, the robot will ignore the specified velocities and turn the swerve
   * modules into the x-stance orientation.
   *
   * <p>If the drive mode is SWERVE_DRIVE_CHARACTERIZATION or SWERVE_ROTATE_CHARACTERIZATION, the
   * robot will ignore the specified velocities and run the appropriate characterization routine.
   * Refer to the FeedForwardCharacterization command class for more information.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param rotationalVelocity the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   * @param overrideFieldRelative true to force field-relative motion; false to use the current
   *     setting
   */
  public void drive(
      double xVelocity,
      double yVelocity,
      double rotationalVelocity,
      boolean isOpenLoop,
      boolean isFieldRelative) {

    if (driveMode == DriveMode.NORMAL) {
      // get the slow-mode multiplier from the config
      double slowModeMultiplier = RobotConfig.getInstance().getRobotSlowModeMultiplier();

      // if translation or rotation is in slow mode, multiply the x and y velocities by the
      // slow-mode multiplier
      if (isTranslationSlowMode) {
        xVelocity *= slowModeMultiplier;
        yVelocity *= slowModeMultiplier;
      }

      // if rotation is in slow mode, multiply the rotational velocity by the slow-mode multiplier
      if (isRotationSlowMode) {
        rotationalVelocity *= slowModeMultiplier;
      }

      if (isFieldRelative) {
        // the origin of the field is always the corner to the right of the blue alliance driver
        // station. As a result, "forward" from a field-relative perspective when on the red
        // alliance, is in the negative x direction. Similarly, "left" from a field-relative
        // perspective when on the red alliance is in the negative y direction.
        int allianceMultiplier = this.alliance == Alliance.Blue ? 1 : -1;
        this.io.driveFieldRelative(
            allianceMultiplier * xVelocity,
            allianceMultiplier * yVelocity,
            rotationalVelocity,
            isOpenLoop);
      } else {
        this.io.driveRobotRelative(xVelocity, yVelocity, rotationalVelocity, isOpenLoop);
      }
    } else {
      this.io.holdXStance();
    }
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x and y
   * directions, while keeping the robot aligned to the specified target rotation. The velocities
   * must be specified from the field's frame of reference as field relative mode is assumed. In the
   * field frame of reference, the origin of the field to the lower left corner (i.e., the corner of
   * the field to the driver's right). Zero degrees is away from the driver and increases in the CCW
   * direction.
   *
   * <p>If the translation slow mode feature is enabled, the corresponding velocities will be scaled
   * to enable finer control.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param targetDirection the desired direction of the robot's orientation. Zero degrees is away
   *     from the driver and increases in the CCW direction.
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public void driveFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {

    // get the slow-mode multiplier from the config
    double slowModeMultiplier = RobotConfig.getInstance().getRobotSlowModeMultiplier();

    // if translation or rotation is in slow mode, multiply the x and y velocities by the
    // slow-mode multiplier
    if (isTranslationSlowMode) {
      xVelocity *= slowModeMultiplier;
      yVelocity *= slowModeMultiplier;
    }

    this.io.driveFieldRelativeFacingAngle(xVelocity, yVelocity, targetDirection, isOpenLoop);
  }

  /**
   * Stops the motion of the robot. Since the motors are in break mode, the robot will stop soon
   * after this method is invoked.
   */
  public void stop() {
    this.io.driveRobotRelative(0.0, 0.0, 0.0, false);
  }

  /**
   * This method is invoked each iteration of the scheduler. Typically, when using a command-based
   * model, subsystems don't override the periodic method. However, the drivetrain needs to
   * continually update the odometry of the robot, update and log the gyro and swerve module inputs,
   * update brake mode, and update the tunable values.
   */
  @Override
  public void periodic() {
    if (Constants.TUNING_MODE) {
      this.prevSpeeds =
          new ChassisSpeeds(
              this.inputs.drivetrain.measuredVXMetersPerSec,
              this.inputs.drivetrain.measuredVYMetersPerSec,
              this.inputs.drivetrain.measuredAngularVelocityRadPerSec);
      for (int i = 0; i < this.inputs.swerve.length; i++) {
        this.prevSteerVelocitiesRevPerMin[i] = this.inputs.swerve[i].steerVelocityRevPerMin;
      }
    }

    // when testing, set the drive motor current or the steer motor current based on the Tunables
    // (if non-zero)
    if (TESTING) {
      if (driveCurrent.get() != 0) {
        this.io.setDriveMotorCurrent(driveCurrent.get());
      }

      if (steerCurrent.get() != 0) {
        this.io.setSteerMotorCurrent(steerCurrent.get());
      }
    }

    this.io.updateInputs(this.inputs);
    Logger.processInputs(SUBSYSTEM_NAME, this.inputs.drivetrain);
    Logger.processInputs(SUBSYSTEM_NAME + "/Gyro", this.inputs.gyro);
    Logger.processInputs(SUBSYSTEM_NAME + "/FL", this.inputs.swerve[0]);
    Logger.processInputs(SUBSYSTEM_NAME + "/FR", this.inputs.swerve[1]);
    Logger.processInputs(SUBSYSTEM_NAME + "/BL", this.inputs.swerve[2]);
    Logger.processInputs(SUBSYSTEM_NAME + "/BR", this.inputs.swerve[3]);

    // Check for fallen robot
    LEDs.getInstance()
        .setFallen(
            Math.abs(this.inputs.gyro.pitchDeg) > LEDS_FALLEN_ANGLE_DEGREES
                || Math.abs(this.inputs.gyro.rollDeg) > LEDS_FALLEN_ANGLE_DEGREES);

    // update the brake mode based on the robot's velocity and state (enabled/disabled)
    updateBrakeMode();

    // update tunables
    if (autoDriveKp.hasChanged() || autoDriveKi.hasChanged() || autoDriveKd.hasChanged()) {
      autoXController.setPID(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
      autoYController.setPID(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
    }
    if (autoTurnKp.hasChanged() || autoTurnKi.hasChanged() || autoTurnKd.hasChanged()) {
      autoThetaController.setPID(autoTurnKp.get(), autoTurnKi.get(), autoTurnKd.get());
    }
  }

  /**
   * Returns true if field relative mode is enabled
   *
   * @return true if field relative mode is enabled
   */
  public boolean getFieldRelative() {
    return isFieldRelative;
  }

  /**
   * Enables field-relative mode. When enabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the field.
   */
  public void enableFieldRelative() {
    this.isFieldRelative = true;
  }

  /**
   * Disables field-relative mode. When disabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the robot.
   */
  public void disableFieldRelative() {
    this.isFieldRelative = false;
  }

  /**
   * Enables slow mode for translation. When enabled, the robot's translational velocities will be
   * scaled down.
   */
  public void enableTranslationSlowMode() {
    this.isTranslationSlowMode = true;
  }

  /**
   * Disables slow mode for translation. When disabled, the robot's translational velocities will
   * not be scaled.
   */
  public void disableTranslationSlowMode() {
    this.isTranslationSlowMode = false;
  }

  /**
   * Enables slow mode for rotation. When enabled, the robot's rotational velocity will be scaled
   * down.
   */
  public void enableRotationSlowMode() {
    this.isRotationSlowMode = true;
  }

  /**
   * Disables slow mode for rotation. When disabled, the robot's rotational velocity will not be
   * scaled.
   */
  public void disableRotationSlowMode() {
    this.isRotationSlowMode = false;
  }

  /**
   * Puts the drivetrain into the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This prevents the robot from rolling on an inclined surface and makes it more
   * difficult for other robots to push the robot, which is useful when shooting. The robot cannot
   * be driven until x-stance is disabled.
   */
  public void enableXstance() {
    this.driveMode = DriveMode.X;
    this.io.holdXStance();
  }

  /** Disables x-stance, allowing the robot to be driven. */
  public void disableXstance() {
    this.driveMode = DriveMode.NORMAL;
  }

  /**
   * Returns true if the robot is in the x-stance orientation.
   *
   * @return true if the robot is in the x-stance orientation
   */
  public boolean isXstance() {
    return this.driveMode == DriveMode.X;
  }

  /**
   * Sets the robot's center of rotation. The origin is at the center of the robot. The positive x
   * direction is forward; the positive y direction, left.
   *
   * @param x the x coordinate of the robot's center of rotation (in meters)
   * @param y the y coordinate of the robot's center of rotation (in meters)
   */
  public void setCenterOfRotation(double x, double y) {
    io.setCenterOfRotation(new Translation2d(x, y));
  }

  /** Resets the robot's center of rotation to the center of the robot. */
  public void resetCenterOfRotation() {
    setCenterOfRotation(0.0, 0.0);
  }

  /**
   * Returns the desired velocity of the drivetrain in the x direction (units of m/s)
   *
   * @return the desired velocity of the drivetrain in the x direction (units of m/s)
   */
  public double getVelocityX() {
    return this.inputs.drivetrain.measuredVXMetersPerSec;
  }

  /**
   * Returns the desired velocity of the drivetrain in the y direction (units of m/s)
   *
   * @return the desired velocity of the drivetrain in the y direction (units of m/s)
   */
  public double getVelocityY() {
    return this.inputs.drivetrain.measuredVYMetersPerSec;
  }

  /**
   * Returns the average current of the swerve module drive motors in amps.
   *
   * @return the average current of the swerve module drive motors in amps
   */
  public double getAverageDriveCurrent() {
    return this.inputs.drivetrain.averageDriveCurrent;
  }

  /**
   * Returns the PID controller used to control the robot's x position during autonomous.
   *
   * @return the PID controller used to control the robot's x position during autonomous
   */
  public PIDController getAutoXController() {
    return this.autoXController;
  }

  /**
   * Returns the PID controller used to control the robot's y position during autonomous.
   *
   * @return the PID controller used to control the robot's y position during autonomous
   */
  public PIDController getAutoYController() {
    return this.autoYController;
  }

  /**
   * Returns the PID controller used to control the robot's rotation during autonomous.
   *
   * @return the PID controller used to control the robot's rotation during autonomous
   */
  public PIDController getAutoThetaController() {
    return this.autoThetaController;
  }

  /**
   * Runs forwards at the commanded voltage.
   *
   * @param volts the commanded voltage
   */
  public void runDriveCharacterizationVolts(double volts) {
    this.io.setDriveMotorVoltage(volts);
  }

  /**
   * Returns the average drive velocity in meters/sec.
   *
   * @return the average drive velocity in meters/sec
   */
  public double getDriveCharacterizationVelocity() {
    return Math.sqrt(
        Math.pow(this.inputs.drivetrain.measuredVXMetersPerSec, 2)
            + Math.pow(this.inputs.drivetrain.measuredVYMetersPerSec, 2));
  }

  /**
   * Returns the average drive velocity in meters/sec.
   *
   * @return the average drive velocity in meters/sec
   */
  public double getDriveCharacterizationAcceleration() {
    return Math.sqrt(
            Math.pow(
                    (this.inputs.drivetrain.measuredVXMetersPerSec
                        - this.prevSpeeds.vxMetersPerSecond),
                    2)
                + Math.pow(
                    (this.inputs.drivetrain.measuredVYMetersPerSec
                        - this.prevSpeeds.vyMetersPerSecond),
                    2))
        / LOOP_PERIOD_SECS;
  }

  /**
   * Rotates swerve modules at the commanded voltage.
   *
   * @param volts the commanded voltage
   */
  public void runRotateCharacterizationVolts(double volts) {
    this.io.setSteerMotorVoltage(volts);
  }

  /**
   * Returns the average rotational velocity in radians/sec.
   *
   * @return the average rotational velocity in radians/sec
   */
  public double getRotateCharacterizationVelocity() {
    double avgVelocity = 0.0;
    for (SwerveIOInputs swerveInputs : this.inputs.swerve) {
      avgVelocity += swerveInputs.steerVelocityRevPerMin;
    }
    avgVelocity /= this.inputs.swerve.length;
    avgVelocity *= (2.0 * Math.PI) / 60.0;
    return avgVelocity;
  }

  /**
   * Returns the average rotational acceleration in radians/sec^2.
   *
   * @return the average rotational acceleration in radians/sec^2
   */
  public double getRotateCharacterizationAcceleration() {
    double avgAcceleration = 0.0;
    for (int i = 0; i < this.inputs.swerve.length; i++) {
      avgAcceleration +=
          Math.abs(
              this.inputs.swerve[i].steerVelocityRevPerMin - this.prevSteerVelocitiesRevPerMin[i]);
    }
    avgAcceleration /= this.inputs.swerve.length;
    avgAcceleration *= (2.0 * Math.PI) / 60.0;
    return avgAcceleration / LOOP_PERIOD_SECS;
  }

  /**
   * Enables or disables the move-to-pose feature. Refer to the MoveToPose command class for more
   * information.
   *
   * @param state true to enable, false to disable
   */
  public void enableMoveToPose(boolean state) {
    this.isMoveToPoseEnabled = state;
  }

  /**
   * Returns true if the move-to-pose feature is enabled; false otherwise.
   *
   * @return true if the move-to-pose feature is enabled; false otherwise
   */
  public boolean isMoveToPoseEnabled() {
    return this.isMoveToPoseEnabled;
  }

  /**
   * This method should be invoked once the alliance color is known. Refer to the RobotContainer's
   * checkAllianceColor method for best practices on when to check the alliance's color. The
   * alliance color is needed when running auto paths as those paths are always defined for
   * blue-alliance robots and need to be flipped for red-alliance robots.
   *
   * @param newAlliance the new alliance color
   */
  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
  }

  /**
   * Returns true if the auto path, which is always defined for a blue alliance robot, should be
   * flipped to the red alliance side of the field.
   *
   * @return true if the auto path should be flipped to the red alliance side of the field
   */
  public boolean shouldFlipAutoPath() {
    return this.alliance == Alliance.Red;
  }

  private Command getSystemCheckCommand() {
    return Commands.sequence(Commands.sequence(Commands.waitSeconds(0.25)))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(Commands.runOnce(() -> drive(0, 0, 0, true, false), this));
  }

  /**
   * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
   * stopped moving for the specified period of time, and brake mode is enabled, disable it.
   */
  private void updateBrakeMode() {
    if (DriverStation.isEnabled() && !brakeMode) {
      brakeMode = true;
      setBrakeMode(true);
      brakeModeTimer.restart();

    } else if (!DriverStation.isEnabled()) {
      boolean stillMoving = false;
      double velocityLimit = RobotConfig.getInstance().getRobotMaxCoastVelocity();
      if (Math.abs(this.inputs.drivetrain.measuredVXMetersPerSec) > velocityLimit
          || Math.abs(this.inputs.drivetrain.measuredVYMetersPerSec) > velocityLimit) {
        stillMoving = true;
        brakeModeTimer.restart();
      }

      if (brakeMode && !stillMoving && brakeModeTimer.hasElapsed(BREAK_MODE_DELAY_SEC)) {
        brakeMode = false;
        setBrakeMode(false);
      }
    }
  }

  private void setBrakeMode(boolean enable) {
    this.io.setBrakeMode(enable);
  }

  private enum DriveMode {
    NORMAL,
    X
  }
}
