// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team3061.swerve_drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainConstants.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainConstants.SysIDCharacterizationMode;
import frc.lib.team3061.util.CustomPoseEstimator;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team3061.util.SwerveRobotOdometry;
import frc.lib.team3061.util.SysIdRoutineChooser;
import frc.lib.team6328.util.FieldConstants;
import frc.lib.team6328.util.LoggedTracer;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.Field2d;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's drivetrain mechanism. It consists of a four swerve modules in
 * the MK4 family, each with two TalonFX motors and a CANcoder. It also consists of a Pigeon which
 * is used to measure the robot's rotation. While other hardware configurations are possible, and
 * 3061-lib supports hardware abstraction via the standard AdvantageKit architecture, 3061-lib only
 * supports CTRE devices at this time.
 */
public class SwerveDrivetrain extends SubsystemBase implements CustomPoseEstimator {

  private final SwerveDrivetrainIO io;
  private final SwerveDrivetrainIO.SwerveDrivetrainIOInputsCollection inputs =
      new SwerveDrivetrainIO.SwerveDrivetrainIOInputsCollection();

  /*
   * If TUNING is set to true in Constants.java, the following tunables will be available in
   * AdvantageScope. This enables efficient tuning of PID coefficients without restarting the code.
   */
  private final LoggedTunableNumber autoDriveKp =
      new LoggedTunableNumber("AutoDrive/DriveKp", RobotConfig.getInstance().getAutoDriveKP());
  private final LoggedTunableNumber autoDriveKi =
      new LoggedTunableNumber("AutoDrive/DriveKi", RobotConfig.getInstance().getAutoDriveKI());
  private final LoggedTunableNumber autoDriveKd =
      new LoggedTunableNumber("AutoDrive/DriveKd", RobotConfig.getInstance().getAutoDriveKD());
  private final LoggedTunableNumber autoTurnKp =
      new LoggedTunableNumber("AutoDrive/TurnKp", RobotConfig.getInstance().getAutoTurnKP());
  private final LoggedTunableNumber autoTurnKi =
      new LoggedTunableNumber("AutoDrive/TurnKi", RobotConfig.getInstance().getAutoTurnKI());
  private final LoggedTunableNumber autoTurnKd =
      new LoggedTunableNumber("AutoDrive/TurnKd", RobotConfig.getInstance().getAutoTurnKD());

  private final PIDController autoXController =
      new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoYController =
      new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoThetaController =
      new PIDController(autoTurnKp.get(), autoTurnKi.get(), autoTurnKd.get());

  private boolean isFieldRelative = true;
  private boolean isTranslationSlowMode = false;
  private boolean isRotationSlowMode = false;

  // set to true upon construction to trigger disabling break mode shortly after the code starts
  private boolean brakeMode = true;
  private Timer brakeModeTimer = new Timer();
  private static final double BREAK_MODE_DELAY_SEC = 10.0;

  private SlewRateLimiter xFilter =
      new SlewRateLimiter(
          RobotConfig.getInstance()
              .getRobotMaxAccelerationWhenLimited()
              .in(MetersPerSecondPerSecond));
  private SlewRateLimiter yFilter =
      new SlewRateLimiter(
          RobotConfig.getInstance()
              .getRobotMaxAccelerationWhenLimited()
              .in(MetersPerSecondPerSecond));
  private SlewRateLimiter thetaFilter =
      new SlewRateLimiter(
          RobotConfig.getInstance()
              .getRobotMaxAngularAccelerationWhenLimited()
              .in(RadiansPerSecondPerSecond));

  private boolean accelerationLimiting = false;
  private boolean driveToPoseCanceled = false;

  private Alert noPoseAlert =
      new Alert("Attempted to reset pose from vision, but no pose was found.", AlertType.kWarning);
  private Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);

  private final SwerveRobotOdometry odometry;
  private int constrainPoseToFieldCount = 0;

  private Pose2d customPose = new Pose2d();

  private double[] initialDistance = {0.0, 0.0, 0.0, 0.0};

  private boolean isRotationOverrideEnabled = false;

  private SwerveModulePosition[] modulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /**
   * SysId routine for characterizing translation. This is used to find FF/PID gains for the drive
   * motors. This should be used when the drive motors are using voltage control.
   */
  private final SysIdRoutine sysIdRoutineTranslationVolts =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationVolts_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output ->
                  applySysIdCharacterization(
                      SysIDCharacterizationMode.TRANSLATION_VOLTS, output.in(Volts)),
              null,
              this));

  /**
   * SysId routine for characterizing translation. This is used to find FF/PID gains for the drive
   * motors. This should be used when the drive motors are using TorqueCurrentFOC control.
   */
  private final SysIdRoutine sysIdRoutineTranslationCurrent =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(10).per(Second), // Use ramp rate of 5 A/s
              Volts.of(20), // Use dynamic step of 10 A
              Seconds.of(5), // Use timeout of 5 seconds
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output ->
                  applySysIdCharacterization(
                      SysIDCharacterizationMode.TRANSLATION_CURRENT,
                      output.in(Volts)), // treat volts as amps
              null,
              this));

  /**
   * SysId routine for characterizing steer. This is used to find FF/PID gains for the steer motors.
   * This should be used when the steer motors are using voltage control.
   */
  private final SysIdRoutine sysIdRoutineSteerVolts =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteerVolts_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts ->
                  applySysIdCharacterization(
                      SysIDCharacterizationMode.STEER_VOLTS, volts.in(Volts)),
              null,
              this));

  /**
   * SysId routine for characterizing steer. This is used to find FF/PID gains for the steer motors.
   * This should be used when the steer motors are using TorqueCurrentFOC control.
   */
  private final SysIdRoutine sysIdRoutineSteerCurrent =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(10).per(Second), // Use ramp rate of 5 A/s
              Volts.of(20), // Use dynamic step of 10 A
              Seconds.of(5), // Use timeout of 5 seconds
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output ->
                  applySysIdCharacterization(
                      SysIDCharacterizationMode.STEER_CURRENT,
                      output.in(Volts)), // treat volts as amps
              null,
              this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                applySysIdCharacterization(
                    SysIDCharacterizationMode.ROTATION_VOLTS, output.in(Volts));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /**
   * Creates a new Drivetrain subsystem.
   *
   * @param io the abstracted interface for the drivetrain
   */
  public SwerveDrivetrain(SwerveDrivetrainIO io) {
    this.io = io;

    this.autoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    FaultReporter faultReporter = FaultReporter.getInstance();
    faultReporter.registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::applyRobotSpeeds, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new com.pathplanner.lib.config.PIDConstants(
                RobotConfig.getInstance().getAutoDriveKP(),
                RobotConfig.getInstance().getAutoDriveKI(),
                RobotConfig.getInstance().getAutoDriveKD()), // Translation PID constants
            new PIDConstants(
                RobotConfig.getInstance().getAutoTurnKP(),
                RobotConfig.getInstance().getAutoTurnKI(),
                RobotConfig.getInstance().getAutoTurnKD())), // Rotation PID constants
        RobotConfig.getInstance().getPathPlannerRobotConfig(),
        this::shouldFlipAutoPath,
        this // Reference to this subsystem to set requirements
        );

    this.odometry = new SwerveRobotOdometry();
    RobotOdometry.setInstance(this.odometry);
    this.odometry.setCustomEstimator(this);

    this.xFilter.reset(0.0);
    this.yFilter.reset(0.0);
    this.thetaFilter.reset(0.0);

    SysIdRoutineChooser.getInstance().addOption("Translation Volts", sysIdRoutineTranslationVolts);

    SysIdRoutineChooser.getInstance()
        .addOption("Translation Current", sysIdRoutineTranslationCurrent);

    SysIdRoutineChooser.getInstance().addOption("Steer Volts", sysIdRoutineSteerVolts);

    SysIdRoutineChooser.getInstance().addOption("Steer Current", sysIdRoutineSteerCurrent);

    SysIdRoutineChooser.getInstance().addOption("Rotation Volts", sysIdRoutineRotation);
  }

  /**
   * Returns the robot-relative speeds of the robot.
   *
   * @return the robot-relative speeds of the robot
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return this.inputs.drivetrain.measuredChassisSpeeds;
  }

  /**
   * Applies the specified robot-relative speeds to the drivetrain along with the specified feed
   * forward forces.
   *
   * @param chassisSpeeds the robot-relative speeds of the robot
   * @param feedforwards the feed forward forces to apply
   */
  public void applyRobotSpeeds(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {

    // always calculate whenever we are driving so that we maintain a history of recent values
    this.xFilter.calculate(chassisSpeeds.vxMetersPerSecond);
    this.yFilter.calculate(chassisSpeeds.vyMetersPerSecond);
    this.thetaFilter.calculate(chassisSpeeds.omegaRadiansPerSecond);

    this.io.applyRobotSpeeds(
        chassisSpeeds,
        feedforwards.robotRelativeForcesX(),
        feedforwards.robotRelativeForcesY(),
        false);
  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment between the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */
  public void zeroGyroscope() {
    setGyroOffset(Radians.of(0.0));
  }

  /**
   * Returns the rotation of the robot. Zero degrees is facing away from the driver station; CCW is
   * positive.
   *
   * @return the rotation of the robot
   */
  public Rotation2d getRotation() {
    return this.odometry.getEstimatedPose().getRotation();
  }

  /**
   * Returns the raw heading of the drivetrain as reported by the gyro in degrees. Usually, the
   * getRotation method should be invoked instead.
   *
   * @return the raw heading of the drivetrain as reported by the gyro in degrees
   */
  public Angle getYaw() {
    return this.inputs.drivetrain.rawHeading;
  }

  /**
   * Sets the rotation of the robot to the specified value. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). Usually, the
   * resetPose method is used instead. Zero degrees is facing away from the driver station; CCW is
   * positive.
   *
   * @param expectedYaw the rotation of the robot (in degrees)
   */
  public void setGyroOffset(Angle expectedYaw) {
    this.resetPose(
        new Pose2d(
            this.odometry.getEstimatedPose().getTranslation(),
            Rotation2d.fromRadians(expectedYaw.in(Radians))));
  }

  /**
   * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
   * rotation). The origin of the field is always the blue origin (i.e., the positive x-axis points
   * away from the blue alliance wall). Zero degrees is aligned to the positive x axis and increases
   * in the CCW direction.
   *
   * @return the pose of the robot
   */
  public Pose2d getPose() {
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
        Rotation2d.fromRadians(this.inputs.drivetrain.rawHeading.in(Radians)),
        this.modulePositions,
        pose);
  }

  /**
   * Sets the odometry of the robot based on the supplied pose (e.g., from the vision subsystem).
   * The robot can be positioned in front of an AprilTag and this method can be invoked to reset the
   * robot's pose based on tag.
   *
   * @param poseSupplier the supplier of the pose to which set the robot's odometry
   */
  public void resetPoseToVision(Supplier<Pose3d> poseSupplier) {
    Pose3d pose = poseSupplier.get();
    if (pose != null) {
      noPoseAlert.set(false);
      this.resetPose(pose.toPose2d());
    } else {
      noPoseAlert.set(true);
    }
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities may be specified from either the robot's frame of
   * reference or the field's frame of reference. In the robot's frame of reference, the positive x
   * direction is forward; the positive y direction, left; position rotation, CCW. In the field
   * frame of reference, the positive x direction is away from the blue alliance wall and the
   * positive y direction is to a blue alliance driver's left. This method accounts for the fact
   * that the origin of the field is always the corner to the right of the blue alliance driver
   * station. A positive rotational velocity always rotates the robot in the CCW direction.
   *
   * <p>If the translation or rotation slow mode features are enabled, the corresponding velocities
   * will be scaled to enable finer control.
   *
   * <p>If the drive mode is X, the robot will ignore the specified velocities and turn the swerve
   * modules into the x-stance orientation.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param rotationalVelocity the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   * @param isFieldRelative true to for field-relative motion; false, for robot-relative
   */
  public void drive(
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rotationalVelocity,
      boolean isOpenLoop,
      boolean isFieldRelative) {

    // get the slow-mode multiplier from the config
    double slowModeMultiplier = RobotConfig.getInstance().getRobotSlowModeMultiplier();

    // log velocity before and after filter
    Logger.recordOutput(SUBSYSTEM_NAME + "/requestedXVelocity", xVelocity);
    Logger.recordOutput(SUBSYSTEM_NAME + "/requestedYVelocity", yVelocity);
    Logger.recordOutput(SUBSYSTEM_NAME + "/requestedRotationalVelocity", rotationalVelocity);

    // always calculate whenever we are driving so that we maintain a history of recent values
    this.xFilter.calculate(xVelocity.in(MetersPerSecond));
    this.yFilter.calculate(yVelocity.in(MetersPerSecond));
    this.thetaFilter.calculate(rotationalVelocity.in(RadiansPerSecond));

    // log velocity after filter
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/filteredXVelocity", this.xFilter.lastValue(), MetersPerSecond);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/filteredYVelocity", this.yFilter.lastValue(), MetersPerSecond);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/filteredRotationalVelocity",
        this.thetaFilter.lastValue(),
        RadiansPerSecond);

    if (accelerationLimiting) {
      xVelocity = MetersPerSecond.of(this.xFilter.lastValue());
      yVelocity = MetersPerSecond.of(this.yFilter.lastValue());
      rotationalVelocity = RadiansPerSecond.of(this.thetaFilter.lastValue());
    }

    // if translation or rotation is in slow mode, multiply the x and y velocities by the
    // slow-mode multiplier
    if (isTranslationSlowMode) {
      xVelocity = xVelocity.times(slowModeMultiplier);
      yVelocity = yVelocity.times(slowModeMultiplier);
    }

    if (Constants.DEMO_MODE) {
      LinearVelocity velocity =
          MetersPerSecond.of(
              Math.sqrt(
                  Math.pow(xVelocity.in(MetersPerSecond), 2)
                      + Math.pow(yVelocity.in(MetersPerSecond), 2)));
      if (velocity.gt(DEMO_MODE_MAX_VELOCITY)) {
        double scale = DEMO_MODE_MAX_VELOCITY.div(velocity).magnitude();
        xVelocity = xVelocity.times(scale);
        yVelocity = yVelocity.times(scale);
      }
    }

    // if rotation is in slow mode, multiply the rotational velocity by the slow-mode multiplier
    if (isRotationSlowMode) {
      rotationalVelocity = rotationalVelocity.times(slowModeMultiplier);
    }

    if (isFieldRelative) {
      // the origin of the field is always the corner to the right of the blue alliance driver
      // station. As a result, "forward" from a field-relative perspective when on the red
      // alliance, is in the negative x direction. Similarly, "left" from a field-relative
      // perspective when on the red alliance is in the negative y direction.
      if (Field2d.getInstance().getAlliance() != Alliance.Blue) {
        xVelocity = xVelocity.unaryMinus();
        yVelocity = yVelocity.unaryMinus();
      }
      this.io.driveFieldRelative(xVelocity, yVelocity, rotationalVelocity, isOpenLoop);
    } else {
      this.io.driveRobotRelative(xVelocity, yVelocity, rotationalVelocity, isOpenLoop);
    }
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x and y
   * directions, while keeping the robot aligned to the specified target rotation. The velocities
   * must be specified from the field's frame of reference as field relative mode is assumed. In the
   * field frame of reference, the origin of the field is always the blue origin (i.e., the positive
   * x-axis points away from the blue alliance wall). Zero degrees is aligned to the positive x axis
   * and increases in the CCW direction.
   *
   * <p>If the translation slow mode feature is enabled, the corresponding velocities will be scaled
   * to enable finer control.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param targetDirection the desired direction of the robot's orientation. Zero degrees is
   *     aligned to the positive x axis and increases in the CCW direction.
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public void driveFacingAngle(
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      Rotation2d targetDirection,
      boolean isOpenLoop) {

    // always calculate whenever we are driving so that we maintain a history of recent values
    this.xFilter.calculate(xVelocity.in(MetersPerSecond));
    this.yFilter.calculate(yVelocity.in(MetersPerSecond));
    this.thetaFilter.calculate(0.0);

    // get the slow-mode multiplier from the config
    double slowModeMultiplier = RobotConfig.getInstance().getRobotSlowModeMultiplier();

    // if translation or rotation is in slow mode, multiply the x and y velocities by the
    // slow-mode multiplier
    if (isTranslationSlowMode) {
      xVelocity = xVelocity.times(slowModeMultiplier);
      yVelocity = yVelocity.times(slowModeMultiplier);
    }

    if (Constants.DEMO_MODE) {
      LinearVelocity velocity =
          MetersPerSecond.of(
              Math.sqrt(
                  Math.pow(xVelocity.in(MetersPerSecond), 2)
                      + Math.pow(yVelocity.in(MetersPerSecond), 2)));
      if (velocity.gt(DEMO_MODE_MAX_VELOCITY)) {
        double scale = DEMO_MODE_MAX_VELOCITY.div(velocity).magnitude();
        xVelocity = xVelocity.times(scale);
        yVelocity = yVelocity.times(scale);
      }
    }

    if (Field2d.getInstance().getAlliance() != Alliance.Blue) {
      xVelocity = xVelocity.unaryMinus();
      yVelocity = yVelocity.unaryMinus();
    }
    this.io.driveFieldRelativeFacingAngle(xVelocity, yVelocity, targetDirection, isOpenLoop);

    Logger.recordOutput(SUBSYSTEM_NAME + "/DriveFacingAngle/targetDirection", targetDirection);
  }

  /**
   * Stops the motion of the robot. Since the motors are in brake mode, the robot will stop soon
   * after this method is invoked.
   */
  public void stop() {
    this.io.driveRobotRelative(
        MetersPerSecond.of(0.0), MetersPerSecond.of(0.0), RadiansPerSecond.of(0.0), false);
  }

  /**
   * Puts the drivetrain into the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This prevents the robot from rolling on an inclined surface and makes it more
   * difficult for other robots to push the robot, which is useful when shooting.
   */
  public void holdXstance() {
    this.io.holdXStance();
  }

  /**
   * This method is invoked each iteration of the scheduler. The primarily responsibility is to
   * update the drivetrain inputs from the hardware-specific layer. In addition, the odometry of the
   * robot, brake mode, and controllers are updated.
   */
  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    Logger.processInputs(SUBSYSTEM_NAME, this.inputs.drivetrain);
    Logger.processInputs(SUBSYSTEM_NAME + "/FL", this.inputs.swerve[0]);
    Logger.processInputs(SUBSYSTEM_NAME + "/FR", this.inputs.swerve[1]);
    Logger.processInputs(SUBSYSTEM_NAME + "/BL", this.inputs.swerve[2]);
    Logger.processInputs(SUBSYSTEM_NAME + "/BR", this.inputs.swerve[3]);

    // update odometry
    for (int i = 0; i < inputs.drivetrain.odometryTimestamps.length; i++) {
      for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
        this.modulePositions[moduleIndex].distanceMeters =
            inputs.swerve[moduleIndex].odometryDrivePositionsMeters[i];
        this.modulePositions[moduleIndex].angle =
            inputs.swerve[moduleIndex].odometryTurnPositions[i];
      }

      this.odometry.updateWithTime(
          inputs.drivetrain.odometryTimestamps[i],
          inputs.drivetrain.odometryYawPositions[i],
          modulePositions);
    }

    // custom pose vs default pose
    Pose2d pose = this.odometry.getEstimatedPose();
    this.customPose = this.inputs.drivetrain.customPose;
    Logger.recordOutput(SUBSYSTEM_NAME + "/Pose", pose);
    Logger.recordOutput(SUBSYSTEM_NAME + "/CustomPose", this.customPose);

    Logger.recordOutput(
        SUBSYSTEM_NAME + "/FRPose",
        pose.transformBy(
            new Transform2d(
                RobotConfig.getInstance().getFrontRightCornerPosition(), new Rotation2d())));

    // check for position outside the field due to slipping
    if (pose.getX() < 0) {
      this.resetPose(new Pose2d(0, pose.getY(), pose.getRotation()));
      this.constrainPoseToFieldCount++;
    } else if (pose.getX() > FieldConstants.fieldLength) {
      this.resetPose(new Pose2d(FieldConstants.fieldLength, pose.getY(), pose.getRotation()));
      this.constrainPoseToFieldCount++;
    }
    if (pose.getY() < 0) {
      this.resetPose(new Pose2d(pose.getX(), 0, pose.getRotation()));
      this.constrainPoseToFieldCount++;
    } else if (pose.getY() > FieldConstants.fieldWidth) {
      this.resetPose(new Pose2d(pose.getX(), FieldConstants.fieldWidth, pose.getRotation()));
      this.constrainPoseToFieldCount++;
    }
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/ConstrainPoseToFieldCount", this.constrainPoseToFieldCount);

    Logger.recordOutput(SUBSYSTEM_NAME + "/FieldRelative", this.getFieldRelative());

    Logger.recordOutput(
        SUBSYSTEM_NAME + "/Speed",
        Math.hypot(
            inputs.drivetrain.measuredChassisSpeeds.vxMetersPerSecond,
            inputs.drivetrain.measuredChassisSpeeds.vyMetersPerSecond),
        MetersPerSecond);

    // update the brake mode based on the robot's velocity and state (enabled/disabled)
    updateBrakeMode();

    // update tunables
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          autoXController.setPID(pid[0], pid[1], pid[2]);
          autoYController.setPID(pid[0], pid[1], pid[2]);
        },
        autoDriveKp,
        autoDriveKi,
        autoDriveKd);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> autoThetaController.setPID(pid[0], pid[1], pid[2]),
        autoTurnKp,
        autoTurnKi,
        autoTurnKd);

    // Record cycle time
    LoggedTracer.record("Drivetrain");
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
   * Returns the average current of the swerve module drive motors in amps.
   *
   * @return the average current of the swerve module drive motors in amps
   */
  public Current getAverageDriveCurrent() {
    return this.inputs.drivetrain.averageDriveCurrent;
  }

  /**
   * Get the position of all drive wheels in radians.
   *
   * @return the position of all drive wheels in radians
   */
  public double[] getWheelRadiusCharacterizationPosition() {
    double[] positions = new double[inputs.swerve.length];
    for (int i = 0; i < inputs.swerve.length; i++) {
      positions[i] =
          inputs.drivetrain.swerveModulePositions[i].distanceMeters
              / (RobotConfig.getInstance().getWheelRadius().in(Meters));
    }
    return positions;
  }

  /**
   * Captures the initial positions of the drive wheels. This method is intended to be invoked at
   * the start of an autonomous path to measure the distance traveled by the robot.
   */
  public void captureInitialConditions() {
    for (int i = 0; i < this.inputs.swerve.length; i++) {
      this.initialDistance[i] = inputs.drivetrain.swerveModulePositions[i].distanceMeters;
    }
  }

  /**
   * Captures the final positions of the drive wheels and the final pose of the robot. This method
   * is intended to be invoked at the end of an autonomous path to measure the distance traveled by
   * the robot and the final pose of the robot. It logs the difference between the pose and the
   * final target pose of the specified autonomous path. If also logs the distance traveled by the
   * robot during the autonomous path.
   *
   * @param autoName the name of the autonomous path
   * @param measureDistance true to measure the distance traveled by the robot; false otherwise
   */
  public void captureFinalConditions(String autoName, boolean measureDistance) {
    try {
      List<Pose2d> pathPoses = PathPlannerPath.fromPathFile(autoName).getPathPoses();
      Pose2d targetPose = pathPoses.get(pathPoses.size() - 1);
      Logger.recordOutput(SUBSYSTEM_NAME + "/AutoPoseDiff", targetPose.minus(this.customPose));

      if (measureDistance) {
        double distance = 0.0;
        for (int i = 0; i < this.inputs.swerve.length; i++) {
          distance +=
              Math.abs(
                  inputs.drivetrain.swerveModulePositions[i].distanceMeters
                      - this.initialDistance[i]);
        }

        distance /= this.inputs.swerve.length;
        Logger.recordOutput(SUBSYSTEM_NAME + "/AutoDistanceDiff", distance, Meters);
      }
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file: " + autoName);
      pathFileMissingAlert.set(true);
    }
  }

  /**
   * Returns true if the auto path, which is always defined for a blue alliance robot, should be
   * flipped to the red alliance side of the field.
   *
   * @return true if the auto path should be flipped to the red alliance side of the field
   */
  public boolean shouldFlipAutoPath() {
    return Field2d.getInstance().getAlliance() == Alliance.Red;
  }

  /**
   * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
   * stopped moving for the specified period of time, and brake mode is enabled; disable it.
   */
  private void updateBrakeMode() {
    if (DriverStation.isEnabled()) {
      if (!brakeMode) {
        brakeMode = true;
        setBrakeMode(true);
      }
      brakeModeTimer.restart();

    } else if (!DriverStation.isEnabled()) {
      boolean stillMoving = false;
      double velocityLimit =
          RobotConfig.getInstance().getRobotMaxCoastVelocity().in(MetersPerSecond);
      if (Math.abs(this.inputs.drivetrain.measuredChassisSpeeds.vxMetersPerSecond) > velocityLimit
          || Math.abs(this.inputs.drivetrain.measuredChassisSpeeds.vyMetersPerSecond)
              > velocityLimit) {
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

  public void enableRotationOverride() {
    this.isRotationOverrideEnabled = true;
  }

  public void disableRotationOverride() {
    this.isRotationOverrideEnabled = false;
  }

  /*
   * enable and disable acceleration limiting
   */
  public void enableAccelerationLimiting() {
    this.accelerationLimiting = true;
  }

  public void disableAccelerationLimiting() {
    this.accelerationLimiting = false;
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    // Some condition that should decide if we want to override rotation
    if (this.isRotationOverrideEnabled) {
      Rotation2d targetRotation = new Rotation2d();
      Logger.recordOutput(SUBSYSTEM_NAME + "/rotationOverride", targetRotation);
      return Optional.of(targetRotation);
    } else {
      // return an empty optional when we don't want to override the path's rotation
      return Optional.empty();
    }
  }

  public Pose2d getFutureRobotPose(
      double translationSecondsInFuture, double rotationSecondsInFuture) {
    // project the robot pose into the future based on the current translational velocity; don't
    // project the current rotational velocity as that will adversely affect the control loop
    // attempting to reach the rotational setpoint.
    return this.getPose()
        .exp(
            new Twist2d(
                this.getRobotRelativeSpeeds().vxMetersPerSecond * translationSecondsInFuture,
                this.getRobotRelativeSpeeds().vyMetersPerSecond * translationSecondsInFuture,
                this.getRobotRelativeSpeeds().omegaRadiansPerSecond * rotationSecondsInFuture));
  }

  public Pose2d getCustomEstimatedPose() {
    return this.customPose;
  }

  public void resetCustomPose(Pose2d poseMeters) {
    this.io.resetPose(poseMeters);
  }

  public Optional<Pose2d> samplePoseAt(double timestamp) {
    return this.io.samplePoseAt(timestamp);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    this.io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  private void applySysIdCharacterization(SysIDCharacterizationMode mode, double value) {
    this.io.applySysIdCharacterization(mode, value);
  }

  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(this::disableFieldRelative, this),
            getSwerveCheckCommand(SwerveCheckTypes.LEFT),
            getSwerveCheckCommand(SwerveCheckTypes.RIGHT),
            getSwerveCheckCommand(SwerveCheckTypes.FORWARD),
            getSwerveCheckCommand(SwerveCheckTypes.BACKWARD),
            getSwerveCheckCommand(SwerveCheckTypes.CLOCKWISE),
            getSwerveCheckCommand(SwerveCheckTypes.COUNTERCLOCKWISE))
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(
            Commands.runOnce(
                () ->
                    this.drive(
                        MetersPerSecond.of(0.0),
                        MetersPerSecond.of(0.0),
                        RadiansPerSecond.of(0.0),
                        true,
                        false),
                this));
  }

  public void setDriveToPoseCanceled(boolean canceled) {
    this.driveToPoseCanceled = canceled;
  }

  public boolean getDriveToPoseCanceled() {
    return this.driveToPoseCanceled;
  }

  public boolean isTilted() {
    boolean isTilted =
        this.inputs.drivetrain.roll.gt(TILT_THRESHOLD)
            || this.inputs.drivetrain.roll.lt(TILT_THRESHOLD.unaryMinus())
            || this.inputs.drivetrain.pitch.gt(TILT_THRESHOLD)
            || this.inputs.drivetrain.pitch.lt(TILT_THRESHOLD.unaryMinus());
    if (isTilted) {
      LEDs.getInstance().requestState(LEDs.States.UNTILTING_ROBOT);
    }

    return isTilted;
  }

  public void untilt() {
    double rollRad = this.inputs.drivetrain.roll.in(Radians);
    double pitchRad = this.inputs.drivetrain.pitch.in(Radians);

    double gravityX =
        (9.8 * Math.cos(pitchRad) * Math.cos(rollRad) * Math.sin(pitchRad) * Math.cos(pitchRad));
    double gravityY = (-9.8 * Math.cos(pitchRad) * Math.cos(rollRad) * Math.sin(rollRad));

    double heading = Math.atan2(gravityY, gravityX);
    LinearVelocity xVelocity = UNTILT_VELOCITY.times(Math.cos(heading));
    LinearVelocity yVelocity = UNTILT_VELOCITY.times(Math.sin(heading));

    this.drive(xVelocity, yVelocity, RadiansPerSecond.of(0.0), false, false);
  }

  // method to convert swerve module number to location
  private String getSwerveLocation(int swerveModuleNumber) {
    switch (swerveModuleNumber) {
      case 0:
        return "FL";
      case 1:
        return "FR";
      case 2:
        return "BL";
      case 3:
        return "BR";
      default:
        return "UNKNOWN";
    }
  }

  /**
   * Checks the swerve module to see if its velocity and rotation are within the specified tolerance
   * of the specified values.
   *
   * @param swerveModuleNumber the swerve module number to check
   * @param angleTarget the target angle of the swerve module
   * @param angleTolerance the tolerance of the angle
   * @param velocityTarget the target velocity of the swerve module
   * @param velocityTolerance the tolerance of the velocity
   */
  private void checkSwerveModule(
      int swerveModuleNumber,
      Angle angleTarget,
      Angle angleTolerance,
      LinearVelocity velocityTarget,
      LinearVelocity velocityTolerance) {

    boolean isOffset = false;

    // Check to see if the direction is rotated properly
    // steerAbsolutePositionDeg is a value that is between (-180, 180]
    if (this.inputs.swerve[swerveModuleNumber].steerAbsolutePosition.isNear(
        angleTarget.minus(Degrees.of(180)), angleTolerance)) {
      isOffset = true;
    } else if (this.inputs.swerve[swerveModuleNumber].steerAbsolutePosition.isNear(
        angleTarget.plus(Degrees.of(180)), angleTolerance)) {
      isOffset = true;
    }
    // if not, add fault
    else if (!this.inputs.swerve[swerveModuleNumber].steerAbsolutePosition.isNear(
        angleTarget, angleTolerance)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "[System Check] Swerve module "
                  + getSwerveLocation(swerveModuleNumber)
                  + " not rotating in the threshold as expected. Should be: "
                  + angleTarget
                  + " is: "
                  + inputs.swerve[swerveModuleNumber].steerAbsolutePosition);
    }

    // Checks the velocity of the swerve module depending on if there is an offset
    LinearVelocity velocityMeasured =
        MetersPerSecond.of(
            inputs.drivetrain.swerveMeasuredStates[swerveModuleNumber].speedMetersPerSecond);
    if (!isOffset) {
      if (!velocityMeasured.isNear(velocityTarget.unaryMinus(), velocityTolerance)) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "[System Check] Swerve module "
                    + getSwerveLocation(swerveModuleNumber)
                    + " not moving as fast as expected. Should be: "
                    + velocityTarget
                    + " is: "
                    + velocityMeasured);
      }
    } else { // if there is an offset, check the velocity in the opposite direction
      if (!velocityMeasured.isNear(velocityTarget, velocityTolerance)) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "[System Check] Swerve module "
                    + getSwerveLocation(swerveModuleNumber)
                    + " not moving as fast as expected. REVERSED Should be: "
                    + velocityTarget
                    + " is: "
                    + velocityMeasured);
      }
    }
  }

  private Command getSwerveCheckCommand(SwerveCheckTypes type) {

    LinearVelocity xVelocity;
    LinearVelocity yVelocity;
    AngularVelocity rotationalVelocity;

    Angle angleTarget;
    LinearVelocity velocityTarget;

    switch (type) {
      case LEFT:
        xVelocity = MetersPerSecond.of(0.0);
        yVelocity = MetersPerSecond.of(1.0);
        rotationalVelocity = RadiansPerSecond.of(0.0);
        velocityTarget = MetersPerSecond.of(1.0);
        angleTarget = Degrees.of(90.0);
        break;
      case RIGHT:
        xVelocity = MetersPerSecond.of(0.0);
        yVelocity = MetersPerSecond.of(-1.0);
        rotationalVelocity = RadiansPerSecond.of(0.0);
        velocityTarget = MetersPerSecond.of(-1.0);
        angleTarget = Degrees.of(90.0);
        break;
      case FORWARD:
        xVelocity = MetersPerSecond.of(1.0);
        yVelocity = MetersPerSecond.of(0.0);
        rotationalVelocity = RadiansPerSecond.of(0.0);
        velocityTarget = MetersPerSecond.of(1.0);
        angleTarget = Degrees.of(0.0);
        break;
      case BACKWARD:
        xVelocity = MetersPerSecond.of(-1.0);
        yVelocity = MetersPerSecond.of(0.0);
        rotationalVelocity = RadiansPerSecond.of(0.0);
        velocityTarget = MetersPerSecond.of(-1.0);
        angleTarget = Degrees.of(0.0);
        break;
      case CLOCKWISE:
        return Commands.parallel(
                Commands.run(
                    () ->
                        this.drive(
                            MetersPerSecond.of(0.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(-Math.PI),
                            false,
                            false),
                    this),
                Commands.waitSeconds(1)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              checkSwerveModule(
                                  0,
                                  Degrees.of(135.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE,
                                  MetersPerSecond.of(-0.38 * Math.PI),
                                  SYSTEM_TEST_VELOCITY_TOLERANCE);
                              checkSwerveModule(
                                  1,
                                  Degrees.of(45.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE,
                                  MetersPerSecond.of(-0.38 * Math.PI),
                                  SYSTEM_TEST_VELOCITY_TOLERANCE);
                              checkSwerveModule(
                                  2,
                                  Degrees.of(45.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE,
                                  MetersPerSecond.of(0.38 * Math.PI),
                                  SYSTEM_TEST_VELOCITY_TOLERANCE);
                              checkSwerveModule(
                                  3,
                                  Degrees.of(135.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE,
                                  MetersPerSecond.of(0.38 * Math.PI),
                                  SYSTEM_TEST_VELOCITY_TOLERANCE);
                            })))
            .withTimeout(1);
      case COUNTERCLOCKWISE:
        return Commands.parallel(
                Commands.run(
                    () ->
                        this.drive(
                            MetersPerSecond.of(0.0),
                            MetersPerSecond.of(0.0),
                            RadiansPerSecond.of(Math.PI),
                            false,
                            false),
                    this),
                Commands.waitSeconds(1)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              checkSwerveModule(
                                  0,
                                  Degrees.of(135.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE,
                                  MetersPerSecond.of(0.38 * Math.PI),
                                  SYSTEM_TEST_VELOCITY_TOLERANCE);
                              checkSwerveModule(
                                  1,
                                  Degrees.of(45.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE,
                                  MetersPerSecond.of(0.38 * Math.PI),
                                  SYSTEM_TEST_VELOCITY_TOLERANCE);
                              checkSwerveModule(
                                  2,
                                  Degrees.of(45.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE,
                                  MetersPerSecond.of(-0.38 * Math.PI),
                                  SYSTEM_TEST_VELOCITY_TOLERANCE);
                              checkSwerveModule(
                                  3,
                                  Degrees.of(135.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE,
                                  MetersPerSecond.of(-0.38 * Math.PI),
                                  SYSTEM_TEST_VELOCITY_TOLERANCE);
                            })))
            .withTimeout(1);
      default:
        xVelocity = MetersPerSecond.of(0.0);
        yVelocity = MetersPerSecond.of(0.0);
        rotationalVelocity = RadiansPerSecond.of(0.0);
        velocityTarget = MetersPerSecond.of(0.0);
        angleTarget = Degrees.of(0.0);
        break;
    }

    return Commands.parallel(
            Commands.run(
                () -> this.drive(xVelocity, yVelocity, rotationalVelocity, false, false), this),
            Commands.waitSeconds(1)
                .andThen(
                    Commands.runOnce(
                        () -> {
                          for (int i = 0; i < this.inputs.swerve.length; i++) {
                            checkSwerveModule(
                                i,
                                angleTarget,
                                SYSTEM_TEST_ANGLE_TOLERANCE,
                                velocityTarget,
                                SYSTEM_TEST_VELOCITY_TOLERANCE);
                          }
                        })))
        .withTimeout(2);
  }

  private enum SwerveCheckTypes {
    LEFT,
    RIGHT,
    FORWARD,
    BACKWARD,
    CLOCKWISE,
    COUNTERCLOCKWISE
  }
}
