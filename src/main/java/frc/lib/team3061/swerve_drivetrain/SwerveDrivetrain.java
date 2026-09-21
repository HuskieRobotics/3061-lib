// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team3061.swerve_drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainConstants.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
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
import frc.lib.team3061.util.MathUtils;
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

  private final LoggedTunableNumber testingMode =
      new LoggedTunableNumber("Drivetrain/TestingMode", 0);
  private final LoggedTunableNumber driveCurrent =
      new LoggedTunableNumber("Drivetrain/DriveCurrent", 0);
  private final LoggedTunableNumber driveVelocity =
      new LoggedTunableNumber("Drivetrain/DriveVelocity", 0);

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

  private final LoggedTunableNumber slowModeMultiplierTuneable =
      new LoggedTunableNumber(
          "Drivetrain/SlowModeMultiplier", RobotConfig.getInstance().getRobotSlowModeMultiplier());
  private final LoggedTunableNumber maxAccelerationWhenLimitedTuneable =
      new LoggedTunableNumber(
          "Drivetrain/MaxAccelerationWhenLimitedMPSPS",
          RobotConfig.getInstance().getRobotMaxAccelerationWhenLimitedMPSPS());
  private final LoggedTunableNumber maxAngularAccelerationWhenLimitedTuneable =
      new LoggedTunableNumber(
          "Drivetrain/MaxAngularAccelerationWhenLimitedRPSPS",
          RobotConfig.getInstance().getRobotMaxAngularAccelerationWhenLimitedRPSPS());

  private final PIDController autoXController =
      new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoYController =
      new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoThetaController =
      new PIDController(autoTurnKp.get(), autoTurnKi.get(), autoTurnKd.get());

  private boolean isFieldRelative = true;
  private boolean isTranslationSlowMode = false;
  private boolean isRotationSlowMode = false;
  private double slowModeMultiplier = RobotConfig.getInstance().getRobotSlowModeMultiplier();

  // set to true upon construction to trigger disabling break mode shortly after the code starts
  private boolean brakeMode = true;
  private Timer brakeModeTimer = new Timer();
  private static final double BREAK_MODE_DELAY_SEC = 10.0;

  private SlewRateLimiter xFilter =
      new SlewRateLimiter(RobotConfig.getInstance().getRobotMaxAccelerationWhenLimitedMPSPS());
  private SlewRateLimiter yFilter =
      new SlewRateLimiter(RobotConfig.getInstance().getRobotMaxAccelerationWhenLimitedMPSPS());
  private SlewRateLimiter thetaFilter =
      new SlewRateLimiter(
          RobotConfig.getInstance().getRobotMaxAngularAccelerationWhenLimitedRPSPS());

  private boolean accelerationLimiting = false;
  private boolean driveToPoseCanceled = false;
  private static final double ACCELERATION_LIMITING_MAX_VELOCITY_MPS = 1.5;
  private static final double ACCELERATION_LIMITING_MAX_ANGULAR_VELOCITY_RPS = 4.0;

  private Alert noPoseAlert =
      new Alert("Attempted to reset pose from vision, but no pose was found.", AlertType.kWarning);
  private Alert pathFileMissingAlert =
      new Alert("Could not find the specified path file.", AlertType.kError);

  private final SwerveRobotOdometry odometry;
  private int constrainPoseToFieldCount = 0;

  /*
   * State for validating odometry samples. When a CAN bus drops off the RIO (e.g., a CANivore
   * browning out and re-enumerating on USB), Phoenix continues to invoke the telemetry callback with
   * a valid timestamp but with every signal at its default value of zero. Integrating those zeros
   * produces an enormous, bogus twist. Refer to isOdometrySampleValid.
   */
  private int previousSuccessfulDAQs = -1;
  private double lastAcceptedOdometryTimestamp = -1.0;
  private Rotation2d lastAcceptedOdometryYaw = new Rotation2d();
  private int rejectedOdometrySampleCount = 0;
  private int skidCorrectedSampleCount = 0;

  /*
   * The module positions most recently handed to the pose estimator. While the robot is skidding
   * these diverge from the raw measurements in modulePositions, so the estimator's baseline has to
   * be tracked separately. Refer to removeSkid.
   */
  private SwerveModulePosition[] integratedModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final Alert odometryUnhealthyAlert =
      new Alert(
          "Rejecting odometry samples; check the CAN bus and the gyro connection.",
          AlertType.kError);
  private final Alert poseFarOutsideFieldAlert =
      new Alert(
          "Estimated pose is far outside of the field; relying on vision to recover.",
          AlertType.kWarning);

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
    Pose2d pose = this.getPose();
    Pose2d zeroedPose = new Pose2d(pose.getX(), pose.getY(), new Rotation2d());

    this.odometry.resetPose(
        Rotation2d.fromRadians(Units.degreesToRadians(this.inputs.drivetrain.rawHeadingDeg)),
        this.modulePositions,
        zeroedPose);
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
  public double getYawDeg() {
    return this.inputs.drivetrain.rawHeadingDeg;
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
        Rotation2d.fromDegrees(this.inputs.drivetrain.rawHeadingDeg), this.modulePositions, pose);

    // resetPosition sets the estimator's baseline to the raw positions passed above, so the
    // skid-corrected baseline has to be resynchronized or the next sample's delta would be the
    // accumulated difference between the two.
    for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
      this.integratedModulePositions[moduleIndex].distanceMeters =
          this.modulePositions[moduleIndex].distanceMeters;
      this.integratedModulePositions[moduleIndex].angle = this.modulePositions[moduleIndex].angle;
    }
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
   * @param xVelocityMPS the desired velocity in the x direction (m/s)
   * @param yVelocityMPS the desired velocity in the y direction (m/s)
   * @param rotationalVelocityRadiansPerSecond the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   * @param isFieldRelative true to for field-relative motion; false, for robot-relative
   */
  public void drive(
      double xVelocityMPS,
      double yVelocityMPS,
      double rotationalVelocityRadiansPerSecond,
      boolean isOpenLoop,
      boolean isFieldRelative) {

    // log velocity before and after filter
    Logger.recordOutput(SUBSYSTEM_NAME + "/requestedXVelocity", xVelocityMPS);
    Logger.recordOutput(SUBSYSTEM_NAME + "/requestedYVelocity", yVelocityMPS);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/requestedRotationalVelocity", rotationalVelocityRadiansPerSecond);

    // always calculate whenever we are driving so that we maintain a history of recent values
    this.xFilter.calculate(xVelocityMPS);
    this.yFilter.calculate(yVelocityMPS);
    this.thetaFilter.calculate(rotationalVelocityRadiansPerSecond);

    // log velocity after filter
    Logger.recordOutput(SUBSYSTEM_NAME + "/filteredXVelocity", this.xFilter.lastValue());
    Logger.recordOutput(SUBSYSTEM_NAME + "/filteredYVelocity", this.yFilter.lastValue());
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/filteredRotationalVelocity", this.thetaFilter.lastValue());

    if (accelerationLimiting) {
      xVelocityMPS = this.xFilter.lastValue();
      yVelocityMPS = this.yFilter.lastValue();
      rotationalVelocityRadiansPerSecond = this.thetaFilter.lastValue();
      double currentVelocity = Math.sqrt(Math.pow(xVelocityMPS, 2) + Math.pow(yVelocityMPS, 2));
      if (currentVelocity > ACCELERATION_LIMITING_MAX_VELOCITY_MPS) {
        xVelocityMPS = xVelocityMPS / currentVelocity * ACCELERATION_LIMITING_MAX_VELOCITY_MPS;
        yVelocityMPS = yVelocityMPS / currentVelocity * ACCELERATION_LIMITING_MAX_VELOCITY_MPS;
      }

      if (rotationalVelocityRadiansPerSecond > ACCELERATION_LIMITING_MAX_ANGULAR_VELOCITY_RPS) {
        rotationalVelocityRadiansPerSecond = ACCELERATION_LIMITING_MAX_ANGULAR_VELOCITY_RPS;
      } else if (rotationalVelocityRadiansPerSecond
          < -ACCELERATION_LIMITING_MAX_ANGULAR_VELOCITY_RPS) {
        rotationalVelocityRadiansPerSecond = -ACCELERATION_LIMITING_MAX_ANGULAR_VELOCITY_RPS;
      }
    }

    // if translation or rotation is in slow mode, multiply the x and y velocities by the
    // slow-mode multiplier
    if (isTranslationSlowMode) {
      xVelocityMPS = xVelocityMPS * slowModeMultiplier;
      yVelocityMPS = yVelocityMPS * slowModeMultiplier;
    }

    if (Constants.DEMO_MODE) {
      double velocity = Math.sqrt(Math.pow(xVelocityMPS, 2) + Math.pow(yVelocityMPS, 2));

      if (velocity > DEMO_MODE_MAX_VELOCITY_MPS) {
        double scale = DEMO_MODE_MAX_VELOCITY_MPS / velocity;
        xVelocityMPS = xVelocityMPS * scale;
        yVelocityMPS = yVelocityMPS * scale;
      }
    }

    // if rotation is in slow mode, multiply the rotational velocity by the slow-mode multiplier
    if (isRotationSlowMode) {
      rotationalVelocityRadiansPerSecond = rotationalVelocityRadiansPerSecond * slowModeMultiplier;
    }

    if (isFieldRelative) {
      // the origin of the field is always the corner to the right of the blue alliance driver
      // station. As a result, "forward" from a field-relative perspective when on the red
      // alliance, is in the negative x direction. Similarly, "left" from a field-relative
      // perspective when on the red alliance is in the negative y direction.
      if (Field2d.getInstance().getAlliance() != Alliance.Blue) {
        xVelocityMPS = xVelocityMPS * -1;
        yVelocityMPS = yVelocityMPS * -1;
      }
      this.io.driveFieldRelative(
          xVelocityMPS, yVelocityMPS, rotationalVelocityRadiansPerSecond, isOpenLoop);
    } else {
      this.io.driveRobotRelative(
          xVelocityMPS, yVelocityMPS, rotationalVelocityRadiansPerSecond, isOpenLoop);
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
   * @param xVelocityMPS the desired velocity in the x direction (m/s)
   * @param yVelocityMPS the desired velocity in the y direction (m/s)
   * @param targetDirection the desired direction of the robot's orientation. Zero degrees is
   *     aligned to the positive x axis and increases in the CCW direction.
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  public void driveFacingAngle(
      double xVelocityMPS, double yVelocityMPS, Rotation2d targetDirection, boolean isOpenLoop) {

    // always calculate whenever we are driving so that we maintain a history of recent values
    this.xFilter.calculate(xVelocityMPS);
    this.yFilter.calculate(yVelocityMPS);
    this.thetaFilter.calculate(0.0);

    // if translation or rotation is in slow mode, multiply the x and y velocities by the
    // slow-mode multiplier
    if (isTranslationSlowMode) {
      xVelocityMPS = xVelocityMPS * slowModeMultiplier;
      yVelocityMPS = yVelocityMPS * slowModeMultiplier;
    }

    if (Constants.DEMO_MODE) {
      double velocity = Math.sqrt(Math.pow(xVelocityMPS, 2) + Math.pow(yVelocityMPS, 2));
      if (velocity > DEMO_MODE_MAX_VELOCITY_MPS) {
        double scale = DEMO_MODE_MAX_VELOCITY_MPS / velocity;
        xVelocityMPS = xVelocityMPS * scale;
        yVelocityMPS = yVelocityMPS * scale;
      }
    }

    if (Field2d.getInstance().getAlliance() != Alliance.Blue) {
      xVelocityMPS = xVelocityMPS * -1;
      yVelocityMPS = yVelocityMPS * -1;
    }
    this.io.driveFieldRelativeFacingAngle(xVelocityMPS, yVelocityMPS, targetDirection, isOpenLoop);

    Logger.recordOutput(SUBSYSTEM_NAME + "/DriveFacingAngle/targetDirection", targetDirection);
  }

  /**
   * Stops the motion of the robot. Since the motors are in brake mode, the robot will stop soon
   * after this method is invoked.
   */
  public void stop() {
    this.io.driveRobotRelative(0.0, 0.0, 0.0, false);
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

    if (testingMode.get() == 1) {
      if (driveCurrent.get() != 0) {
        this.io.setDriveCurrent(driveCurrent.get());
      } else if (driveVelocity.get() != 0) {
        this.drive(driveVelocity.get(), 0.0, 0.0, false, true);
      }
    }

    this.io.updateInputs(this.inputs);
    Logger.processInputs(SUBSYSTEM_NAME, this.inputs.drivetrain);
    Logger.processInputs(SUBSYSTEM_NAME + "/FL", this.inputs.swerve[0]);
    Logger.processInputs(SUBSYSTEM_NAME + "/FR", this.inputs.swerve[1]);
    Logger.processInputs(SUBSYSTEM_NAME + "/BL", this.inputs.swerve[2]);
    Logger.processInputs(SUBSYSTEM_NAME + "/BR", this.inputs.swerve[3]);

    // update odometry
    this.updateOdometry();

    // give chassis speeds to odometry for public access throughout the robot
    this.odometry.updateChassisSpeeds(this.getRobotRelativeSpeeds());

    // custom pose vs default pose
    Pose2d pose = this.odometry.getEstimatedPose();
    this.customPose = this.inputs.drivetrain.customPose;
    Logger.recordOutput(SUBSYSTEM_NAME + "/Pose", pose);
    Logger.recordOutput(SUBSYSTEM_NAME + "/CustomPose", this.customPose);

    // check for position outside the field due to slipping
    this.constrainPoseToField(pose);

    Logger.recordOutput(SUBSYSTEM_NAME + "/FieldRelative", this.getFieldRelative());

    if (ENABLE_EXTRA_LOGGING) {
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/FRPose",
          pose.transformBy(
              new Transform2d(
                  RobotConfig.getInstance().getFrontRightCornerPosition(), new Rotation2d())));

      Logger.recordOutput(
          SUBSYSTEM_NAME + "/Speed",
          Math.hypot(
              inputs.drivetrain.measuredChassisSpeeds.vxMetersPerSecond,
              inputs.drivetrain.measuredChassisSpeeds.vyMetersPerSecond),
          MetersPerSecond);
    }

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

    LoggedTunableNumber.ifChanged(
        hashCode(),
        limits -> {
          slowModeMultiplier = limits[0];
        },
        slowModeMultiplierTuneable);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        limits -> {
          xFilter = new SlewRateLimiter(limits[0]);
          yFilter = new SlewRateLimiter(limits[0]);
          thetaFilter = new SlewRateLimiter(limits[1]);
        },
        maxAccelerationWhenLimitedTuneable,
        maxAngularAccelerationWhenLimitedTuneable);

    Logger.recordOutput(SUBSYSTEM_NAME + "/SlowModeMultiplier", slowModeMultiplier);
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/MaxAccelerationWhenLimited", maxAccelerationWhenLimitedTuneable.get());
    Logger.recordOutput(
        SUBSYSTEM_NAME + "/MaxAngularAccelerationWhenLimited",
        maxAngularAccelerationWhenLimitedTuneable.get());

    // Record cycle time
    LoggedTracer.record("Drivetrain");
  }

  /**
   * Replays the odometry samples that were captured by the hardware-specific layer since the last
   * iteration into the pose estimator. Samples that cannot be trusted are rejected instead of being
   * integrated.
   *
   * <p>When a CAN bus disappears from the RIO (e.g., a CANivore that browns out and re-enumerates
   * on USB), Phoenix continues to publish a SwerveDriveState with a valid timestamp but with every
   * signal at its default value of zero. Integrating a sample in which all four wheel distances and
   * the gyro's yaw simultaneously jump to zero yields a twist of tens of meters, which teleports
   * the estimated pose off of the field.
   */
  private void updateOdometry() {
    // The queues in the hardware-specific layer are drained together, so these arrays are expected
    // to be the same length. Bail out rather than risk an exception if that ever changes.
    int sampleCount =
        Math.min(
            inputs.drivetrain.odometryTimestamps.length,
            inputs.drivetrain.odometryYawPositions.length);
    for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
      sampleCount =
          Math.min(
              sampleCount,
              Math.min(
                  inputs.swerve[moduleIndex].odometryDrivePositionsMeters.length,
                  inputs.swerve[moduleIndex].odometryTurnPositions.length));
    }

    // The gyro and data acquisition counters describe the state of the bus at the end of this
    // iteration, whereas the queued samples span the entire iteration. Samples that were captured
    // before the bus failed are still valid, so these signals only raise an alert for the driver;
    // they must not gate the samples themselves. Each sample is validated individually instead.
    boolean daqAdvanced = inputs.drivetrain.successfulDAQs != this.previousSuccessfulDAQs;
    this.previousSuccessfulDAQs = inputs.drivetrain.successfulDAQs;
    this.odometryUnhealthyAlert.set(
        !inputs.drivetrain.gyroConnected || (sampleCount > 0 && !daqAdvanced));

    // The skid ratio is derived from inputs.drivetrain.swerveMeasuredStates, which is a single
    // snapshot of the module velocities taken once per iteration, so it is the same for every
    // sample drained below.
    boolean skidding = this.computeSkidRatio() >= SKID_RATIO_THRESHOLD;

    for (int i = 0; i < sampleCount; i++) {
      if (!isOdometrySampleValid(i)) {
        // Do not commit this sample to modulePositions. The pose estimator computes its wheel
        // deltas relative to the last sample it was given, so skipping a sample makes the next
        // accepted sample's delta span the gap. No distance is lost; it is only deferred.
        this.rejectedOdometrySampleCount++;
        continue;
      }

      this.integrateOdometrySample(i, skidding);
    }

    Logger.recordOutput(
        SUBSYSTEM_NAME + "/SkidCorrectedSampleCount", this.skidCorrectedSampleCount);

    Logger.recordOutput(
        SUBSYSTEM_NAME + "/RejectedOdometrySampleCount", this.rejectedOdometrySampleCount);
  }

  /**
   * Returns the ratio between the translational speed of the fastest module and that of the
   * slowest, after the rotational component of each module's velocity has been removed. A robot
   * that is tracking cleanly has all four modules translating at the same speed, giving a ratio of
   * 1.0; a wheel that is slipping spins faster than the rest and drives the ratio up. Refer to
   * https://www.pramit.gg/post/is-my-robot-skidding for details.
   *
   * @return the skid ratio, or 1.0 if the robot is not translating enough to compute one
   */
  private double computeSkidRatio() {
    SwerveDriveKinematics kinematics = RobotConfig.getInstance().getSwerveDriveKinematics();
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(inputs.drivetrain.swerveMeasuredStates);
    SwerveModuleState[] rotationOnly =
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, speeds.omegaRadiansPerSecond));

    double maxTranslation = 0.0;
    double minTranslation = Double.POSITIVE_INFINITY;
    for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
      Translation2d measured =
          new Translation2d(
              inputs.drivetrain.swerveMeasuredStates[moduleIndex].speedMetersPerSecond,
              inputs.drivetrain.swerveMeasuredStates[moduleIndex].angle);
      Translation2d rotational =
          new Translation2d(
              rotationOnly[moduleIndex].speedMetersPerSecond, rotationOnly[moduleIndex].angle);
      double translation = measured.minus(rotational).getNorm();
      maxTranslation = Math.max(translation, maxTranslation);
      minTranslation = Math.min(translation, minTranslation);
    }

    if (ENABLE_EXTRA_LOGGING) {
      Logger.recordOutput(SUBSYSTEM_NAME + "/maxTranslation", maxTranslation);
      Logger.recordOutput(SUBSYSTEM_NAME + "/minTranslation", minTranslation);
    }

    double skidRatio = 1.0;
    // only calculate the skid ratio if the robot has a significant translation
    if (minTranslation > 1e-4 && maxTranslation > .01) {
      skidRatio = maxTranslation / minTranslation;
    }
    Logger.recordOutput(SUBSYSTEM_NAME + "/skidRatio", skidRatio);

    return skidRatio;
  }

  /**
   * Integrates one validated odometry sample into the pose estimator and advances the baselines
   * that subsequent samples are measured against.
   *
   * @param sampleIndex the index of the sample to integrate
   * @param skidding true if the modules are slipping and the wheel distances should be corrected
   *     before they are integrated
   */
  private void integrateOdometrySample(int sampleIndex, boolean skidding) {
    boolean firstSample = this.lastAcceptedOdometryTimestamp < 0.0;

    if (firstSample) {
      // There is no previous sample to take a displacement against, so seed both baselines.
      for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
        this.integratedModulePositions[moduleIndex].distanceMeters =
            inputs.swerve[moduleIndex].odometryDrivePositionsMeters[sampleIndex];
        this.integratedModulePositions[moduleIndex].angle =
            inputs.swerve[moduleIndex].odometryTurnPositions[sampleIndex];
      }
    } else {
      // The displacement of each module since the last accepted sample, as a vector in the robot
      // frame: how far the wheel rolled, in the direction the wheel was pointing.
      Translation2d[] displacements = new Translation2d[this.modulePositions.length];
      for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
        displacements[moduleIndex] =
            new Translation2d(
                inputs.swerve[moduleIndex].odometryDrivePositionsMeters[sampleIndex]
                    - this.modulePositions[moduleIndex].distanceMeters,
                inputs.swerve[moduleIndex].odometryTurnPositions[sampleIndex]);
      }

      if (skidding) {
        displacements =
            this.removeSkid(
                displacements,
                inputs.drivetrain.odometryYawPositions[sampleIndex].minus(
                    this.lastAcceptedOdometryYaw));
        this.skidCorrectedSampleCount++;
      }

      // SwerveDriveKinematics.toTwist2d takes each module's delta as the scalar difference in
      // distanceMeters directed along the *end* angle, so storing the magnitude and the direction
      // of the desired displacement makes the estimator integrate exactly that vector. This turns
      // distanceMeters into an unsigned path length rather than a signed odometer, which is
      // immaterial because only the difference between consecutive samples is ever read.
      for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
        this.integratedModulePositions[moduleIndex].distanceMeters +=
            displacements[moduleIndex].getNorm();
        this.integratedModulePositions[moduleIndex].angle = displacements[moduleIndex].getAngle();
      }
    }

    for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
      this.modulePositions[moduleIndex].distanceMeters =
          inputs.swerve[moduleIndex].odometryDrivePositionsMeters[sampleIndex];
      this.modulePositions[moduleIndex].angle =
          inputs.swerve[moduleIndex].odometryTurnPositions[sampleIndex];
    }

    this.lastAcceptedOdometryTimestamp = inputs.drivetrain.odometryTimestamps[sampleIndex];
    this.lastAcceptedOdometryYaw = inputs.drivetrain.odometryYawPositions[sampleIndex];

    this.odometry.updateWithTime(
        inputs.drivetrain.odometryTimestamps[sampleIndex],
        inputs.drivetrain.odometryYawPositions[sampleIndex],
        this.integratedModulePositions);
  }

  /**
   * Replaces the measured module displacements with the displacements the modules would have had if
   * none of them were slipping.
   *
   * <p>Each module's displacement is the sum of a rotational component, which is fixed by the
   * change in heading and the module's location, and a translational component, which is common to
   * all four modules when the robot is tracking cleanly. A slipping wheel rolls farther than it
   * carries the chassis, so it inflates its own translational component; the smallest of the four
   * is the one least contaminated by slip. Attributing that one to every module discards the
   * slipped distance instead of integrating it.
   *
   * <p>Skipping the update entirely would not discard anything: the pose estimator's baseline only
   * advances when it is given a sample, so the next accepted sample's delta would span the skid and
   * reintroduce every slipped meter.
   *
   * <p>This recovers the true chassis displacement exactly as long as at least one wheel is not
   * slipping, and is a no-op when none of them are. It has two limitations. If all four wheels slip
   * together there is no clean reference and the correction can only partially help. And because it
   * assumes a bad wheel reports too much distance, a wheel that reports too little -- one that is
   * dragging, or whose encoder is miscalibrated -- becomes the reference and its deficit is spread
   * to the other three, which is worse than not correcting at all.
   *
   * <p>An alternative this method does not take is to use the second smallest translation as the
   * reference rather than the smallest. That would fix the under-reporting wheel, but it gives up
   * the case of three wheels slipping at once, which the smallest recovers exactly and the second
   * smallest degrades to worse than no correction.
   *
   * @param displacements the measured displacement of each module since the last accepted sample
   * @param dtheta the change in the robot's heading over the same interval
   * @return the corrected displacement of each module
   */
  private Translation2d[] removeSkid(Translation2d[] displacements, Rotation2d dtheta) {
    // Kinematics is linear, so supplying an angular displacement where an angular velocity is
    // expected yields a linear displacement where a linear velocity is expected.
    SwerveModuleState[] rotationOnly =
        RobotConfig.getInstance()
            .getSwerveDriveKinematics()
            .toSwerveModuleStates(new ChassisSpeeds(0, 0, dtheta.getRadians()));

    Translation2d[] rotational = new Translation2d[displacements.length];
    Translation2d smallestTranslation = null;
    for (int moduleIndex = 0; moduleIndex < displacements.length; moduleIndex++) {
      rotational[moduleIndex] =
          new Translation2d(
              rotationOnly[moduleIndex].speedMetersPerSecond, rotationOnly[moduleIndex].angle);
      Translation2d translational = displacements[moduleIndex].minus(rotational[moduleIndex]);
      if (smallestTranslation == null || translational.getNorm() < smallestTranslation.getNorm()) {
        smallestTranslation = translational;
      }
    }

    Translation2d[] corrected = new Translation2d[displacements.length];
    for (int moduleIndex = 0; moduleIndex < displacements.length; moduleIndex++) {
      corrected[moduleIndex] = smallestTranslation.plus(rotational[moduleIndex]);
    }
    return corrected;
  }

  /**
   * Returns true if the specified odometry sample is physically plausible given the last accepted
   * sample. A sample is rejected if any wheel or the gyro moved farther than the robot could have
   * moved in the elapsed time.
   *
   * @param sampleIndex the index of the sample to validate
   * @return true if the sample can be trusted
   */
  private boolean isOdometrySampleValid(int sampleIndex) {
    double timestamp = inputs.drivetrain.odometryTimestamps[sampleIndex];

    // Always accept the first sample; there is nothing to compare it against.
    if (this.lastAcceptedOdometryTimestamp < 0.0) {
      return true;
    }

    // A timestamp that does not advance falls back to the minimum tolerance rather than rejecting
    // the sample outright, so that a stalled timestamp cannot reject every subsequent sample.
    double elapsedTime = Math.max(0.0, timestamp - this.lastAcceptedOdometryTimestamp);

    double maxWheelDelta =
        Math.max(
            ODOMETRY_MIN_WHEEL_DELTA_METERS,
            RobotConfig.getInstance().getRobotMaxVelocityMPS()
                * ODOMETRY_MAX_DELTA_SCALAR
                * elapsedTime);

    for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
      double wheelDelta =
          Math.abs(
              inputs.swerve[moduleIndex].odometryDrivePositionsMeters[sampleIndex]
                  - this.modulePositions[moduleIndex].distanceMeters);
      if (wheelDelta > maxWheelDelta) {
        return false;
      }
    }

    double maxYawDelta =
        Math.max(
            ODOMETRY_MIN_YAW_DELTA_DEG,
            Units.radiansToDegrees(RobotConfig.getInstance().getRobotMaxAngularVelocityRPS())
                * ODOMETRY_MAX_DELTA_SCALAR
                * elapsedTime);

    // Subtract the headings as plain degrees rather than with Rotation2d.minus. A gyro that has
    // dropped off the bus reports a raw yaw of zero, and minus wraps its result to +/-180 degrees,
    // which hides that jump whenever the robot is near a whole number of turns: in the log that
    // motivated this check the heading was 720.21 degrees, whose wrapped difference from zero is
    // 0.21 degrees and would have been accepted at any threshold.
    //
    // WARNING: this relies on these Rotation2d values carrying a continuous angle. Rotation2d
    // stores the value it was constructed with verbatim, so getDegrees returns 720.21 here, but it
    // normalizes to +/-180 degrees on any arithmetic (minus, plus, rotateBy, interpolate). These
    // come straight from Rotation2d.fromDegrees in SwerveDrivetrainIOCTRE, which preserves the
    // winding that Phoenix reports. If anything is ever inserted into that path that operates on
    // them, this check silently degrades to detecting the jump only when the robot is far enough
    // from a whole number of turns -- no compile error and no exception, just weaker detection.
    double yawDelta =
        Math.abs(
            inputs.drivetrain.odometryYawPositions[sampleIndex].getDegrees()
                - this.lastAcceptedOdometryYaw.getDegrees());

    return yawDelta <= maxYawDelta;
  }

  /**
   * Constrains the estimated pose to the field if it has drifted slightly off of it due to wheel
   * slip.
   *
   * <p>Only small excursions are corrected. A pose that is far off of the field is the result of
   * corrupt odometry rather than slip, and clamping it would pin the robot to a field corner and
   * discard an estimate that vision can still recover. Resetting the pose also clears the pose
   * estimator's buffered odometry samples and accumulated vision corrections, so the deadband keeps
   * this from resetting the pose on every iteration while the robot is pressed against a wall.
   *
   * @param pose the current estimated pose
   */
  private void constrainPoseToField(Pose2d pose) {
    double constrainedX = MathUtil.clamp(pose.getX(), 0.0, FieldConstants.fieldLength);
    double constrainedY = MathUtil.clamp(pose.getY(), 0.0, FieldConstants.fieldWidth);

    double errorX = Math.abs(pose.getX() - constrainedX);
    double errorY = Math.abs(pose.getY() - constrainedY);
    double error = Math.max(errorX, errorY);

    boolean farOutsideField = error > CONSTRAIN_POSE_TO_FIELD_MAX_ERROR_METERS;
    this.poseFarOutsideFieldAlert.set(farOutsideField);

    if (error > CONSTRAIN_POSE_TO_FIELD_DEADBAND_METERS && !farOutsideField) {
      // Correct both axes with a single reset so that neither axis is computed from a stale pose.
      this.resetPose(new Pose2d(constrainedX, constrainedY, pose.getRotation()));
      this.constrainPoseToFieldCount++;
    }

    Logger.recordOutput(
        SUBSYSTEM_NAME + "/ConstrainPoseToFieldCount", this.constrainPoseToFieldCount);
    Logger.recordOutput(SUBSYSTEM_NAME + "/PoseOutsideFieldMeters", error);
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
  public double getAverageDriveCurrent() {
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
              / (RobotConfig.getInstance().getWheelRadiusMeters());
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
      double velocityLimit = RobotConfig.getInstance().getRobotMaxCoastVelocityMPS();
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

  public Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(this::disableFieldRelative, this),
            getSwerveCheckCommand(SwerveCheckTypes.LEFT),
            getSwerveCheckCommand(SwerveCheckTypes.RIGHT),
            getSwerveCheckCommand(SwerveCheckTypes.FORWARD),
            getSwerveCheckCommand(SwerveCheckTypes.BACKWARD),
            getSwerveCheckCommand(SwerveCheckTypes.CLOCKWISE),
            getSwerveCheckCommand(SwerveCheckTypes.COUNTERCLOCKWISE))
        .andThen(Commands.runOnce(() -> this.drive(0.0, 0.0, 0.0, true, false), this));
  }

  public void setDriveToPoseCanceled(boolean canceled) {
    this.driveToPoseCanceled = canceled;
  }

  public boolean getDriveToPoseCanceled() {
    return this.driveToPoseCanceled;
  }

  public boolean isTilted() {
    boolean isTilted =
        this.inputs.drivetrain.rollDeg > TILT_THRESHOLD_DEG
            || this.inputs.drivetrain.rollDeg < -TILT_THRESHOLD_DEG
            || this.inputs.drivetrain.pitchDeg > TILT_THRESHOLD_DEG
            || this.inputs.drivetrain.pitchDeg < -TILT_THRESHOLD_DEG;
    if (isTilted) {
      LEDs.getInstance().requestState(LEDs.States.UNTILTING_ROBOT);
    }

    return isTilted;
  }

  public void untilt() {
    double rollRad = Units.degreesToRadians(this.inputs.drivetrain.rollDeg);
    double pitchRad = Units.degreesToRadians(this.inputs.drivetrain.pitchDeg);

    double gravityX =
        (9.8 * Math.cos(pitchRad) * Math.cos(rollRad) * Math.sin(pitchRad) * Math.cos(pitchRad));
    double gravityY = (-9.8 * Math.cos(pitchRad) * Math.cos(rollRad) * Math.sin(rollRad));

    double heading = Math.atan2(gravityY, gravityX);
    double xVelocity = UNTILT_VELOCITY_MPS * (Math.cos(heading));
    double yVelocity = UNTILT_VELOCITY_MPS * (Math.sin(heading));

    this.drive(xVelocity, yVelocity, 0.0, false, false);
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
   * @param angleTargetRot the target angle of the swerve module
   * @param angleToleranceRot the tolerance of the angle
   * @param velocityTargetMPS the target velocity of the swerve module
   * @param velocityToleranceMPS the tolerance of the velocity
   */
  private void checkSwerveModule(
      int swerveModuleNumber,
      double angleTargetRot,
      double angleToleranceRot,
      double velocityTargetMPS,
      double velocityToleranceMPS) {

    boolean isOffset = false;

    // Check to see if the direction is rotated properly
    // steerAbsolutePositionDeg is a value that is between (-180, 180]
    if (MathUtils.isNear(
        this.inputs.swerve[swerveModuleNumber].steerAbsolutePositionRot,
        angleTargetRot - 0.5,
        angleToleranceRot)) {
      isOffset = true;
    } else if (MathUtils.isNear(
        this.inputs.swerve[swerveModuleNumber].steerAbsolutePositionRot,
        angleTargetRot + 0.5,
        angleToleranceRot)) {
      isOffset = true;
    }
    // if not, add fault
    else if (!MathUtils.isNear(
        this.inputs.swerve[swerveModuleNumber].steerAbsolutePositionRot,
        angleTargetRot,
        angleToleranceRot)) {
      FaultReporter.getInstance()
          .addFault(
              SUBSYSTEM_NAME,
              "[System Check] Swerve module "
                  + getSwerveLocation(swerveModuleNumber)
                  + " not rotating in the threshold as expected. Should be: "
                  + angleTargetRot
                  + " is: "
                  + inputs.swerve[swerveModuleNumber].steerAbsolutePositionRot);
    }

    // Checks the velocity of the swerve module depending on if there is an offset
    double velocityMeasuredMPS =
        inputs.drivetrain.swerveMeasuredStates[swerveModuleNumber].speedMetersPerSecond;
    if (!isOffset) {
      if (!MathUtils.isNear(velocityMeasuredMPS, velocityTargetMPS, velocityToleranceMPS)) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "[System Check] Swerve module "
                    + getSwerveLocation(swerveModuleNumber)
                    + " not moving as fast as expected. Should be: "
                    + velocityTargetMPS
                    + " is: "
                    + velocityMeasuredMPS);
      }
    } else { // if there is an offset, check the velocity in the opposite direction
      if (!MathUtils.isNear(velocityMeasuredMPS, -velocityTargetMPS, velocityToleranceMPS)) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "[System Check] Swerve module "
                    + getSwerveLocation(swerveModuleNumber)
                    + " not moving as fast as expected. REVERSED Should be: "
                    + velocityTargetMPS
                    + " is: "
                    + velocityMeasuredMPS);
      }
    }
  }

  private Command getSwerveCheckCommand(SwerveCheckTypes type) {

    double xVelocityMPS;
    double yVelocityMPS;
    double rotationalVelocityRadiansPerSecond;

    double angleTargetRot;
    double velocityTargetMPS;

    switch (type) {
      case LEFT:
        xVelocityMPS = 0.0;
        yVelocityMPS = 1.0;
        rotationalVelocityRadiansPerSecond = 0.0;
        velocityTargetMPS = 1.0;
        angleTargetRot = Units.degreesToRotations(90.0);
        break;
      case RIGHT:
        xVelocityMPS = 0.0;
        yVelocityMPS = -1.0;
        rotationalVelocityRadiansPerSecond = 0.0;
        velocityTargetMPS = -1.0;
        angleTargetRot = Units.degreesToRotations(90.0);
        break;
      case FORWARD:
        xVelocityMPS = 1.0;
        yVelocityMPS = 0.0;
        rotationalVelocityRadiansPerSecond = 0.0;
        velocityTargetMPS = 1.0;
        angleTargetRot = Units.degreesToRotations(0.0);
        break;
      case BACKWARD:
        xVelocityMPS = -1.0;
        yVelocityMPS = 0.0;
        rotationalVelocityRadiansPerSecond = 0.0;
        velocityTargetMPS = -1.0;
        angleTargetRot = Units.degreesToRotations(0.0);
        break;
      case CLOCKWISE:
        return Commands.parallel(
                Commands.run(() -> this.drive(0.0, 0.0, -Math.PI, false, false), this),
                Commands.waitSeconds(1)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              checkSwerveModule(
                                  0,
                                  Units.degreesToRotations(135.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                  -0.38 * Math.PI,
                                  SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
                              checkSwerveModule(
                                  1,
                                  Units.degreesToRotations(45.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                  -0.38 * Math.PI,
                                  SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
                              checkSwerveModule(
                                  2,
                                  Units.degreesToRotations(45.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                  0.38 * Math.PI,
                                  SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
                              checkSwerveModule(
                                  3,
                                  Units.degreesToRotations(135.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                  0.38 * Math.PI,
                                  SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
                            })))
            .withTimeout(1);
      case COUNTERCLOCKWISE:
        return Commands.parallel(
                Commands.run(() -> this.drive(0.0, 0.0, Math.PI, false, false), this),
                Commands.waitSeconds(1)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              checkSwerveModule(
                                  0,
                                  Units.degreesToRotations(135.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                  0.38 * Math.PI,
                                  SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
                              checkSwerveModule(
                                  1,
                                  Units.degreesToRotations(45.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                  0.38 * Math.PI,
                                  SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
                              checkSwerveModule(
                                  2,
                                  Units.degreesToRotations(45.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                  -0.38 * Math.PI,
                                  SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
                              checkSwerveModule(
                                  3,
                                  Units.degreesToRotations(135.0),
                                  SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                  -0.38 * Math.PI,
                                  SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
                            })))
            .withTimeout(1);
      default:
        xVelocityMPS = 0.0;
        yVelocityMPS = 0.0;
        rotationalVelocityRadiansPerSecond = 0.0;
        velocityTargetMPS = 0.0;
        angleTargetRot = Units.degreesToRotations(0.0);
        break;
    }

    return Commands.parallel(
            Commands.run(
                () ->
                    this.drive(
                        xVelocityMPS,
                        yVelocityMPS,
                        rotationalVelocityRadiansPerSecond,
                        false,
                        false),
                this),
            Commands.waitSeconds(1)
                .andThen(
                    Commands.runOnce(
                        () -> {
                          for (int i = 0; i < this.inputs.swerve.length; i++) {
                            checkSwerveModule(
                                i,
                                angleTargetRot,
                                SYSTEM_TEST_ANGLE_TOLERANCE_ROT,
                                velocityTargetMPS,
                                SYSTEM_TEST_VELOCITY_TOLERANCE_MPS);
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
