// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOInputsAutoLogged;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's drivetrain mechanism. It consists of a four MK4 swerve modules,
 * each with two motors and an encoder. It also consists of a Pigeon which is used to measure the
 * robot's rotation.
 */
public class Drivetrain extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

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

  private final PIDController autoXController =
      new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoYController =
      new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoThetaController =
      new PIDController(autoTurnKp.get(), autoTurnKi.get(), autoTurnKd.get());

  private final double trackwidthMeters = RobotConfig.getInstance().getTrackwidth();
  private final double wheelbaseMeters = RobotConfig.getInstance().getWheelbase();
  private final SwerveDriveKinematics kinematics =
      RobotConfig.getInstance().getSwerveDriveKinematics();

  private final SwerveModule[] swerveModules = new SwerveModule[4]; // FL, FR, BL, BR

  // some of this code is from the SDS example code

  private Translation2d centerGravity;

  private final SwerveModulePosition[] swerveModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveModulePosition[] prevSwerveModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Pose2d estimatedPoseWithoutGyro = new Pose2d();

  private boolean isFieldRelative;

  private double gyroOffset;

  private ChassisSpeeds chassisSpeeds;

  private static final String SUBSYSTEM_NAME = "Drivetrain";
  private static final boolean TESTING = false;
  private static final boolean DEBUGGING = false;

  private final SwerveDrivePoseEstimator poseEstimator;
  private boolean brakeMode;

  private DriveMode driveMode = DriveMode.NORMAL;
  private double characterizationVoltage = 0.0;

  /** Constructs a new DrivetrainSubsystem object. */
  public Drivetrain(
      GyroIO gyroIO,
      SwerveModule flModule,
      SwerveModule frModule,
      SwerveModule blModule,
      SwerveModule brModule) {
    this.gyroIO = gyroIO;
    this.swerveModules[0] = flModule;
    this.swerveModules[1] = frModule;
    this.swerveModules[2] = blModule;
    this.swerveModules[3] = brModule;

    this.autoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.centerGravity = new Translation2d(); // default to (0,0)

    this.zeroGyroscope();

    this.isFieldRelative = false;

    this.gyroOffset = 0;

    this.chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    ShuffleboardTab tabMain = Shuffleboard.getTab("MAIN");
    tabMain.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
    tabMain.addBoolean("X-Stance On?", this::isXstance);
    tabMain.addBoolean("Field-Relative Enabled?", () -> this.isFieldRelative);

    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
      tab.addNumber("vx", this::getVelocityX);
      tab.addNumber("vy", this::getVelocityY);
      tab.addNumber("Pose Est X", () -> poseEstimator.getEstimatedPosition().getX());
      tab.addNumber("Pose Est Y", () -> poseEstimator.getEstimatedPosition().getY());
      tab.addNumber(
          "Pose Est Rot", () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees());
      tab.addNumber("CoG X", () -> this.centerGravity.getX());
      tab.addNumber("CoG Y", () -> this.centerGravity.getY());
    }

    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add("Enable XStance", new InstantCommand(this::enableXstance));
      tab.add("Disable XStance", new InstantCommand(this::disableXstance));
    }
  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment beteween the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */
  public void zeroGyroscope() {
    setGyroOffset(0.0);
  }

  /**
   * Returns the rotation of the robot. Zero degrees is facing away from the driver station; CCW is
   * positive. This method should always be invoked instead of obtaining the yaw directly from the
   * Pigeon as the local offset needs to be added. If the gyro is not connected, the rotation from
   * the estimated pose is returned.
   *
   * @return the rotation of the robot
   */
  private Rotation2d getRotation() {
    if (gyroInputs.connected) {
      return Rotation2d.fromDegrees(gyroInputs.positionDeg + this.gyroOffset);
    } else {
      return estimatedPoseWithoutGyro.getRotation();
    }
  }

  /**
   * Sets the rotation of the robot to the specified value. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). Zero degrees is
   * facing away from the driver station; CCW is positive.
   *
   * @param expectedYaw the rotation of the robot (in degrees)
   */
  public void setGyroOffset(double expectedYaw) {
    // There is a delay between setting the yaw on the Pigeon and that change
    //      taking effect. As a result, it is recommended to never set the yaw and
    //      adjust the local offset instead.
    if (gyroInputs.connected) {
      this.gyroOffset = expectedYaw - gyroInputs.positionDeg;
    } else {
      this.gyroOffset = 0;
      this.estimatedPoseWithoutGyro =
          new Pose2d(
              estimatedPoseWithoutGyro.getX(),
              estimatedPoseWithoutGyro.getY(),
              Rotation2d.fromDegrees(expectedYaw));
    }
  }

  /**
   * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
   * rotation). The origin of the field to the lower left corner (i.e., the corner of the field to
   * the driver's right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @return the pose of the robot
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Sets the odometry of the robot to the specified PathPlanner state. This method should only be
   * invoked when the rotation of the robot is known (e.g., at the start of an autonomous path). The
   * origin of the field to the lower left corner (i.e., the corner of the field to the driver's
   * right). Zero degrees is away from the driver and increases in the CCW direction.
   *
   * @param state the specified PathPlanner state to which is set the odometry
   */
  public void resetOdometry(PathPlannerState state) {
    setGyroOffset(state.holonomicRotation.getDegrees());

    for (int i = 0; i < 4; i++) {
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }

    estimatedPoseWithoutGyro =
        new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
    poseEstimator.resetPosition(
        this.getRotation(),
        swerveModulePositions,
        new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation));
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities may be specified from either the robot's frame of
   * reference of the field's frame of reference. In the robot's frame of reference, the positive x
   * direction is forward; the positive y direction, left; position rotation, CCW. In the field
   * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
   * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
   * direction.
   *
   * <p>If the drive mode is XSTANCE, the robot will ignore the specified velocities and turn the
   * swerve modules into the x-stance orientation.
   *
   * <p>If the drive mode is CHARACTERIZATION, the robot will ignore the specified velocities and
   * run the characterization routine.
   *
   * @param translationXSupplier the desired velocity in the x direction (m/s)
   * @param translationYSupplier the desired velocity in the y direction (m/s)
   * @param rotationSupplier the desired rotational velcoity (rad/s)
   */
  public void drive(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {

    switch (driveMode) {
      case NORMAL:
        if (isFieldRelative) {
          chassisSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xVelocity, yVelocity, rotationalVelocity, getRotation());

        } else {
          chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);
        }

        Logger.getInstance()
            .recordOutput("Drivetrain/chassisSpeedVx", chassisSpeeds.vxMetersPerSecond);
        Logger.getInstance()
            .recordOutput("Drivetrain/chassisSpeedVy", chassisSpeeds.vyMetersPerSecond);
        Logger.getInstance()
            .recordOutput("Drivetrain/chassisSpeedVo", chassisSpeeds.omegaRadiansPerSecond);

        SwerveModuleState[] swerveModuleStates =
            kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, RobotConfig.getInstance().getRobotMaxVelocity());

        for (SwerveModule swerveModule : swerveModules) {
          swerveModule.setDesiredState(
              swerveModuleStates[swerveModule.getModuleNumber()], isOpenLoop, false);
        }
        break;

      case CHARACTERIZATION:
        // In characterization mode, drive at the specified voltage (and turn to zero degrees)
        for (SwerveModule swerveModule : swerveModules) {
          swerveModule.setVoltageForCharacterization(characterizationVoltage);
        }
        break;

      case X:
        this.setXStance();

        break;
    }
  }

  /**
   * Stops the motion of the robot. Since the motors are in break mode, the robot will stop soon
   * after this method is invoked.
   */
  public void stop() {
    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
    setSwerveModuleStates(states);
  }

  /**
   * This method is invoked each iteration of the scheduler. Typically, when using a command-based
   * model, subsystems don't override the periodic method. However, the drivetrain needs to
   * continually update the odometry of the robot, update and log the gyro and swerve module inputs,
   * update brake mode, and update the tunable values.
   */
  @Override
  public void periodic() {

    // update and log gyro inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

    // update and log the swerve moudles inputs
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.updateAndProcessInputs();
    }

    // update estimated poses
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = swerveModules[i].getState();
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }

    // if the gyro is not connected, use the swerve module positions to estimate the robot's
    // rotation
    if (!gyroInputs.connected) {
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int index = 0; index < moduleDeltas.length; index++) {
        SwerveModulePosition current = swerveModulePositions[index];
        SwerveModulePosition previous = prevSwerveModulePositions[index];

        moduleDeltas[index] =
            new SwerveModulePosition(
                current.distanceMeters - previous.distanceMeters, current.angle);
        previous.distanceMeters = current.distanceMeters;
      }

      Twist2d twist = kinematics.toTwist2d(moduleDeltas);

      estimatedPoseWithoutGyro = estimatedPoseWithoutGyro.exp(twist);
    }

    poseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), this.getRotation(), swerveModulePositions);

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

    // log poses, 3D geometry, and swerve module states, gyro offset
    Pose2d poseEstimatorPose = poseEstimator.getEstimatedPosition();
    Logger.getInstance().recordOutput("Odometry/RobotNoGyro", estimatedPoseWithoutGyro);
    Logger.getInstance().recordOutput("Odometry/Robot", poseEstimatorPose);
    Logger.getInstance().recordOutput("3DField", new Pose3d(poseEstimatorPose));
    Logger.getInstance().recordOutput("SwerveModuleStates", states);
    Logger.getInstance().recordOutput(SUBSYSTEM_NAME + "/gyroOffset", this.gyroOffset);
  }

  /**
   * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
   * stopped moving, and brake mode is enabled, disable it.
   */
  private void updateBrakeMode() {
    if (DriverStation.isEnabled() && !brakeMode) {
      brakeMode = true;
      setBrakeMode(true);

    } else {
      boolean stillMoving = false;
      for (SwerveModule mod : swerveModules) {
        if (Math.abs(mod.getState().speedMetersPerSecond)
            > RobotConfig.getInstance().getRobotMaxCoastVelocity()) {
          stillMoving = true;
        }
      }

      if (brakeMode && !stillMoving) {
        brakeMode = false;
        setBrakeMode(false);
      }
    }
  }

  private void setBrakeMode(boolean enable) {
    for (SwerveModule mod : swerveModules) {
      mod.setAngleBrakeMode(enable);
      mod.setDriveBrakeMode(enable);
    }
  }

  /**
   * Sets each of the swerve modules based on the specified corresponding swerve module state.
   * Incorporates the configured feedforward when setting each swerve module. The order of the
   * states in the array must be front left, front right, back left, back right.
   *
   * <p>This method is invoked by the FollowPath autonomous command.
   *
   * @param states the specified swerve module state for each swerve module
   */
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, RobotConfig.getInstance().getRobotMaxVelocity());

    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(states[swerveModule.getModuleNumber()], false, false);
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
   * Sets the swerve modules in the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
   * useful when shooting.
   */
  public void setXStance() {
    chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds, centerGravity);
    states[0].angle = new Rotation2d(Math.PI / 2 - Math.atan(trackwidthMeters / wheelbaseMeters));
    states[1].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackwidthMeters / wheelbaseMeters));
    states[2].angle = new Rotation2d(Math.PI / 2 + Math.atan(trackwidthMeters / wheelbaseMeters));
    states[3].angle =
        new Rotation2d(3.0 / 2.0 * Math.PI - Math.atan(trackwidthMeters / wheelbaseMeters));
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(states[swerveModule.getModuleNumber()], true, true);
    }
  }

  /**
   * Sets the robot's center of gravity about which it will rotate. The origin is at the center of
   * the robot. The positive x direction is forward; the positive y direction, left.
   *
   * @param x the x coordinate of the robot's center of gravity (in meters)
   * @param y the y coordinate of the robot's center of gravity (in meters)
   */
  public void setCenterGrav(double x, double y) {
    this.centerGravity = new Translation2d(x, y);
  }

  /** Resets the robot's center of gravity about which it will rotate to the center of the robot. */
  public void resetCenterGrav() {
    setCenterGrav(0.0, 0.0);
  }

  /**
   * Returns the desired velocity of the drivetrain in the x direction (units of m/s)
   *
   * @return the desired velocity of the drivetrain in the x direction (units of m/s)
   */
  public double getVelocityX() {
    return chassisSpeeds.vxMetersPerSecond;
  }

  /**
   * Returns the desired velocity of the drivetrain in the y direction (units of m/s)
   *
   * @return the desired velocity of the drivetrain in the y direction (units of m/s)
   */
  public double getVelocityY() {
    return chassisSpeeds.vyMetersPerSecond;
  }

  /**
   * Puts the drivetrain into the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This makes it more difficult for other robots to push the robot, which is
   * useful when shooting. The robot cannot be driven until x-stance is disabled.
   */
  public void enableXstance() {
    this.driveMode = DriveMode.X;
    this.setXStance();
  }

  /** Disables the x-stance, allowing the robot to be driven. */
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
   * Returns the PID controller used to control the robot's x position during autonomous.
   *
   * @return the PID controller used to control the robot's x position during autonomous
   */
  public PIDController getAutoXController() {
    return autoXController;
  }

  /**
   * Returns the PID controller used to control the robot's y position during autonomous.
   *
   * @return the PID controller used to control the robot's y position during autonomous
   */
  public PIDController getAutoYController() {
    return autoYController;
  }

  /**
   * Returns the PID controller used to control the robot's rotation during autonomous.
   *
   * @return the PID controller used to control the robot's rotation during autonomous
   */
  public PIDController getAutoThetaController() {
    return autoThetaController;
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    driveMode = DriveMode.CHARACTERIZATION;
    characterizationVoltage = volts;

    // invoke drive which will set the characterization voltage to each module
    drive(0, 0, 0, true);
  }

  /** Returns the average drive velocity in meters/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (SwerveModule swerveModule : swerveModules) {
      driveVelocityAverage += swerveModule.getState().speedMetersPerSecond;
    }
    return driveVelocityAverage / 4.0;
  }

  private enum DriveMode {
    NORMAL,
    X,
    CHARACTERIZATION
  }
}
