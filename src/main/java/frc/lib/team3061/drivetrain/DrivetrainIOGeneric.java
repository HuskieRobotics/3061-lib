package frc.lib.team3061.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOInputsAutoLogged;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DrivetrainIOGeneric implements DrivetrainIO {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final double trackwidthMeters = RobotConfig.getInstance().getTrackwidth();
  private final double wheelbaseMeters = RobotConfig.getInstance().getWheelbase();
  private final SwerveDriveKinematics kinematics =
      RobotConfig.getInstance().getSwerveDriveKinematics();

  private final SwerveModule[] swerveModules = new SwerveModule[4]; // FL, FR, BL, BR

  // some of this code is from the SDS example code

  private Translation2d centerOfRotation;

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
  private final SwerveModuleState[] swerveModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private final SwerveModuleState[] prevSwerveModuleStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  private SwerveModuleState[] swerveReferenceStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private ChassisSpeeds targetChassisSpeeds;

  private final SwerveDrivePoseEstimator poseEstimator;
  private Pose2d estimatedPoseWithoutGyro = new Pose2d();

  private final List<StatusSignal<Double>> odometrySignals = new ArrayList<>();

  /**
   * Creates a new Drivetrain subsystem.
   *
   * @param gyroIO the abstracted interface for the gyro for the drivetrain
   * @param flModule the front left swerve module
   * @param frModule the front right swerve module
   * @param blModule the back left swerve module
   * @param brModule the back right swerve module
   */
  public DrivetrainIOGeneric(
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

    this.centerOfRotation = new Translation2d(); // default to (0,0)

    this.setGyroOffset(0.0);

    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();

    this.odometrySignals.addAll(this.gyroIO.getOdometryStatusSignals());
    for (SwerveModule swerveModule : swerveModules) {
      this.odometrySignals.addAll(swerveModule.getOdometryStatusSignals());
    }
  }

  @Override
  public void updateInputs(DrivetrainIOInputs inputs) {
    // synchronize all of the signals related to pose estimation
    if (RobotConfig.getInstance().getPhoenix6Licensed()) {
      // this is a licensed method
      BaseStatusSignal.waitForAll(
          Constants.LOOP_PERIOD_SECS, this.odometrySignals.toArray(new BaseStatusSignal[0]));
    }

    // update and log gyro inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drivetrain/Gyro", gyroInputs);

    // update and log the swerve modules inputs
    for (SwerveModule swerveModule : swerveModules) {
      // FIXME: invoke updateInputs on the IO class instead
      swerveModule.updateAndProcessInputs();
    }

    // update swerve module states and positions
    for (int i = 0; i < this.swerveModuleStates.length; i++) {
      prevSwerveModuleStates[i] = swerveModuleStates[i];
      swerveModuleStates[i] = swerveModules[i].getState();
      prevSwerveModulePositions[i] = swerveModulePositions[i];
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }

    // if the gyro is not connected, use the swerve module positions to estimate the robot's
    // rotation
    if (!gyroInputs.connected || Constants.getMode() == Constants.Mode.SIM) {
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int index = 0; index < moduleDeltas.length; index++) {
        SwerveModulePosition current = swerveModulePositions[index];
        SwerveModulePosition previous = prevSwerveModulePositions[index];

        moduleDeltas[index] =
            new SwerveModulePosition(
                current.distanceMeters - previous.distanceMeters, current.angle);
        // FIXME: I don't think this assignment is needed...
        previous.distanceMeters = current.distanceMeters;
      }

      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      this.gyroIO.addYaw(Math.toDegrees(twist.dtheta));

      estimatedPoseWithoutGyro = estimatedPoseWithoutGyro.exp(twist);
    }

    inputs.swerveMeasuredStates = this.swerveModuleStates;
    inputs.swerveReferenceStates = this.swerveReferenceStates;

    // update the pose estimator based on the gyro and swerve module positions
    this.poseEstimator.updateWithTime(
        Logger.getRealTimestamp() / 1e6, this.getRotation(), swerveModulePositions);

    // log poses, 3D geometry, and swerve module states, gyro offset
    inputs.robotPoseWithoutGyro = estimatedPoseWithoutGyro;
    inputs.robotPose = poseEstimator.getEstimatedPosition();
    inputs.robotPose3D = new Pose3d(inputs.robotPose);

    inputs.targetChassisSpeeds = this.targetChassisSpeeds;
    inputs.measuredChassisSpeeds = kinematics.toChassisSpeeds(this.swerveModuleStates);

    inputs.averageDriveCurrent = this.getAverageDriveCurrent();

    if (gyroInputs.connected) {
      inputs.rotation = Rotation2d.fromDegrees(gyroInputs.yawDeg);
    } else {
      inputs.rotation = estimatedPoseWithoutGyro.getRotation();
    }
  }

  @Override
  public void holdXStance() {
    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    this.swerveReferenceStates =
        kinematics.toSwerveModuleStates(this.targetChassisSpeeds, this.centerOfRotation);
    this.swerveReferenceStates[0].angle =
        new Rotation2d(Math.PI / 2 - Math.atan(trackwidthMeters / wheelbaseMeters));
    this.swerveReferenceStates[1].angle =
        new Rotation2d(Math.PI / 2 + Math.atan(trackwidthMeters / wheelbaseMeters));
    this.swerveReferenceStates[2].angle =
        new Rotation2d(Math.PI / 2 + Math.atan(trackwidthMeters / wheelbaseMeters));
    this.swerveReferenceStates[3].angle =
        new Rotation2d(3.0 / 2.0 * Math.PI - Math.atan(trackwidthMeters / wheelbaseMeters));
    setSwerveModuleStates(this.swerveReferenceStates, true, true);
  }

  @Override
  public void driveFieldRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {

    this.targetChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, yVelocity, rotationalVelocity, getRotation());

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(this.targetChassisSpeeds, Constants.LOOP_PERIOD_SECS);

    this.swerveReferenceStates =
        kinematics.toSwerveModuleStates(this.targetChassisSpeeds, centerOfRotation);
    setSwerveModuleStates(this.swerveReferenceStates, isOpenLoop, false);
  }

  @Override
  public void driveFieldRelativeFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {
    // FIXME: add support for holding a rotation angle
    this.targetChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, 0.0, getRotation());

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(this.targetChassisSpeeds, Constants.LOOP_PERIOD_SECS);

    this.swerveReferenceStates =
        kinematics.toSwerveModuleStates(this.targetChassisSpeeds, centerOfRotation);
    setSwerveModuleStates(this.swerveReferenceStates, isOpenLoop, false);
  }

  @Override
  public void pointWheelsAt(Rotation2d targetDirection) {
    for (int i = 0; i < this.swerveReferenceStates.length; i++) {
      this.swerveReferenceStates[i] = new SwerveModuleState(0.0, targetDirection);
    }

    setSwerveModuleStates(this.swerveReferenceStates, false, true);
  }

  @Override
  public void driveRobotRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {
    this.targetChassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);
    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(this.targetChassisSpeeds, Constants.LOOP_PERIOD_SECS);

    this.swerveReferenceStates =
        kinematics.toSwerveModuleStates(this.targetChassisSpeeds, centerOfRotation);
    setSwerveModuleStates(this.swerveReferenceStates, isOpenLoop, false);
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    this.targetChassisSpeeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    this.targetChassisSpeeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
    this.targetChassisSpeeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
    this.swerveReferenceStates =
        kinematics.toSwerveModuleStates(this.targetChassisSpeeds, centerOfRotation);
    setSwerveModuleStates(this.swerveReferenceStates, isOpenLoop, false);
  }

  @Override
  public void setGyroOffset(double expectedYaw) {
    if (this.gyroInputs.connected) {
      this.gyroIO.setYaw(expectedYaw);
    } else {
      this.estimatedPoseWithoutGyro =
          new Pose2d(
              estimatedPoseWithoutGyro.getX(),
              estimatedPoseWithoutGyro.getY(),
              Rotation2d.fromDegrees(expectedYaw));
    }
  }

  @Override
  public void setCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  @Override
  public void resetPose(Pose2d pose) {
    setGyroOffset(pose.getRotation().getDegrees());

    for (int i = 0; i < this.swerveModules.length; i++) {
      this.swerveModulePositions[i] = this.swerveModules[i].getPosition();
    }

    this.estimatedPoseWithoutGyro = new Pose2d(pose.getTranslation(), pose.getRotation());
    this.poseEstimator.resetPosition(this.getRotation(), swerveModulePositions, pose);
  }

  @Override
  public void setDriveMotorVoltage(double volts) {
    for (SwerveModule swerveModule : this.swerveModules) {
      swerveModule.setVoltageForDriveCharacterization(volts);
    }
  }

  @Override
  public void setSteerMotorVoltage(double volts) {
    for (SwerveModule swerveModule : this.swerveModules) {
      swerveModule.setVoltageForRotateCharacterization(volts);
    }
  }

  @Override
  public void setBrakeMode(boolean enable) {
    for (SwerveModule mod : this.swerveModules) {
      mod.setAngleBrakeMode(enable);
      mod.setDriveBrakeMode(enable);
    }
  }

  /**
   * Returns the average current of the swerve module drive motors in amps.
   *
   * @return the average current of the swerve module drive motors in amps
   */
  private double getAverageDriveCurrent() {
    double totalCurrent = 0.0;
    for (SwerveModule module : swerveModules) {
      totalCurrent += Math.abs(module.getDriveCurrent());
    }
    return totalCurrent / swerveModules.length;
  }

  private void setSwerveModuleStates(
      SwerveModuleState[] states, boolean isOpenLoop, boolean forceAngle) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, RobotConfig.getInstance().getRobotMaxVelocity());

    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(states[swerveModule.getModuleNumber()], isOpenLoop, forceAngle);
    }
  }

  /**
   * Returns the rotation of the robot. Zero degrees is facing away from the driver station; CCW is
   * positive. If the gyro is not connected, the rotation from the estimated pose is returned.
   *
   * @return the rotation of the robot
   */
  private Rotation2d getRotation() {
    if (gyroInputs.connected) {
      return Rotation2d.fromDegrees(gyroInputs.yawDeg);
    } else {
      return estimatedPoseWithoutGyro.getRotation();
    }
  }
}
