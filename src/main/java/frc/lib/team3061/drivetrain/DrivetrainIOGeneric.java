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
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIO;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOInputsAutoLogged;
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

  private final SwerveModuleIO[] swerveModules = new SwerveModuleIO[4]; // FL, FR, BL, BR

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

  private double[] steerMotorsLastAngle = new double[4];

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
      SwerveModuleIO flModule,
      SwerveModuleIO frModule,
      SwerveModuleIO blModule,
      SwerveModuleIO brModule) {
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
    for (SwerveModuleIO swerveModule : swerveModules) {
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
    for (int i = 0; i < this.swerveModules.length; i++) {
      this.swerveModules[i].updateInputs(inputs.swerveInputs[i]);
    }

    // update swerve module states and positions
    for (int i = 0; i < this.swerveModuleStates.length; i++) {
      prevSwerveModuleStates[i] = swerveModuleStates[i];
      swerveModuleStates[i] =
          new SwerveModuleState(
              inputs.swerveInputs[i].driveVelocityMetersPerSec,
              Rotation2d.fromDegrees(inputs.swerveInputs[i].steerPositionDeg));
      prevSwerveModulePositions[i] = swerveModulePositions[i];
      swerveModulePositions[i] =
          new SwerveModulePosition(
              inputs.swerveInputs[i].driveDistanceMeters,
              Rotation2d.fromDegrees(inputs.swerveInputs[i].steerPositionDeg));
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

    inputs.averageDriveCurrent = this.getAverageDriveCurrent(inputs);

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
    this.estimatedPoseWithoutGyro = new Pose2d(pose.getTranslation(), pose.getRotation());
    this.poseEstimator.resetPosition(this.getRotation(), swerveModulePositions, pose);
  }

  @Override
  public void setDriveMotorVoltage(double voltage) {
    for (int i = 0; i < this.swerveModules.length; i++) {
      this.swerveModules[i].setAnglePosition(0.0);
      steerMotorsLastAngle[i] = 0.0;
      this.swerveModules[i].setDriveMotorVoltage(voltage);
    }
  }

  @Override
  public void setSteerMotorVoltage(double voltage) {
    for (SwerveModuleIO swerveModule : this.swerveModules) {
      swerveModule.setAngleMotorVoltage(voltage);
    }
  }

  @Override
  public void setBrakeMode(boolean enable) {
    for (SwerveModuleIO swerveModule : this.swerveModules) {
      swerveModule.setAngleBrakeMode(enable);
      swerveModule.setDriveBrakeMode(enable);
    }
  }

  /**
   * Returns the average current of the swerve module drive motors in amps.
   *
   * @return the average current of the swerve module drive motors in amps
   */
  private double getAverageDriveCurrent(DrivetrainIOInputs inputs) {
    double totalCurrent = 0.0;
    for (SwerveIOInputs swerveInputs : inputs.swerveInputs) {
      totalCurrent += swerveInputs.driveStatorCurrentAmps;
    }
    return totalCurrent / inputs.swerveInputs.length;
  }

  private void setSwerveModuleStates(
      SwerveModuleState[] states, boolean isOpenLoop, boolean forceAngle) {
    double maxVelocity = RobotConfig.getInstance().getRobotMaxVelocity();

    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);

    for (int i = 0; i < this.swerveModules.length; i++) {
      states[i] = SwerveModuleState.optimize(states[i], swerveModuleStates[i].angle);

      if (isOpenLoop) {
        this.swerveModules[i].setDriveMotorVoltage(
            states[i].speedMetersPerSecond / maxVelocity * 12.0);
      } else {
        this.swerveModules[i].setDriveVelocity(states[i].speedMetersPerSecond);
      }

      // Unless the angle is forced (e.g., X-stance), don't rotate the module if speed is less then
      // 1%. This prevents jittering if the controller isn't tuned perfectly. Perhaps more
      // importantly, it allows for smooth repeated movement as the wheel direction doesn't reset
      // during pauses (e.g., multi-segmented auto paths).
      double angle;
      if (!forceAngle && Math.abs(states[i].speedMetersPerSecond) <= (maxVelocity * 0.01)) {
        angle = this.steerMotorsLastAngle[i];
      } else {
        angle = states[i].angle.getDegrees();
      }

      this.swerveModules[i].setAnglePosition(angle);
      steerMotorsLastAngle[i] = angle;
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
