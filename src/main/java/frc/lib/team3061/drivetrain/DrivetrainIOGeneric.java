package frc.lib.team3061.drivetrain;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIO;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DrivetrainIOGeneric implements DrivetrainIO {
  private final GyroIO gyroIO;
  private double robotRotationDeg = 0.0;

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

  private final RobotOdometry odometry;
  private Pose2d estimatedPoseWithoutGyro = new Pose2d();

  private final List<BaseStatusSignal> odometrySignals = new ArrayList<>();

  protected static final TunableNumber thetaKp =
      new TunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKp",
          RobotConfig.getInstance().getDriveFacingAngleThetaKP());
  protected static final TunableNumber thetaKi =
      new TunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKi",
          RobotConfig.getInstance().getDriveFacingAngleThetaKI());
  protected static final TunableNumber thetaKd =
      new TunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKd",
          RobotConfig.getInstance().getDriveFacingAngleThetaKD());
  protected static final TunableNumber thetaMaxVelocity =
      new TunableNumber("Drivetrain/DriveFacingAngle/ThetaMaxVelocity", 8);
  protected static final TunableNumber thetaMaxAcceleration =
      new TunableNumber("Drivetrain/DriveFacingAngle/ThetaMaxAcceleration", 100);

  protected final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  protected boolean lastRequestWasDriveFacingAngle = false;

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

    this.odometry = RobotOdometry.getInstance();

    this.odometrySignals.addAll(this.gyroIO.getOdometryStatusSignals());
    for (SwerveModuleIO swerveModule : swerveModules) {
      this.odometrySignals.addAll(swerveModule.getOdometryStatusSignals());
    }
  }

  @Override
  public void updateInputs(DrivetrainIOInputsCollection inputs) {
    // synchronize all of the signals related to pose estimation
    if (RobotConfig.getInstance().getPhoenix6Licensed()) {
      // this is a licensed method
      BaseStatusSignal.waitForAll(
          Constants.LOOP_PERIOD_SECS, this.odometrySignals.toArray(new BaseStatusSignal[0]));
    }

    // update and log gyro inputs
    gyroIO.updateInputs(inputs.gyro);

    // update and log the swerve modules inputs
    for (int i = 0; i < this.swerveModules.length; i++) {
      this.swerveModules[i].updateInputs(inputs.swerve[i]);
    }

    // update swerve module states and positions
    for (int i = 0; i < this.swerveModuleStates.length; i++) {
      prevSwerveModuleStates[i] = swerveModuleStates[i];
      swerveModuleStates[i] =
          new SwerveModuleState(
              inputs.swerve[i].driveVelocityMetersPerSec,
              Rotation2d.fromDegrees(inputs.swerve[i].steerPositionDeg));
      prevSwerveModulePositions[i] = swerveModulePositions[i];
      swerveModulePositions[i] =
          new SwerveModulePosition(
              inputs.swerve[i].driveDistanceMeters,
              Rotation2d.fromDegrees(inputs.swerve[i].steerPositionDeg));
    }

    // if the gyro is not connected, use the swerve module positions to estimate the robot's
    // rotation
    if (!inputs.gyro.connected || Constants.getMode() == Constants.Mode.SIM) {
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int index = 0; index < moduleDeltas.length; index++) {
        SwerveModulePosition current = swerveModulePositions[index];
        SwerveModulePosition previous = prevSwerveModulePositions[index];

        moduleDeltas[index] =
            new SwerveModulePosition(
                current.distanceMeters - previous.distanceMeters, current.angle);
      }

      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      this.gyroIO.addYaw(Math.toDegrees(twist.dtheta));

      estimatedPoseWithoutGyro = estimatedPoseWithoutGyro.exp(twist);
      this.robotRotationDeg = estimatedPoseWithoutGyro.getRotation().getDegrees();
    } else {
      this.robotRotationDeg = inputs.gyro.yawDeg;
    }

    inputs.drivetrain.swerveMeasuredStates = this.swerveModuleStates;
    inputs.drivetrain.swerveReferenceStates = this.swerveReferenceStates;

    inputs.drivetrain.targetVXMetersPerSec = this.targetChassisSpeeds.vxMetersPerSecond;
    inputs.drivetrain.targetVYMetersPerSec = this.targetChassisSpeeds.vyMetersPerSecond;
    inputs.drivetrain.targetAngularVelocityRadPerSec =
        this.targetChassisSpeeds.omegaRadiansPerSecond;

    ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(this.swerveModuleStates);
    inputs.drivetrain.measuredVXMetersPerSec = measuredChassisSpeeds.vxMetersPerSecond;
    inputs.drivetrain.measuredVYMetersPerSec = measuredChassisSpeeds.vyMetersPerSecond;
    inputs.drivetrain.measuredAngularVelocityRadPerSec =
        measuredChassisSpeeds.omegaRadiansPerSecond;

    inputs.drivetrain.averageDriveCurrent = this.getAverageDriveCurrent(inputs);

    inputs.drivetrain.odometryTimestamps = new double[] {Logger.getRealTimestamp() / 1e6};

    if (thetaKp.hasChanged()
        || thetaKd.hasChanged()
        || thetaKi.hasChanged()
        || thetaMaxVelocity.hasChanged()
        || thetaMaxAcceleration.hasChanged()) {
      thetaController.setP(thetaKp.get());
      thetaController.setI(thetaKi.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
    }
  }

  @Override
  public void holdXStance() {
    this.lastRequestWasDriveFacingAngle = false;

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
    this.lastRequestWasDriveFacingAngle = false;

    this.targetChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity,
            yVelocity,
            rotationalVelocity,
            Rotation2d.fromDegrees(this.robotRotationDeg));

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(this.targetChassisSpeeds, Constants.LOOP_PERIOD_SECS);

    this.swerveReferenceStates =
        kinematics.toSwerveModuleStates(this.targetChassisSpeeds, centerOfRotation);
    setSwerveModuleStates(this.swerveReferenceStates, isOpenLoop, false);
  }

  @Override
  public void driveFieldRelativeFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {

    if (!this.lastRequestWasDriveFacingAngle) {

      thetaController.reset(Units.degreesToRadians(this.robotRotationDeg));

      // configure the controller such that the range of values is centered on the target angle
      thetaController.enableContinuousInput(
          targetDirection.getRadians() - Math.PI, targetDirection.getRadians() + Math.PI);

      this.lastRequestWasDriveFacingAngle = true;
    }

    double thetaVelocity =
        thetaController.calculate(
            Units.degreesToRadians(this.robotRotationDeg), targetDirection.getRadians());

    this.targetChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, yVelocity, thetaVelocity, Rotation2d.fromDegrees(this.robotRotationDeg));

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(this.targetChassisSpeeds, Constants.LOOP_PERIOD_SECS);

    this.swerveReferenceStates =
        kinematics.toSwerveModuleStates(this.targetChassisSpeeds, centerOfRotation);
    setSwerveModuleStates(this.swerveReferenceStates, isOpenLoop, false);
  }

  @Override
  public void pointWheelsAt(Rotation2d targetDirection) {
    this.lastRequestWasDriveFacingAngle = false;

    for (int i = 0; i < this.swerveReferenceStates.length; i++) {
      this.swerveReferenceStates[i] = new SwerveModuleState(0.0, targetDirection);
    }

    setSwerveModuleStates(this.swerveReferenceStates, false, true);
  }

  @Override
  public void driveRobotRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {
    this.lastRequestWasDriveFacingAngle = false;

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
    this.gyroIO.setYaw(expectedYaw);
    this.estimatedPoseWithoutGyro =
        new Pose2d(
            estimatedPoseWithoutGyro.getX(),
            estimatedPoseWithoutGyro.getY(),
            Rotation2d.fromDegrees(expectedYaw));
  }

  @Override
  public void setCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  @Override
  public void resetPose(Pose2d pose) {
    setGyroOffset(pose.getRotation().getDegrees());
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
  private double getAverageDriveCurrent(DrivetrainIOInputsCollection inputs) {
    double totalCurrent = 0.0;
    for (SwerveIOInputs swerveInputs : inputs.swerve) {
      totalCurrent += swerveInputs.driveStatorCurrentAmps;
    }
    return totalCurrent / inputs.swerve.length;
  }

  private void setSwerveModuleStates(
      SwerveModuleState[] states, boolean isOpenLoop, boolean forceAngle) {
    double maxVelocity = RobotConfig.getInstance().getRobotMaxVelocity();

    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxVelocity);

    for (int i = 0; i < this.swerveModules.length; i++) {
      states[i].optimize(swerveModuleStates[i].angle);

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
}
