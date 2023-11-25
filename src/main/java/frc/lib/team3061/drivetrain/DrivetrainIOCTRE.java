package frc.lib.team3061.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
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
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.DrivetrainIO.DrivetrainIOInputs;
import frc.lib.team3061.drivetrain.DrivetrainIO.SwerveIOInputs;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIO;
import frc.lib.team3061.gyro.GyroIO.GyroIOInputs;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class DrivetrainIOCTRE extends SwerveDrivetrain implements DrivetrainIO {

  static class CustomSlotGains extends Slot0Configs {
    public CustomSlotGains(double kP, double kI, double kD, double kV, double kS) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kV = kV;
      this.kS = kS;
    }
  }

  static class SwerveModuleSignals {
    public SwerveModuleSignals(
        StatusSignal<Double> angleVelocityStatusSignal,
        StatusSignal<Double> anglePositionErrorStatusSignal,
        StatusSignal<Double> anglePositionReferenceStatusSignal,
        StatusSignal<Double> drivePositionStatusSignal,
        StatusSignal<Double> driveVelocityErrorStatusSignal,
        StatusSignal<Double> driveVelocityReferenceStatusSignal) {
      this.angleVelocityStatusSignal = angleVelocityStatusSignal;
      this.anglePositionErrorStatusSignal = anglePositionErrorStatusSignal;
      this.anglePositionReferenceStatusSignal = anglePositionReferenceStatusSignal;
      this.drivePositionStatusSignal = drivePositionStatusSignal;
      this.driveVelocityErrorStatusSignal = driveVelocityErrorStatusSignal;
      this.driveVelocityReferenceStatusSignal = driveVelocityReferenceStatusSignal;
    }

    StatusSignal<Double> angleVelocityStatusSignal;
    StatusSignal<Double> anglePositionErrorStatusSignal;
    StatusSignal<Double> anglePositionReferenceStatusSignal;
    StatusSignal<Double> drivePositionStatusSignal;
    StatusSignal<Double> driveVelocityErrorStatusSignal;
    StatusSignal<Double> driveVelocityReferenceStatusSignal;
  }

  private final TunableNumber driveKp =
      new TunableNumber("Drive/DriveKp", RobotConfig.getInstance().getSwerveDriveKP());
  private final TunableNumber driveKi =
      new TunableNumber("Drive/DriveKi", RobotConfig.getInstance().getSwerveDriveKI());
  private final TunableNumber driveKd =
      new TunableNumber("Drive/DriveKd", RobotConfig.getInstance().getSwerveDriveKD());
  private final TunableNumber steerKp =
      new TunableNumber("Drive/TurnKp", RobotConfig.getInstance().getSwerveAngleKP());
  private final TunableNumber steerKi =
      new TunableNumber("Drive/TurnKi", RobotConfig.getInstance().getSwerveAngleKI());
  private final TunableNumber steerKd =
      new TunableNumber("Drive/TurnKd", RobotConfig.getInstance().getSwerveAngleKD());

  private static final CustomSlotGains steerGains =
      new CustomSlotGains(
          RobotConfig.getInstance().getSwerveAngleKP(),
          RobotConfig.getInstance().getSwerveAngleKI(),
          RobotConfig.getInstance().getSwerveAngleKD(),
          RobotConfig.getInstance().getSwerveAngleKV(),
          RobotConfig.getInstance().getSwerveAngleKS());
  private static final CustomSlotGains driveGains =
      new CustomSlotGains(
          RobotConfig.getInstance().getSwerveDriveKP(),
          RobotConfig.getInstance().getSwerveDriveKI(),
          RobotConfig.getInstance().getSwerveDriveKD(),
          RobotConfig.getInstance().getDriveKV(),
          RobotConfig.getInstance().getDriveKS());

  private static final double COUPLE_RATIO = 0.0;
  private static final double STEER_INERTIA = 0.00001;
  private static final double DRIVE_INERTIA = 0.001;

  private static final SwerveDrivetrainConstants drivetrainConstants =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(RobotConfig.getInstance().getGyroCANID())
          .withCANbusName(RobotConfig.getInstance().getCANBusName());

  private static final SwerveModuleConstantsFactory constantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(SwerveConstants.MK4I_L2_DRIVE_GEAR_RATIO)
          .withSteerMotorGearRatio(SwerveConstants.MK4I_L2_ANGLE_GEAR_RATIO)
          .withWheelRadius(
              Units.metersToInches(SwerveConstants.MK4I_L2_WHEEL_DIAMETER_METERS / 2.0))
          .withSlipCurrent(800)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSpeedAt12VoltsMps(RobotConfig.getInstance().getRobotMaxVelocity())
          .withSteerInertia(STEER_INERTIA)
          .withDriveInertia(DRIVE_INERTIA)
          .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(
              COUPLE_RATIO) // Every 1 rotation of the azimuth results in couple ratio drive turns
          .withSteerMotorInverted(SwerveConstants.MK4I_L2_ANGLE_MOTOR_INVERTED);

  private static final SwerveModuleConstants frontLeft =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[0],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[0],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[0],
          RobotConfig.getInstance().getSwerveSteerOffsets()[0],
          RobotConfig.getInstance().getWheelbase() / 2.0,
          RobotConfig.getInstance().getTrackwidth() / 2.0,
          !SwerveConstants.MK4I_L2_DRIVE_MOTOR_INVERTED);
  private static final SwerveModuleConstants frontRight =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[1],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[1],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[1],
          RobotConfig.getInstance().getSwerveSteerOffsets()[1],
          RobotConfig.getInstance().getWheelbase() / 2.0,
          -RobotConfig.getInstance().getTrackwidth() / 2.0,
          SwerveConstants.MK4I_L2_DRIVE_MOTOR_INVERTED);
  private static final SwerveModuleConstants backLeft =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[2],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[2],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[2],
          RobotConfig.getInstance().getSwerveSteerOffsets()[2],
          -RobotConfig.getInstance().getWheelbase() / 2.0,
          RobotConfig.getInstance().getTrackwidth() / 2.0,
          !SwerveConstants.MK4I_L2_DRIVE_MOTOR_INVERTED);
  private static final SwerveModuleConstants backRight =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[3],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[3],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[3],
          RobotConfig.getInstance().getSwerveSteerOffsets()[3],
          -RobotConfig.getInstance().getWheelbase() / 2.0,
          -RobotConfig.getInstance().getTrackwidth() / 2.0,
          SwerveConstants.MK4I_L2_DRIVE_MOTOR_INVERTED);

  // gyro signals
  private final StatusSignal<Double> pitchStatusSignal;
  private final StatusSignal<Double> rollStatusSignal;
  private final StatusSignal<Double> angularVelocityXStatusSignal;
  private final StatusSignal<Double> angularVelocityYStatusSignal;

  // swerve module signals
  SwerveModuleSignals swerveModulesSignals[] = new SwerveModuleSignals[4];

  private double robotRotationDeg = 0.0;

  private final double trackwidthMeters = RobotConfig.getInstance().getTrackwidth();
  private final double wheelbaseMeters = RobotConfig.getInstance().getWheelbase();
  private final SwerveDriveKinematics kinematics =
      RobotConfig.getInstance().getSwerveDriveKinematics();

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

  /**
   * Creates a new Drivetrain subsystem.
   *
   * @param gyroIO the abstracted interface for the gyro for the drivetrain
   * @param flModule the front left swerve module
   * @param frModule the front right swerve module
   * @param blModule the back left swerve module
   * @param brModule the back right swerve module
   */
  public DrivetrainIOCTRE() {
    super(
        drivetrainConstants,
        RobotConfig.getInstance().getOdometryUpdateFrequency(),
        frontLeft,
        frontRight,
        backLeft,
        backRight);

    this.pitchStatusSignal = this.m_pigeon2.getPitch();
    this.pitchStatusSignal.setUpdateFrequency(100);
    this.rollStatusSignal = this.m_pigeon2.getRoll();
    this.rollStatusSignal.setUpdateFrequency(100);
    this.angularVelocityXStatusSignal = this.m_pigeon2.getAngularVelocityX();
    this.angularVelocityXStatusSignal.setUpdateFrequency(100);
    this.angularVelocityYStatusSignal = this.m_pigeon2.getAngularVelocityY();
    this.angularVelocityYStatusSignal.setUpdateFrequency(100);

    for (int i = 0; i < swerveModulesSignals.length; i++) {
      swerveModulesSignals[i] =
          new SwerveModuleSignals(
              this.Modules[i].getSteerMotor().getVelocity(),
              this.Modules[i].getSteerMotor().getClosedLoopError(),
              this.Modules[i].getSteerMotor().getClosedLoopReference(),
              this.Modules[i].getDriveMotor().getPosition(),
              this.Modules[i].getDriveMotor().getClosedLoopError(),
              this.Modules[i].getDriveMotor().getClosedLoopReference());
    }

    this.centerOfRotation = new Translation2d(); // default to (0,0)

    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.poseEstimator = RobotOdometry.getInstance().getPoseEstimator();
  }

  @Override
  public void updateInputs(DrivetrainIOInputs inputs) {

    // update and log gyro inputs
    this.updateGyroInputs(inputs.gyro);

    // update and log the swerve modules inputs
    for (int i = 0; i < swerveModulesSignals.length; i++) {
      this.updateSwerveModuleInputs(inputs.swerveInputs[i], swerveModulesSignals[i]);
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
    if (!inputs.gyro.connected || Constants.getMode() == Constants.Mode.SIM) {
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
      this.robotRotationDeg = estimatedPoseWithoutGyro.getRotation().getDegrees();
    } else {
      this.robotRotationDeg = inputs.gyro.yawDeg;
    }

    inputs.swerveMeasuredStates = this.swerveModuleStates;
    inputs.swerveReferenceStates = this.swerveReferenceStates;

    // update the pose estimator based on the gyro and swerve module positions
    this.poseEstimator.updateWithTime(
        Logger.getRealTimestamp() / 1e6,
        Rotation2d.fromDegrees(this.robotRotationDeg),
        swerveModulePositions);

    // log poses, 3D geometry, and swerve module states, gyro offset
    inputs.robotPoseWithoutGyro = estimatedPoseWithoutGyro;
    inputs.robotPose = poseEstimator.getEstimatedPosition();
    inputs.robotPose3D = new Pose3d(inputs.robotPose);

    inputs.targetVXMetersPerSec = this.targetChassisSpeeds.vxMetersPerSecond;
    inputs.targetVYMetersPerSec = this.targetChassisSpeeds.vyMetersPerSecond;
    inputs.targetAngularVelocityRadPerSec = this.targetChassisSpeeds.omegaRadiansPerSecond;

    ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(this.swerveModuleStates);
    inputs.measuredVXMetersPerSec = measuredChassisSpeeds.vxMetersPerSecond;
    inputs.measuredVYMetersPerSec = measuredChassisSpeeds.vyMetersPerSecond;
    inputs.measuredAngularVelocityRadPerSec = measuredChassisSpeeds.omegaRadiansPerSecond;

    inputs.averageDriveCurrent = this.getAverageDriveCurrent(inputs);

    inputs.rotation = Rotation2d.fromDegrees(this.robotRotationDeg);
  }

  private void updateGyroInputs(GyroIOInputs inputs) {
    this.pitchStatusSignal.refresh();
    this.rollStatusSignal.refresh();
    this.angularVelocityXStatusSignal.refresh();
    this.angularVelocityYStatusSignal.refresh();

    inputs.connected = (this.m_yawGetter.getStatus() == StatusCode.OK);
    inputs.yawDeg =
        BaseStatusSignal.getLatencyCompensatedValue(this.m_yawGetter, this.m_angularZGetter);
    inputs.pitchDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.pitchStatusSignal, this.angularVelocityYStatusSignal);
    inputs.rollDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.rollStatusSignal, this.angularVelocityXStatusSignal);
    inputs.rollDegPerSec = this.angularVelocityXStatusSignal.getValue();
    inputs.pitchDegPerSec = this.angularVelocityYStatusSignal.getValue();
    inputs.yawDegPerSec = this.m_angularZGetter.getValue();
  }

  private void updateSwerveModuleInputs(SwerveIOInputs inputs, SwerveModuleSignals signals) {

    anglePositionErrorStatusSignal.refresh();
    anglePositionReferenceStatusSignal.refresh();
    driveVelocityErrorStatusSignal.refresh();
    driveVelocityReferenceStatusSignal.refresh();

    inputs.driveDistanceMeters =
        Conversions.falconRotationsToMechanismMeters(
            BaseStatusSignal.getLatencyCompensatedValue(
                drivePositionStatusSignal, driveVelocityStatusSignal),
            wheelCircumference,
            driveGearRatio);
    inputs.driveVelocityMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            driveVelocityStatusSignal.getValue(), wheelCircumference, driveGearRatio);
    inputs.driveVelocityReferenceMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            driveVelocityReferenceStatusSignal.getValue(), wheelCircumference, driveGearRatio);
    inputs.driveVelocityErrorMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            driveVelocityErrorStatusSignal.getValue(), wheelCircumference, driveGearRatio);
    inputs.driveAppliedVolts = this.driveMotor.getMotorVoltage().getValue();
    inputs.driveStatorCurrentAmps = this.driveMotor.getStatorCurrent().getValue();
    inputs.driveSupplyCurrentAmps = this.driveMotor.getSupplyCurrent().getValue();
    inputs.driveTempCelsius = this.driveMotor.getDeviceTemp().getValue();

    inputs.steerAbsolutePositionDeg = this.angleEncoder.getAbsolutePosition().getValue() * 360.0;
    // since we are using the FusedCANcoder feature, the position and velocity signal for the angle
    // motor accounts for the gear ratio; so, pass a gear ratio of 1 to just convert from rotations
    // to degrees.
    inputs.steerPositionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            BaseStatusSignal.getLatencyCompensatedValue(
                anglePositionStatusSignal, angleVelocityStatusSignal),
            1);
    inputs.steerPositionReferenceDeg =
        Conversions.falconRotationsToMechanismDegrees(
            anglePositionReferenceStatusSignal.getValue(), 1);
    inputs.steerPositionErrorDeg =
        Conversions.falconRotationsToMechanismDegrees(anglePositionErrorStatusSignal.getValue(), 1);
    inputs.steerVelocityRevPerMin =
        Conversions.falconRPSToMechanismRPM(angleVelocityStatusSignal.getValue(), 1);

    inputs.steerAppliedVolts = this.angleMotor.getMotorVoltage().getValue();
    inputs.steerStatorCurrentAmps = this.angleMotor.getStatorCurrent().getValue();
    inputs.steerSupplyCurrentAmps = this.angleMotor.getSupplyCurrent().getValue();
    inputs.steerTempCelsius = this.angleMotor.getDeviceTemp().getValue();
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
    // FIXME: add support for holding a rotation angle
    this.targetChassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVelocity, yVelocity, 0.0, Rotation2d.fromDegrees(this.robotRotationDeg));

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
    this.estimatedPoseWithoutGyro = new Pose2d(pose.getTranslation(), pose.getRotation());
    this.poseEstimator.resetPosition(
        Rotation2d.fromDegrees(this.robotRotationDeg), swerveModulePositions, pose);
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
}
