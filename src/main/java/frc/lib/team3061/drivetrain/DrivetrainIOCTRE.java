package frc.lib.team3061.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;
import frc.lib.team3061.gyro.GyroIO.GyroIOInputs;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;

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
    public SwerveModuleSignals(TalonFX driveMotor, TalonFX steerMotor) {
      this.steerVelocityStatusSignal = steerMotor.getVelocity().clone();
      this.steerPositionErrorStatusSignal = steerMotor.getClosedLoopError().clone();
      this.steerPositionReferenceStatusSignal = steerMotor.getClosedLoopReference().clone();
      this.drivePositionStatusSignal = driveMotor.getPosition().clone();
      this.driveVelocityErrorStatusSignal = driveMotor.getClosedLoopError().clone();
      this.driveVelocityReferenceStatusSignal = driveMotor.getClosedLoopReference().clone();
    }

    StatusSignal<Double> steerVelocityStatusSignal;
    StatusSignal<Double> steerPositionErrorStatusSignal;
    StatusSignal<Double> steerPositionReferenceStatusSignal;
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
  SwerveModuleSignals[] swerveModulesSignals = new SwerveModuleSignals[4];

  private Translation2d centerOfRotation;
  private ChassisSpeeds targetChassisSpeeds;
  private SwerveModuleState[] swerveReferenceStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private SwerveRequest.RobotCentric driveRobotCentricRequest = new SwerveRequest.RobotCentric();
  private SwerveRequest.FieldCentric driveFieldCentricRequest = new SwerveRequest.FieldCentric();
  private SwerveRequest.FieldCentricFacingAngle driveFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle();
  private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
  private SwerveRequest.ApplyChassisSpeeds applyChassisSpeedsRequest =
      new SwerveRequest.ApplyChassisSpeeds();

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

    // configure current limits
    for (SwerveModule swerveModule : this.Modules) {
      CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
      swerveModule.getDriveMotor().getConfigurator().refresh(currentLimits);
      currentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
      currentLimits.SupplyCurrentThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
      currentLimits.SupplyTimeThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;
      currentLimits.SupplyCurrentLimitEnable = SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
      swerveModule.getDriveMotor().getConfigurator().apply(currentLimits);

      currentLimits = new CurrentLimitsConfigs();
      swerveModule.getSteerMotor().getConfigurator().refresh(currentLimits);
      currentLimits.SupplyCurrentLimit = SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT;
      currentLimits.SupplyCurrentThreshold = SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT;
      currentLimits.SupplyTimeThreshold = SwerveConstants.ANGLE_PEAK_CURRENT_DURATION;
      currentLimits.SupplyCurrentLimitEnable = SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT;
      swerveModule.getSteerMotor().getConfigurator().apply(currentLimits);
    }

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
          new SwerveModuleSignals(this.Modules[i].getSteerMotor(), this.Modules[i].getDriveMotor());
    }

    this.centerOfRotation = new Translation2d(); // default to (0,0)

    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // replace the SwerveDrivePoseEstimator created by SwerveDrivetrain with our singleton
    this.m_odometry = RobotOdometry.getInstance().getPoseEstimator();
  }

  @Override
  public void updateInputs(DrivetrainIOInputs inputs) {

    // update and log gyro inputs
    this.updateGyroInputs(inputs.gyro);

    // update and log the swerve modules inputs
    for (int i = 0; i < swerveModulesSignals.length; i++) {
      this.updateSwerveModuleInputs(
          inputs.swerveInputs[i], this.Modules[i], swerveModulesSignals[i]);
    }

    inputs.swerveMeasuredStates = this.getState().ModuleStates;
    inputs.swerveReferenceStates = swerveReferenceStates;

    // log poses, 3D geometry, and swerve module states, gyro offset
    inputs.robotPose = this.getState().Pose;
    inputs.robotPose3D = new Pose3d(this.getState().Pose);

    inputs.targetVXMetersPerSec = this.targetChassisSpeeds.vxMetersPerSecond;
    inputs.targetVYMetersPerSec = this.targetChassisSpeeds.vyMetersPerSecond;
    inputs.targetAngularVelocityRadPerSec = this.targetChassisSpeeds.omegaRadiansPerSecond;

    ChassisSpeeds measuredChassisSpeeds =
        m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    inputs.measuredVXMetersPerSec = measuredChassisSpeeds.vxMetersPerSecond;
    inputs.measuredVYMetersPerSec = measuredChassisSpeeds.vyMetersPerSecond;
    inputs.measuredAngularVelocityRadPerSec = measuredChassisSpeeds.omegaRadiansPerSecond;

    inputs.averageDriveCurrent = this.getAverageDriveCurrent(inputs);

    inputs.rotation = this.getState().Pose.getRotation();

    if (Constants.getMode() == Constants.Mode.SIM) {
      updateSimState(Constants.LOOP_PERIOD_SECS, 12.0);
    }

    // update tunables
    if (driveKp.hasChanged()
        || driveKi.hasChanged()
        || driveKd.hasChanged()
        || steerKp.hasChanged()
        || steerKi.hasChanged()
        || steerKd.hasChanged()) {
      for (SwerveModule swerveModule : this.Modules) {
        Slot0Configs driveSlot0 = new Slot0Configs();
        swerveModule.getDriveMotor().getConfigurator().refresh(driveSlot0);
        driveSlot0.kP = driveKp.get();
        driveSlot0.kI = driveKi.get();
        driveSlot0.kD = driveKd.get();
        swerveModule.getDriveMotor().getConfigurator().apply(driveSlot0);

        Slot0Configs steerSlot0 = new Slot0Configs();
        swerveModule.getSteerMotor().getConfigurator().refresh(steerSlot0);
        steerSlot0.kP = steerKp.get();
        steerSlot0.kI = steerKi.get();
        steerSlot0.kD = steerKd.get();
        swerveModule.getSteerMotor().getConfigurator().apply(steerSlot0);
      }
    }
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

  private void updateSwerveModuleInputs(
      SwerveIOInputs inputs, SwerveModule module, SwerveModuleSignals signals) {

    signals.steerVelocityStatusSignal.refresh();
    signals.steerPositionErrorStatusSignal.refresh();
    signals.steerPositionReferenceStatusSignal.refresh();
    signals.drivePositionStatusSignal.refresh();
    signals.driveVelocityErrorStatusSignal.refresh();
    signals.driveVelocityReferenceStatusSignal.refresh();

    SwerveModulePosition position = module.getPosition(false);
    SwerveModuleState state = module.getCurrentState();

    inputs.driveDistanceMeters = position.distanceMeters;
    inputs.driveVelocityMetersPerSec = state.speedMetersPerSecond;
    inputs.driveVelocityReferenceMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            signals.driveVelocityReferenceStatusSignal.getValue(),
            SwerveConstants.MK4I_L2_WHEEL_CIRCUMFERENCE,
            SwerveConstants.MK4I_L2_DRIVE_GEAR_RATIO);
    inputs.driveVelocityErrorMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            signals.driveVelocityErrorStatusSignal.getValue(),
            SwerveConstants.MK4I_L2_WHEEL_CIRCUMFERENCE,
            SwerveConstants.MK4I_L2_DRIVE_GEAR_RATIO);
    inputs.driveAppliedVolts = module.getDriveMotor().getMotorVoltage().getValue();
    inputs.driveStatorCurrentAmps = module.getDriveMotor().getStatorCurrent().getValue();
    inputs.driveSupplyCurrentAmps = module.getDriveMotor().getSupplyCurrent().getValue();
    inputs.driveTempCelsius = module.getDriveMotor().getDeviceTemp().getValue();

    inputs.steerAbsolutePositionDeg = module.getCANcoder().getAbsolutePosition().getValue() * 360.0;

    // since we are using the FusedCANcoder feature, the position and velocity signal for the angle
    // motor accounts for the gear ratio; so, pass a gear ratio of 1 to just convert from rotations
    // to degrees.
    inputs.steerPositionDeg = position.angle.getDegrees();
    inputs.steerPositionReferenceDeg =
        Conversions.falconRotationsToMechanismDegrees(
            signals.steerPositionReferenceStatusSignal.getValue(), 1);
    inputs.steerPositionErrorDeg =
        Conversions.falconRotationsToMechanismDegrees(
            signals.steerPositionErrorStatusSignal.getValue(), 1);
    inputs.steerVelocityRevPerMin =
        Conversions.falconRPSToMechanismRPM(signals.steerVelocityStatusSignal.getValue(), 1);

    inputs.steerAppliedVolts = module.getSteerMotor().getMotorVoltage().getValue();
    inputs.steerStatorCurrentAmps = module.getSteerMotor().getStatorCurrent().getValue();
    inputs.steerSupplyCurrentAmps = module.getSteerMotor().getSupplyCurrent().getValue();
    inputs.steerTempCelsius = module.getSteerMotor().getDeviceTemp().getValue();
  }

  @Override
  public void holdXStance() {
    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    this.setControl(this.brakeRequest);
  }

  @Override
  public void driveFieldRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, yVelocity, rotationalVelocity, this.getState().Pose.getRotation()),
            Constants.LOOP_PERIOD_SECS);

    this.setControl(
        this.driveFieldCentricRequest
            .withIsOpenLoop(isOpenLoop)
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationalVelocity));
  }

  @Override
  public void driveFieldRelativeFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {
    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, yVelocity, 0.0, getState().Pose.getRotation()),
            Constants.LOOP_PERIOD_SECS);

    this.setControl(
        this.driveFacingAngleRequest
            .withIsOpenLoop(isOpenLoop)
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withTargetDirection(targetDirection));
  }

  @Override
  public void pointWheelsAt(Rotation2d targetDirection) {
    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.setControl(this.pointRequest.withModuleDirection(targetDirection));
  }

  @Override
  public void driveRobotRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {
    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity),
            Constants.LOOP_PERIOD_SECS);

    this.setControl(
        this.driveRobotCentricRequest
            .withIsOpenLoop(isOpenLoop)
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationalVelocity));
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    this.targetChassisSpeeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    this.targetChassisSpeeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
    this.targetChassisSpeeds.vyMetersPerSecond = speeds.vyMetersPerSecond;

    this.setControl(
        this.applyChassisSpeedsRequest
            .withIsOpenLoop(isOpenLoop)
            .withSpeeds(speeds)
            .withCenterOfRotation(this.centerOfRotation));
  }

  @Override
  public void setGyroOffset(double expectedYaw) {
    try {
      m_stateLock.writeLock().lock();

      // FIXME: check the math here not sure if we should add or subtract
      m_fieldRelativeOffset =
          getState().Pose.getRotation().plus(Rotation2d.fromDegrees(expectedYaw));
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  @Override
  public void setCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  @Override
  public void resetPose(Pose2d pose) {
    this.seedFieldRelative(pose);
  }

  @Override
  public void setDriveMotorVoltage(double voltage) {
    // FIXME: implement later with Idle Swerve Request object?
  }

  @Override
  public void setSteerMotorVoltage(double voltage) {
    // FIXME: implement later with Idle Swerve Request object?
  }

  @Override
  public void setBrakeMode(boolean enable) {
    for (SwerveModule swerveModule : this.Modules) {
      MotorOutputConfigs config = new MotorOutputConfigs();
      swerveModule.getDriveMotor().getConfigurator().refresh(config);
      config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      swerveModule.getDriveMotor().getConfigurator().apply(config);
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
}
