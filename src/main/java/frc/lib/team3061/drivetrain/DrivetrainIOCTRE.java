package frc.lib.team3061.drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.lib.team3061.drivetrain.DrivetrainConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;
import frc.lib.team3061.gyro.GyroIO.GyroIOInputs;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

@java.lang.SuppressWarnings({"java:S1450"})
public class DrivetrainIOCTRE extends SwerveDrivetrain implements DrivetrainIO {

  static class CustomSlotGains extends Slot0Configs {
    public CustomSlotGains(double kP, double kI, double kD, double kA, double kV, double kS) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kA = kA;
      this.kV = kV;
      this.kS = kS;
    }
  }

  static class SwerveModuleSignals {
    public SwerveModuleSignals(TalonFX driveMotor, TalonFX steerMotor) {
      this.steerVelocityStatusSignal = steerMotor.getVelocity().clone();
      this.steerAccelerationStatusSignal = steerMotor.getAcceleration().clone();
      this.steerPositionErrorStatusSignal = steerMotor.getClosedLoopError().clone();
      this.steerPositionReferenceStatusSignal = steerMotor.getClosedLoopReference().clone();
      this.drivePositionStatusSignal = driveMotor.getPosition().clone();
      this.driveVelocityErrorStatusSignal = driveMotor.getClosedLoopError().clone();
      this.driveVelocityReferenceStatusSignal = driveMotor.getClosedLoopReference().clone();
      this.driveAccelerationStatusSignal = driveMotor.getAcceleration().clone();
    }

    StatusSignal<AngularVelocity> steerVelocityStatusSignal;
    StatusSignal<AngularAcceleration> steerAccelerationStatusSignal;
    StatusSignal<Double> steerPositionErrorStatusSignal;
    StatusSignal<Double> steerPositionReferenceStatusSignal;
    StatusSignal<Angle> drivePositionStatusSignal;
    StatusSignal<Double> driveVelocityErrorStatusSignal;
    StatusSignal<Double> driveVelocityReferenceStatusSignal;
    StatusSignal<AngularAcceleration> driveAccelerationStatusSignal;
  }

  private final LoggedTunableNumber driveKp =
      new LoggedTunableNumber("Drivetrain/DriveKp", RobotConfig.getInstance().getSwerveDriveKP());
  private final LoggedTunableNumber driveKi =
      new LoggedTunableNumber("Drivetrain/DriveKi", RobotConfig.getInstance().getSwerveDriveKI());
  private final LoggedTunableNumber driveKd =
      new LoggedTunableNumber("Drivetrain/DriveKd", RobotConfig.getInstance().getSwerveDriveKD());
  private final LoggedTunableNumber steerKp =
      new LoggedTunableNumber("Drivetrain/TurnKp", RobotConfig.getInstance().getSwerveAngleKP());
  private final LoggedTunableNumber steerKi =
      new LoggedTunableNumber("Drivetrain/TurnKi", RobotConfig.getInstance().getSwerveAngleKI());
  private final LoggedTunableNumber steerKd =
      new LoggedTunableNumber("Drivetrain/TurnKd", RobotConfig.getInstance().getSwerveAngleKD());

  protected static final LoggedTunableNumber driveFacingAngleThetaKp =
      new LoggedTunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKp",
          RobotConfig.getInstance().getDriveFacingAngleThetaKP());
  protected static final LoggedTunableNumber driveFacingAngleThetaKi =
      new LoggedTunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKi",
          RobotConfig.getInstance().getDriveFacingAngleThetaKI());
  protected static final LoggedTunableNumber driveFacingAngleThetaKd =
      new LoggedTunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKd",
          RobotConfig.getInstance().getDriveFacingAngleThetaKD());

  private static final CustomSlotGains steerGains =
      new CustomSlotGains(
          RobotConfig.getInstance().getSwerveAngleKP(),
          RobotConfig.getInstance().getSwerveAngleKI(),
          RobotConfig.getInstance().getSwerveAngleKD(),
          RobotConfig.getInstance().getSwerveAngleKA(),
          RobotConfig.getInstance().getSwerveAngleKV(),
          RobotConfig.getInstance().getSwerveAngleKS());
  private static final CustomSlotGains driveGains =
      new CustomSlotGains(
          RobotConfig.getInstance().getSwerveDriveKP(),
          RobotConfig.getInstance().getSwerveDriveKI(),
          RobotConfig.getInstance().getSwerveDriveKD(),
          RobotConfig.getInstance().getDriveKA(),
          RobotConfig.getInstance().getDriveKV(),
          RobotConfig.getInstance().getDriveKS());

  // The closed-loop output type to use for the steer motors
  // This affects the PID/FF gains for the steer motors
  // TorqueCurrentFOC is not currently supported in simulation.
  private static final ClosedLoopOutputType steerClosedLoopOutput = getSteerClosedLoopOutputType();

  // The closed-loop output type to use for the drive motors
  // This affects the PID/FF gains for the drive motors
  // TorqueCurrentFOC is not currently supported in simulation.
  private static final ClosedLoopOutputType driveClosedLoopOutput = getDriveClosedLoopOutputType();

  private static final double COUPLE_RATIO =
      RobotConfig.getInstance().getAzimuthSteerCouplingRatio();
  private static final double STEER_INERTIA =
      0.00001; // FIXME: test value of 0.01, which is what CTRE Swerve Generator uses
  private static final double DRIVE_INERTIA =
      0.001; // FIXME: test value of 0.01, which is what CTRE Swerve Generator uses

  // Simulated voltage necessary to overcome friction
  private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.25);
  private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.25);

  private static final SwerveDrivetrainConstants drivetrainConstants =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(RobotConfig.getInstance().getGyroCANID())
          .withCANBusName(RobotConfig.getInstance().getCANBusName())
          .withPigeon2Configs(RobotConfig.getInstance().getPigeonConfigForSwerveDrivetrain());

  private static final SwerveModuleConstantsFactory constantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorInitialConfigs(
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT)
                          .withSupplyCurrentLowerLimit(
                              SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT)
                          .withSupplyCurrentLowerTime(SwerveConstants.DRIVE_PEAK_CURRENT_DURATION)
                          .withSupplyCurrentLimitEnable(
                              SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT)))
          .withSteerMotorInitialConfigs(
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT)
                          .withSupplyCurrentLowerLimit(
                              SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT)
                          .withSupplyCurrentLowerTime(SwerveConstants.ANGLE_PEAK_CURRENT_DURATION)
                          .withSupplyCurrentLimitEnable(SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT)
                          .withStatorCurrentLimit(SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT)
                          .withStatorCurrentLimitEnable(
                              SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT)))
          .withDriveMotorGearRatio(
              RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio())
          .withSteerMotorGearRatio(
              RobotConfig.getInstance().getSwerveConstants().getAngleGearRatio())
          .withWheelRadius(Meters.of(RobotConfig.getInstance().getWheelDiameterMeters() / 2.0))
          .withSlipCurrent(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12Volts(MetersPerSecond.of(RobotConfig.getInstance().getRobotMaxVelocity()))
          .withSteerInertia(STEER_INERTIA)
          .withDriveInertia(DRIVE_INERTIA)
          .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
          .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(
              COUPLE_RATIO); // Every 1 rotation of the azimuth results in couple ratio drive turns

  private static final SwerveModuleConstants frontLeft =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[0],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[0],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[0],
          Rotations.of(RobotConfig.getInstance().getSwerveSteerOffsets()[0]),
          Meters.of(RobotConfig.getInstance().getWheelbase() / 2.0),
          Meters.of(RobotConfig.getInstance().getTrackwidth() / 2.0),
          !RobotConfig.getInstance().getSwerveConstants().isDriveMotorInverted(),
          RobotConfig.getInstance().getSwerveConstants().isAngleMotorInverted(),
          false);
  private static final SwerveModuleConstants frontRight =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[1],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[1],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[1],
          Rotations.of(RobotConfig.getInstance().getSwerveSteerOffsets()[1]),
          Meters.of(RobotConfig.getInstance().getWheelbase() / 2.0),
          Meters.of(-RobotConfig.getInstance().getTrackwidth() / 2.0),
          RobotConfig.getInstance().getSwerveConstants().isDriveMotorInverted(),
          RobotConfig.getInstance().getSwerveConstants().isAngleMotorInverted(),
          false);
  private static final SwerveModuleConstants backLeft =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[2],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[2],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[2],
          Rotations.of(RobotConfig.getInstance().getSwerveSteerOffsets()[2]),
          Meters.of(-RobotConfig.getInstance().getWheelbase() / 2.0),
          Meters.of(RobotConfig.getInstance().getTrackwidth() / 2.0),
          !RobotConfig.getInstance().getSwerveConstants().isDriveMotorInverted(),
          RobotConfig.getInstance().getSwerveConstants().isAngleMotorInverted(),
          false);
  private static final SwerveModuleConstants backRight =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[3],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[3],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[3],
          Rotations.of(RobotConfig.getInstance().getSwerveSteerOffsets()[3]),
          Meters.of(-RobotConfig.getInstance().getWheelbase() / 2.0),
          Meters.of(-RobotConfig.getInstance().getTrackwidth() / 2.0),
          RobotConfig.getInstance().getSwerveConstants().isDriveMotorInverted(),
          RobotConfig.getInstance().getSwerveConstants().isAngleMotorInverted(),
          false);

  // gyro signals
  private final StatusSignal<Angle> yawStatusSignal;
  private final StatusSignal<Angle> pitchStatusSignal;
  private final StatusSignal<Angle> rollStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityZStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityXStatusSignal;
  private final StatusSignal<AngularVelocity> angularVelocityYStatusSignal;

  // swerve module signals
  SwerveModuleSignals[] swerveModulesSignals = new SwerveModuleSignals[4];

  private Translation2d centerOfRotation;
  private ChassisSpeeds targetChassisSpeeds;

  private SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private SwerveRequest.RobotCentric driveRobotCentricRequest = new SwerveRequest.RobotCentric();
  private SwerveRequest.FieldCentric driveFieldCentricRequest = new SwerveRequest.FieldCentric();
  private SwerveRequest.FieldCentricFacingAngle driveFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle();
  private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
  private SwerveRequest.ApplyRobotSpeeds applyRobotSpeedsRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  // only used for TorqueCurrentFOC characterization
  private TorqueCurrentFOC[] driveCurrentRequests = new TorqueCurrentFOC[4];
  private TorqueCurrentFOC[] steerCurrentRequests = new TorqueCurrentFOC[4];

  // queues for odometry updates from CTRE's thread
  private final Lock odometryLock = new ReentrantLock();
  List<Queue<Double>> drivePositionQueues = new ArrayList<>();
  List<Queue<Double>> steerPositionQueues = new ArrayList<>();
  Queue<Double> gyroYawQueue;
  Queue<Double> timestampQueue;

  // simulation
  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
  private double lastSimTime;

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

    this.yawStatusSignal = this.getPigeon2().getYaw().clone();
    this.pitchStatusSignal = this.getPigeon2().getPitch().clone();
    this.pitchStatusSignal.setUpdateFrequency(100);
    this.rollStatusSignal = this.getPigeon2().getRoll().clone();
    this.rollStatusSignal.setUpdateFrequency(100);
    this.angularVelocityZStatusSignal = this.getPigeon2().getAngularVelocityZWorld().clone();
    this.angularVelocityXStatusSignal = this.getPigeon2().getAngularVelocityXWorld().clone();
    this.angularVelocityXStatusSignal.setUpdateFrequency(100);
    this.angularVelocityYStatusSignal = this.getPigeon2().getAngularVelocityYWorld().clone();
    this.angularVelocityYStatusSignal.setUpdateFrequency(100);

    for (int i = 0; i < swerveModulesSignals.length; i++) {
      swerveModulesSignals[i] =
          new SwerveModuleSignals(
              this.getModule(i).getSteerMotor(), this.getModule(i).getDriveMotor());
    }

    this.centerOfRotation = new Translation2d(); // default to (0,0)

    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    for (int i = 0; i < driveCurrentRequests.length; i++) {
      this.driveCurrentRequests[i] = new TorqueCurrentFOC(0.0);
      this.steerCurrentRequests[i] = new TorqueCurrentFOC(0.0);
    }

    // configure PID for drive facing angle
    this.driveFacingAngleRequest.HeadingController.setPID(
        driveFacingAngleThetaKp.get(),
        driveFacingAngleThetaKi.get(),
        driveFacingAngleThetaKd.get());
    this.driveFacingAngleRequest.HeadingController.enableContinuousInput(0, Math.PI * 2);

    // always define 0° (towards the red alliance) as "forward"; the Drivetrain subsystem handles
    //  the definition of forward based on the current alliance
    this.driveFacingAngleRequest.ForwardPerspective =
        SwerveRequest.ForwardPerspectiveValue.BlueAlliance;
    this.driveFieldCentricRequest.ForwardPerspective =
        SwerveRequest.ForwardPerspectiveValue.BlueAlliance;

    // create queues for updates from CTRE's odometry thread
    for (int i = 0; i < 4; i++) {
      this.drivePositionQueues.add(new ArrayBlockingQueue<>(20));
      this.steerPositionQueues.add(new ArrayBlockingQueue<>(20));
    }
    this.gyroYawQueue = new ArrayBlockingQueue<>(20);
    this.timestampQueue = new ArrayBlockingQueue<>(20);

    this.registerTelemetry(this::updateTelemetry);

    // register all drivetrain-related devices with FaultReporter
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Pigeon", this.getPigeon2());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FL Drive", this.getModule(0).getDriveMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FL Steer", this.getModule(0).getSteerMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FL Encoder", this.getModule(0).getCANcoder());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FR Drive", this.getModule(1).getDriveMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FR Steer", this.getModule(1).getSteerMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FR Encoder", this.getModule(1).getCANcoder());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BL Drive", this.getModule(2).getDriveMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BL Steer", this.getModule(2).getSteerMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BL Encoder", this.getModule(2).getCANcoder());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BR Drive", this.getModule(3).getDriveMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BR Steer", this.getModule(3).getSteerMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BR Encoder", this.getModule(3).getCANcoder());

    if (Constants.getMode() == Constants.Mode.SIM) {
      startSimThread();
    }
  }

  private void updateTelemetry(SwerveDriveState state) {

    this.odometryLock.lock();

    // update and log the swerve modules telemetry
    for (int i = 0; i < 4; i++) {
      this.drivePositionQueues.get(i).offer(state.ModulePositions[i].distanceMeters);
      this.steerPositionQueues.get(i).offer(state.ModuleStates[i].angle.getDegrees());
    }

    this.gyroYawQueue.offer(state.RawHeading.getDegrees());

    this.timestampQueue.offer(state.Timestamp);

    this.odometryLock.unlock();
  }

  @Override
  public void updateInputs(DrivetrainIOInputsCollection inputs) {

    // update and log gyro inputs
    this.updateGyroInputs(inputs.gyro);

    // update and log the swerve modules inputs
    for (int i = 0; i < swerveModulesSignals.length; i++) {
      this.updateSwerveModuleInputs(inputs.swerve[i], this.getModule(i), swerveModulesSignals[i]);
    }

    inputs.drivetrain.swerveMeasuredStates = this.getState().ModuleStates;
    inputs.drivetrain.swerveReferenceStates = this.getState().ModuleTargets;

    inputs.drivetrain.referenceChassisSpeeds =
        new ChassisSpeeds(
            this.targetChassisSpeeds.vxMetersPerSecond,
            this.targetChassisSpeeds.vyMetersPerSecond,
            this.targetChassisSpeeds.omegaRadiansPerSecond);
    inputs.drivetrain.measuredChassisSpeeds = this.getState().Speeds;

    inputs.drivetrain.averageDriveCurrent = this.getAverageDriveCurrent(inputs);

    inputs.drivetrain.customPose = this.getState().Pose;

    this.odometryLock.lock();

    inputs.drivetrain.odometryTimestamps =
        this.timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    this.timestampQueue.clear();

    inputs.gyro.odometryYawPositions =
        this.gyroYawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    this.gyroYawQueue.clear();

    for (int i = 0; i < swerveModulesSignals.length; i++) {
      inputs.swerve[i].odometryDrivePositionsMeters =
          this.drivePositionQueues.get(i).stream().mapToDouble(Double::valueOf).toArray();
      inputs.swerve[i].odometryTurnPositions =
          this.steerPositionQueues.get(i).stream()
              .map(Rotation2d::fromDegrees)
              .toArray(Rotation2d[]::new);
      this.drivePositionQueues.get(i).clear();
      this.steerPositionQueues.get(i).clear();
    }

    this.odometryLock.unlock();

    // update tunables
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          for (SwerveModule swerveModule : this.getModules()) {
            Slot0Configs slot0 = new Slot0Configs();
            swerveModule.getDriveMotor().getConfigurator().refresh(slot0);
            slot0.kP = pid[0];
            slot0.kI = pid[1];
            slot0.kD = pid[2];
            swerveModule.getDriveMotor().getConfigurator().apply(slot0);
          }
        },
        driveKp,
        driveKi,
        driveKd);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          for (SwerveModule swerveModule : this.getModules()) {
            Slot0Configs slot0 = new Slot0Configs();
            swerveModule.getSteerMotor().getConfigurator().refresh(slot0);
            slot0.kP = pid[0];
            slot0.kI = pid[1];
            slot0.kD = pid[2];
            swerveModule.getSteerMotor().getConfigurator().apply(slot0);
          }
        },
        steerKp,
        steerKi,
        steerKd);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> this.driveFacingAngleRequest.HeadingController.setPID(pid[0], pid[1], pid[2]),
        driveFacingAngleThetaKp,
        driveFacingAngleThetaKi,
        driveFacingAngleThetaKd);
  }

  private void updateGyroInputs(GyroIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        this.yawStatusSignal,
        this.pitchStatusSignal,
        this.rollStatusSignal,
        this.angularVelocityZStatusSignal,
        this.angularVelocityXStatusSignal,
        this.angularVelocityYStatusSignal);

    inputs.connected = (this.yawStatusSignal.getStatus() == StatusCode.OK);
    inputs.yawDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
                this.yawStatusSignal, this.angularVelocityZStatusSignal)
            .in(Degrees);
    inputs.pitchDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
                this.pitchStatusSignal, this.angularVelocityYStatusSignal)
            .in(Degrees);
    inputs.rollDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
                this.rollStatusSignal, this.angularVelocityXStatusSignal)
            .in(Degrees);
    inputs.rollDegPerSec = this.angularVelocityXStatusSignal.getValue().in(DegreesPerSecond);
    inputs.pitchDegPerSec = this.angularVelocityYStatusSignal.getValue().in(DegreesPerSecond);
    inputs.yawDegPerSec = this.angularVelocityZStatusSignal.getValue().in(DegreesPerSecond);
  }

  private void updateSwerveModuleInputs(
      SwerveIOInputs inputs, SwerveModule module, SwerveModuleSignals signals) {

    BaseStatusSignal.refreshAll(
        signals.steerVelocityStatusSignal,
        signals.steerAccelerationStatusSignal,
        signals.steerPositionErrorStatusSignal,
        signals.steerPositionReferenceStatusSignal,
        signals.drivePositionStatusSignal,
        signals.driveVelocityErrorStatusSignal,
        signals.driveVelocityReferenceStatusSignal,
        signals.driveAccelerationStatusSignal);

    SwerveModulePosition position = module.getPosition(false);
    SwerveModuleState state = module.getCurrentState();

    inputs.driveEnabled =
        module.getDriveMotor().getDeviceEnable().getValue() == DeviceEnableValue.Enabled;
    inputs.driveDistanceMeters = position.distanceMeters;
    inputs.driveVelocityMetersPerSec = state.speedMetersPerSecond;

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.
    inputs.driveVelocityReferenceMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            module.getDriveMotor().getClosedLoopReference().getValue(),
            RobotConfig.getInstance().getWheelDiameterMeters() * Math.PI,
            RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio());
    inputs.driveVelocityErrorMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            module.getDriveMotor().getClosedLoopError().getValue(),
            RobotConfig.getInstance().getWheelDiameterMeters() * Math.PI,
            RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio());
    inputs.driveAccelerationMetersPerSecPerSec =
        Conversions.falconRPSToMechanismMPS(
            signals.driveAccelerationStatusSignal.getValue().in(RotationsPerSecond.per(Second)),
            RobotConfig.getInstance().getWheelDiameterMeters() * Math.PI,
            RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio());
    inputs.driveAppliedVolts = module.getDriveMotor().getMotorVoltage().getValue().in(Volts);
    inputs.driveStatorCurrentAmps = module.getDriveMotor().getStatorCurrent().getValue().in(Amps);
    inputs.driveSupplyCurrentAmps = module.getDriveMotor().getSupplyCurrent().getValue().in(Amps);
    inputs.driveTempCelsius = module.getDriveMotor().getDeviceTemp().getValue().in(Celsius);

    inputs.steerAbsolutePositionDeg =
        module.getCANcoder().getAbsolutePosition().getValue().in(Degrees);

    inputs.steerEnabled =
        module.getSteerMotor().getDeviceEnable().getValue() == DeviceEnableValue.Enabled;
    // since we are using the FusedCANcoder feature, the position and velocity signal for the angle
    // motor accounts for the gear ratio; so, pass a gear ratio of 1 to just convert from rotations
    // to degrees.
    inputs.steerPositionDeg = position.angle.getDegrees();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.
    inputs.steerPositionReferenceDeg =
        Conversions.falconRotationsToMechanismDegrees(
            module.getSteerMotor().getClosedLoopReference().getValue(), 1);
    inputs.steerPositionErrorDeg =
        Conversions.falconRotationsToMechanismDegrees(
            module.getSteerMotor().getClosedLoopError().getValue(), 1);
    // FIXME: if the gear ratio is 1, we can just use the Units methods to convert the values
    inputs.steerVelocityRevPerMin =
        Conversions.falconRPSToMechanismRPM(
            signals.steerVelocityStatusSignal.getValue().in(RotationsPerSecond), 1);
    inputs.steerAccelerationMetersPerSecPerSec =
        Conversions.falconRPSToMechanismRPM(
            signals.steerAccelerationStatusSignal.getValue().in(RotationsPerSecond.per(Second)), 1);

    inputs.steerAppliedVolts = module.getSteerMotor().getMotorVoltage().getValue().in(Volts);
    inputs.steerStatorCurrentAmps = module.getSteerMotor().getStatorCurrent().getValue().in(Amps);
    inputs.steerSupplyCurrentAmps = module.getSteerMotor().getSupplyCurrent().getValue().in(Amps);
    inputs.steerTempCelsius = module.getSteerMotor().getDeviceTemp().getValue().in(Celsius);
  }

  @Override
  public void holdXStance() {
    this.targetChassisSpeeds.vxMetersPerSecond = 0.0;
    this.targetChassisSpeeds.vyMetersPerSecond = 0.0;
    this.targetChassisSpeeds.omegaRadiansPerSecond = 0.0;

    this.setControl(this.brakeRequest);
  }

  @Override
  public void driveFieldRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {

    this.targetChassisSpeeds.vxMetersPerSecond = xVelocity;
    this.targetChassisSpeeds.vyMetersPerSecond = yVelocity;
    this.targetChassisSpeeds.omegaRadiansPerSecond = rotationalVelocity;
    this.targetChassisSpeeds.toRobotRelativeSpeeds(
        RobotOdometry.getInstance().getEstimatedPose().getRotation());
    this.targetChassisSpeeds.discretize(Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveFieldCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    } else {
      this.setControl(
          this.driveFieldCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    }
  }

  @Override
  public void driveFieldRelativeFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {

    this.targetChassisSpeeds.vxMetersPerSecond = xVelocity;
    this.targetChassisSpeeds.vyMetersPerSecond = yVelocity;
    this.targetChassisSpeeds.omegaRadiansPerSecond = 0.0;
    this.targetChassisSpeeds.toRobotRelativeSpeeds(
        RobotOdometry.getInstance().getEstimatedPose().getRotation());
    this.targetChassisSpeeds.discretize(Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveFacingAngleRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withTargetDirection(targetDirection));
    } else {
      this.setControl(
          this.driveFacingAngleRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withTargetDirection(targetDirection));
    }
  }

  @Override
  public void pointWheelsAt(Rotation2d targetDirection) {
    this.targetChassisSpeeds.vxMetersPerSecond = 0.0;
    this.targetChassisSpeeds.vyMetersPerSecond = 0.0;
    this.targetChassisSpeeds.omegaRadiansPerSecond = 0.0;

    this.setControl(this.pointRequest.withModuleDirection(targetDirection));
  }

  @Override
  public void driveRobotRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {

    this.targetChassisSpeeds.vxMetersPerSecond = xVelocity;
    this.targetChassisSpeeds.vyMetersPerSecond = yVelocity;
    this.targetChassisSpeeds.omegaRadiansPerSecond = rotationalVelocity;
    this.targetChassisSpeeds.discretize(Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveRobotCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    } else {
      this.setControl(
          this.driveRobotCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    }
  }

  @Override
  public void applyRobotSpeeds(
      ChassisSpeeds speeds, Force[] forcesX, Force[] forcesY, boolean isOpenLoop) {
    this.targetChassisSpeeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
    this.targetChassisSpeeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
    this.targetChassisSpeeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;

    if (isOpenLoop) {
      this.setControl(
          this.applyRobotSpeedsRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(forcesX)
              .withWheelForceFeedforwardsY(forcesY)
              .withCenterOfRotation(this.centerOfRotation));
    } else {
      this.setControl(
          this.applyRobotSpeedsRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withSpeeds(speeds)
              .withWheelForceFeedforwardsX(forcesX)
              .withWheelForceFeedforwardsY(forcesY)
              .withCenterOfRotation(this.centerOfRotation));
    }
  }

  @Override
  public void setCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  @Override
  public void setDriveMotorCurrent(double amps) {
    // ensure that the SwerveDrivetrain class doesn't control either motor
    this.setControl(idleRequest);

    for (int i = 0; i < this.getModules().length; i++) {
      this.getModule(i).getDriveMotor().setControl(driveCurrentRequests[i].withOutput(amps));
    }
  }

  @Override
  public void setSteerMotorCurrent(double amps) {
    // ensure that the SwerveDrivetrain class doesn't control either motor
    this.setControl(idleRequest);

    for (int i = 0; i < this.getModules().length; i++) {
      this.getModule(i).getSteerMotor().setControl(steerCurrentRequests[i].withOutput(amps));
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

  private static ClosedLoopOutputType getSteerClosedLoopOutputType() {
    if (RobotConfig.getInstance().getSwerveSteerControlMode()
        == RobotConfig.SWERVE_CONTROL_MODE.TORQUE_CURRENT_FOC) {
      return ClosedLoopOutputType.TorqueCurrentFOC;
    } else {
      return ClosedLoopOutputType.Voltage;
    }
  }

  private static ClosedLoopOutputType getDriveClosedLoopOutputType() {
    if (RobotConfig.getInstance().getSwerveDriveControlMode()
        == RobotConfig.SWERVE_CONTROL_MODE.TORQUE_CURRENT_FOC) {
      return ClosedLoopOutputType.TorqueCurrentFOC;
    } else {
      return ClosedLoopOutputType.Voltage;
    }
  }

  @SuppressWarnings("resource")
  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    Notifier simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }
}
