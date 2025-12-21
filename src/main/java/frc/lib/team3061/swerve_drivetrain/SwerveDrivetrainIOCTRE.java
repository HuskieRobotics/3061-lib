package frc.lib.team3061.swerve_drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainConstants.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrainConstants.SysIDCharacterizationMode;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

@java.lang.SuppressWarnings({"java:S1450"})
public class SwerveDrivetrainIOCTRE extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements SwerveDrivetrainIO {

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

  /*
   * If TUNING is set to true in Constants.java, the following tunables will be available in
   * AdvantageScope. This enables efficient tuning of PID coefficients without restarting the code.
   */
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
  private static final ClosedLoopOutputType steerClosedLoopOutput = getSteerClosedLoopOutputType();

  // The closed-loop output type to use for the drive motors
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput = getDriveClosedLoopOutputType();

  private static final double COUPLE_RATIO =
      RobotConfig.getInstance().getAzimuthSteerCouplingRatio();
  private static final double STEER_INERTIA = 0.01;
  private static final double DRIVE_INERTIA = 0.01;

  // Simulated voltage necessary to overcome friction
  private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
  private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

  private static final SwerveDrivetrainConstants drivetrainConstants =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(RobotConfig.getInstance().getGyroCANID())
          .withCANBusName(RobotConfig.getInstance().getCANBusName())
          .withPigeon2Configs(RobotConfig.getInstance().getPigeonConfigForSwerveDrivetrain());

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontModulesConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withSupplyCurrentLimit(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT)
                              .withSupplyCurrentLowerLimit(
                                  SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT)
                              .withSupplyCurrentLowerTime(
                                  SwerveConstants.DRIVE_PEAK_CURRENT_DURATION)
                              .withSupplyCurrentLimitEnable(
                                  SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT)))
              .withSteerMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withSupplyCurrentLimit(SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT)
                              .withSupplyCurrentLowerLimit(
                                  SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT)
                              .withSupplyCurrentLowerTime(
                                  SwerveConstants.ANGLE_PEAK_CURRENT_DURATION)
                              .withSupplyCurrentLimitEnable(
                                  SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT)
                              .withStatorCurrentLimit(SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT)
                              .withStatorCurrentLimitEnable(
                                  SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT)))
              .withDriveMotorGearRatio(
                  RobotConfig.getInstance().getFrontSwerveConstants().getDriveGearRatio())
              .withSteerMotorGearRatio(
                  RobotConfig.getInstance().getFrontSwerveConstants().getAngleGearRatio())
              .withWheelRadius(RobotConfig.getInstance().getWheelRadius())
              .withSlipCurrent(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12Volts(RobotConfig.getInstance().getRobotMaxVelocity())
              .withSteerInertia(STEER_INERTIA)
              .withDriveInertia(DRIVE_INERTIA)
              .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
              .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(
                  COUPLE_RATIO); // Every 1 rotation of the azimuth results in couple ratio drive
  // turns

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backModulesConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withSupplyCurrentLimit(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT)
                              .withSupplyCurrentLowerLimit(
                                  SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT)
                              .withSupplyCurrentLowerTime(
                                  SwerveConstants.DRIVE_PEAK_CURRENT_DURATION)
                              .withSupplyCurrentLimitEnable(
                                  SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT)))
              .withSteerMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              .withSupplyCurrentLimit(SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT)
                              .withSupplyCurrentLowerLimit(
                                  SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT)
                              .withSupplyCurrentLowerTime(
                                  SwerveConstants.ANGLE_PEAK_CURRENT_DURATION)
                              .withSupplyCurrentLimitEnable(
                                  SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT)
                              .withStatorCurrentLimit(SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT)
                              .withStatorCurrentLimitEnable(
                                  SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT)))
              .withDriveMotorGearRatio(
                  RobotConfig.getInstance().getBackSwerveConstants().getDriveGearRatio())
              .withSteerMotorGearRatio(
                  RobotConfig.getInstance().getBackSwerveConstants().getAngleGearRatio())
              .withWheelRadius(RobotConfig.getInstance().getWheelRadius())
              .withSlipCurrent(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12Volts(RobotConfig.getInstance().getRobotMaxVelocity())
              .withSteerInertia(STEER_INERTIA)
              .withDriveInertia(DRIVE_INERTIA)
              .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
              .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(
                  COUPLE_RATIO); // Every 1 rotation of the azimuth results in couple ratio drive
  // turns

  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontLeft =
          frontModulesConstantCreator.createModuleConstants(
              RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[0],
              RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[0],
              RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[0],
              RobotConfig.getInstance().getSwerveSteerOffsets()[0],
              RobotConfig.getInstance().getWheelbase().div(2.0),
              RobotConfig.getInstance().getTrackwidth().div(2.0),
              !RobotConfig.getInstance().getFrontSwerveConstants().isDriveMotorInverted(),
              RobotConfig.getInstance().getFrontSwerveConstants().isAngleMotorInverted(),
              false);
  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontRight =
          frontModulesConstantCreator.createModuleConstants(
              RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[1],
              RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[1],
              RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[1],
              RobotConfig.getInstance().getSwerveSteerOffsets()[1],
              RobotConfig.getInstance().getWheelbase().div(2.0),
              RobotConfig.getInstance().getTrackwidth().div(-2.0),
              RobotConfig.getInstance().getFrontSwerveConstants().isDriveMotorInverted(),
              RobotConfig.getInstance().getFrontSwerveConstants().isAngleMotorInverted(),
              false);
  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backLeft =
          backModulesConstantCreator.createModuleConstants(
              RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[2],
              RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[2],
              RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[2],
              RobotConfig.getInstance().getSwerveSteerOffsets()[2],
              RobotConfig.getInstance().getWheelbase().div(-2.0),
              RobotConfig.getInstance().getTrackwidth().div(2.0),
              !RobotConfig.getInstance().getBackSwerveConstants().isDriveMotorInverted(),
              RobotConfig.getInstance().getBackSwerveConstants().isAngleMotorInverted(),
              false);
  private static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backRight =
          backModulesConstantCreator.createModuleConstants(
              RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[3],
              RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[3],
              RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[3],
              RobotConfig.getInstance().getSwerveSteerOffsets()[3],
              RobotConfig.getInstance().getWheelbase().div(-2.0),
              RobotConfig.getInstance().getTrackwidth().div(-2.0),
              RobotConfig.getInstance().getBackSwerveConstants().isDriveMotorInverted(),
              RobotConfig.getInstance().getBackSwerveConstants().isAngleMotorInverted(),
              false);

  private Translation2d centerOfRotation;
  private ChassisSpeeds targetChassisSpeeds;

  private SwerveRequest.RobotCentric driveRobotCentricRequest = new SwerveRequest.RobotCentric();
  private SwerveRequest.FieldCentric driveFieldCentricRequest = new SwerveRequest.FieldCentric();
  private SwerveRequest.FieldCentricFacingAngle driveFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle();
  private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
  private SwerveRequest.ApplyRobotSpeeds applyRobotSpeedsRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation translationCharacterizationVolts =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SysIdSwerveTranslation_Torque translationCharacterizationCurrent =
      new SysIdSwerveTranslation_Torque();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterizationVolts =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SysIdSwerveSteerGains_Torque steerCharacterizationCurrent =
      new SysIdSwerveSteerGains_Torque();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  // queues for odometry updates from CTRE's thread
  private final Lock odometryLock = new ReentrantLock();
  List<Queue<Double>> drivePositionQueues = new ArrayList<>();
  List<Queue<Double>> steerPositionQueues = new ArrayList<>();
  Queue<Double> gyroYawQueue;
  Queue<Double> timestampQueue;
  Queue<Double> ctreTimestampQueue;

  // gyro status signals
  private StatusSignal<Angle> pitchStatusSignal;
  private StatusSignal<Angle> rollStatusSignal;
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  // brake mode
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(1);

  // simulation
  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms
  private double lastSimTime;

  /** Creates a new Drivetrain subsystem. */
  public SwerveDrivetrainIOCTRE() {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        drivetrainConstants,
        RobotConfig.getInstance().getOdometryUpdateFrequency(),
        frontLeft,
        frontRight,
        backLeft,
        backRight);

    this.centerOfRotation = new Translation2d(); // default to (0,0)

    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

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
    this.ctreTimestampQueue = new ArrayBlockingQueue<>(20);

    this.registerTelemetry(this::updateTelemetry);

    this.rollStatusSignal = this.getPigeon2().getRoll();
    this.pitchStatusSignal = this.getPigeon2().getPitch();

    Phoenix6Util.registerSignals(true, rollStatusSignal, pitchStatusSignal);

    // register all drivetrain-related devices with FaultReporter
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "Pigeon", this.getPigeon2());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FL Drive", this.getModule(0).getDriveMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FL Steer", this.getModule(0).getSteerMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FL Encoder", this.getModule(0).getEncoder());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FR Drive", this.getModule(1).getDriveMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FR Steer", this.getModule(1).getSteerMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "FR Encoder", this.getModule(1).getEncoder());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BL Drive", this.getModule(2).getDriveMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BL Steer", this.getModule(2).getSteerMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BL Encoder", this.getModule(2).getEncoder());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BR Drive", this.getModule(3).getDriveMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BR Steer", this.getModule(3).getSteerMotor());
    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, "BR Encoder", this.getModule(3).getEncoder());

    if (Constants.getMode() == Constants.Mode.SIM) {
      startSimThread();
    }
  }

  private void updateTelemetry(SwerveDriveState state) {
    this.odometryLock.lock();

    // update and log the swerve modules telemetry
    for (int i = 0; i < state.ModuleStates.length; i++) {
      this.drivePositionQueues.get(i).offer(state.ModulePositions[i].distanceMeters);
      this.steerPositionQueues.get(i).offer(state.ModuleStates[i].angle.getDegrees());
    }

    this.gyroYawQueue.offer(state.RawHeading.getDegrees());

    // convert from the timebase used by getCurrentTimeSeconds to the FPGA timebase to enable
    // replays
    this.timestampQueue.offer(
        Timer.getFPGATimestamp() - (Utils.getCurrentTimeSeconds() - state.Timestamp));
    this.ctreTimestampQueue.offer(state.Timestamp);

    this.odometryLock.unlock();
  }

  @Override
  public void updateInputs(SwerveDrivetrainIOInputsCollection inputs) {

    // update and log the swerve modules inputs
    for (int i = 0; i < this.getModules().length; i++) {
      this.updateSwerveModuleInputs(inputs.swerve[i], this.getModule(i));
    }

    inputs.drivetrain.swerveModulePositions = this.getState().ModulePositions;
    inputs.drivetrain.swerveMeasuredStates = this.getState().ModuleStates;
    inputs.drivetrain.swerveReferenceStates = this.getState().ModuleTargets;

    if (TUNING_MODE) {
      double averageMeasuredRotations = 0.0;
      double averageReferenceRotations = 0.0;
      double averageMeasuredSpeedMetersPerSecond = 0.0;
      double averageReferenceSpeedMetersPerSecond = 0.0;

      // to facilitate tuning, coerce the measured swerve module states' positions to 0 to 0.5 so
      // they can be compared to the reference positions; we coerce to 0.5 instead of 1.0 since the
      // swerve module may run the drive motor backward to move to a closer position.
      for (int i = 0; i < inputs.drivetrain.swerveMeasuredStates.length; i++) {
        double measuredRotations = inputs.drivetrain.swerveMeasuredStates[i].angle.getRotations();
        double referenceRotations = inputs.drivetrain.swerveReferenceStates[i].angle.getRotations();

        // subtract the integer portion of the rotations
        measuredRotations = measuredRotations - Math.floor(measuredRotations);
        referenceRotations = referenceRotations - Math.floor(referenceRotations);

        // if rotations is negative, add 1 to make it positive
        if (measuredRotations < 0) {
          measuredRotations += 1.0;
        }
        if (referenceRotations < 0) {
          referenceRotations += 1.0;
        }

        // if rotations is greater than 0.5, subtract 0.5 to coerce it to the range [0, 0.5]
        if (measuredRotations > 0.5) {
          measuredRotations -= 0.5;
        }
        if (referenceRotations > 0.5) {
          referenceRotations -= 0.5;
        }

        averageMeasuredRotations += measuredRotations;
        averageReferenceRotations += referenceRotations;
        averageMeasuredSpeedMetersPerSecond +=
            Math.abs(inputs.drivetrain.swerveMeasuredStates[i].speedMetersPerSecond);
        averageReferenceSpeedMetersPerSecond +=
            Math.abs(inputs.drivetrain.swerveReferenceStates[i].speedMetersPerSecond);
      }
      inputs.drivetrain.averageSwerveMeasuredAngle =
          Rotation2d.fromRotations(
              averageMeasuredRotations / inputs.drivetrain.swerveMeasuredStates.length);
      inputs.drivetrain.averageSwerveReferenceAngle =
          Rotation2d.fromRotations(
              averageReferenceRotations / inputs.drivetrain.swerveReferenceStates.length);
      inputs.drivetrain.averageSwerveMeasuredSpeed =
          MetersPerSecond.of(
              averageMeasuredSpeedMetersPerSecond / inputs.drivetrain.swerveMeasuredStates.length);
      inputs.drivetrain.averageSwerveReferenceSpeed =
          MetersPerSecond.of(
              averageReferenceSpeedMetersPerSecond
                  / inputs.drivetrain.swerveReferenceStates.length);
    }

    inputs.drivetrain.referenceChassisSpeeds =
        new ChassisSpeeds(
            this.targetChassisSpeeds.vxMetersPerSecond,
            this.targetChassisSpeeds.vyMetersPerSecond,
            this.targetChassisSpeeds.omegaRadiansPerSecond);
    inputs.drivetrain.measuredChassisSpeeds = this.getState().Speeds;

    inputs.drivetrain.averageDriveCurrent = this.getAverageDriveCurrent(inputs);
    inputs.drivetrain.rawHeading = Degrees.of(this.getState().RawHeading.getDegrees());

    inputs.drivetrain.gyroConnected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(pitchStatusSignal, rollStatusSignal));

    inputs.drivetrain.pitch = this.pitchStatusSignal.getValue();
    inputs.drivetrain.roll = this.rollStatusSignal.getValue();

    inputs.drivetrain.customPose = this.getState().Pose;

    inputs.drivetrain.odometryPeriod = this.getState().OdometryPeriod;
    inputs.drivetrain.successfulDAQs = this.getState().SuccessfulDaqs;
    inputs.drivetrain.failedDAQs = this.getState().FailedDaqs;

    this.odometryLock.lock();

    inputs.drivetrain.odometryTimestamps =
        this.timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    this.timestampQueue.clear();

    inputs.drivetrain.odometryCTRETimestamps =
        this.ctreTimestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    this.ctreTimestampQueue.clear();

    inputs.drivetrain.odometryYawPositions =
        this.gyroYawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    this.gyroYawQueue.clear();

    for (int i = 0; i < this.getModules().length; i++) {
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
          for (SwerveModule<TalonFX, TalonFX, CANcoder> swerveModule : this.getModules()) {
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
          for (SwerveModule<TalonFX, TalonFX, CANcoder> swerveModule : this.getModules()) {
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

  private void updateSwerveModuleInputs(
      SwerveIOInputs inputs, SwerveModule<TalonFX, TalonFX, CANcoder> module) {
    inputs.driveEnabled =
        module.getDriveMotor().getDeviceEnable().getValue() == DeviceEnableValue.Enabled;
    inputs.driveStatorCurrent = module.getDriveMotor().getStatorCurrent().getValue();
    inputs.driveSupplyCurrent = module.getDriveMotor().getSupplyCurrent().getValue();
    inputs.driveTemp = module.getDriveMotor().getDeviceTemp().getValue();
    inputs.driveVoltage = module.getDriveMotor().getMotorVoltage().getValue();

    inputs.steerAbsolutePosition = module.getEncoder().getAbsolutePosition().getValue();

    inputs.steerEnabled =
        module.getSteerMotor().getDeviceEnable().getValue() == DeviceEnableValue.Enabled;
    inputs.steerStatorCurrent = module.getSteerMotor().getStatorCurrent().getValue();
    inputs.steerSupplyCurrent = module.getSteerMotor().getSupplyCurrent().getValue();
    inputs.steerTemp = module.getSteerMotor().getDeviceTemp().getValue();
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
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rotationalVelocity,
      boolean isOpenLoop) {

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                rotationalVelocity,
                RobotOdometry.getInstance().getEstimatedPose().getRotation()),
            Constants.LOOP_PERIOD_SECS);

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
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      Rotation2d targetDirection,
      boolean isOpenLoop) {

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                RadiansPerSecond.of(0.0),
                RobotOdometry.getInstance().getEstimatedPose().getRotation()),
            Constants.LOOP_PERIOD_SECS);

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
      LinearVelocity xVelocity,
      LinearVelocity yVelocity,
      AngularVelocity rotationalVelocity,
      boolean isOpenLoop) {

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity),
            Constants.LOOP_PERIOD_SECS);

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
  public void applySysIdCharacterization(SysIDCharacterizationMode mode, double value) {
    switch (mode) {
      case TRANSLATION_VOLTS:
        this.setControl(this.translationCharacterizationVolts.withVolts(value));
        break;
      case TRANSLATION_CURRENT:
        this.setControl(this.translationCharacterizationCurrent.withTorqueCurrent(value));
        break;
      case STEER_VOLTS:
        this.setControl(this.steerCharacterizationVolts.withVolts(value));
        break;
      case STEER_CURRENT:
        this.setControl(this.steerCharacterizationCurrent.withTorqueCurrent(value));
        break;
      case ROTATION_VOLTS:
        this.setControl(this.rotationCharacterization.withRotationalRate(value));
        break;
      default:
        break;
    }
  }

  @Override
  public void setCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // Change the neutral mode configuration in a separate thread since changing the configuration
    // of a CTRE device may take a significant amount of time (~200 ms).
    brakeModeExecutor.execute(
        () -> {
          for (SwerveModule<TalonFX, TalonFX, CANcoder> swerveModule : this.getModules()) {
            swerveModule
                .getDriveMotor()
                .setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast, 0.25);
            swerveModule
                .getSteerMotor()
                .setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast, 0.25);
          }
        });
  }

  /**
   * Returns the average current of the swerve module drive motors in amps.
   *
   * @return the average current of the swerve module drive motors in amps
   */
  private Current getAverageDriveCurrent(SwerveDrivetrainIOInputsCollection inputs) {
    double totalCurrent = 0.0;
    for (SwerveIOInputs swerveInputs : inputs.swerve) {
      totalCurrent += Math.abs(swerveInputs.driveStatorCurrent.in(Amps));
    }
    return Amps.of(totalCurrent / inputs.swerve.length);
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
