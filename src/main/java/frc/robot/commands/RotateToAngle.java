package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to rotate to the specified angle
 * while driving based on the supplied x and y values (e.g., from a joystick). The execute method
 * invokes the drivetrain subsystem's drive method.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified angle (within the specified tolerances)
 *
 * <p>At End: nothing (the drivetrain is left in whatever state it was in when the command finished)
 */
public class RotateToAngle extends CommandBase {
  private final Drivetrain drivetrain;
  private final DoubleSupplier targetAngleSupplier;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  protected static final TunableNumber thetaKp = new TunableNumber("RotateToAngle/ThetaKp", 2);
  protected static final TunableNumber thetaKi = new TunableNumber("RotateToAngle/ThetaKi", 10.0);
  protected static final TunableNumber thetaKd = new TunableNumber("RotateToAngle/ThetaKd", 0.1);
  protected static final TunableNumber thetaMaxVelocity =
      new TunableNumber("RotateToAngle/ThetaMaxVelocity", 8);
  protected static final TunableNumber thetaMaxAcceleration =
      new TunableNumber("RotateToAngle/ThetaMaxAcceleration", 100);
  protected static final TunableNumber thetaTolerance =
      new TunableNumber("RotateToAngle/ThetaTolerance", 0.008);

  protected final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
          LOOP_PERIOD_SECS);

  /**
   * Constructs a new RotateToAngle command that, when executed, instructs the drivetrain subsystem
   * to rotate to the specified angle in place.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param targetAngleSupplier the supplier of the target angle, in degrees. Zero degrees is away
   *     from the driver and increases in the CCW direction.
   */
  public RotateToAngle(Drivetrain drivetrain, DoubleSupplier targetAngleSupplier) {
    this(drivetrain, () -> 0, () -> 0, targetAngleSupplier);
  }

  /**
   * Constructs a new RotateToAngle command that, when executed, instructs the drivetrain subsystem
   * to rotate to the specified angle while driving based on the supplied x and y values (e.g., from
   * a joystick).
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param translationXSupplier the supplier of the x value as a percentage of the maximum velocity
   *     in the x direction as defined by the standard field or robot coordinate system
   * @param translationYSupplier the supplier of the y value as a percentage of the maximum velocity
   *     in the y direction as defined by the standard field or robot coordinate system
   * @param targetAngleSupplier the supplier of the target angle, in degrees. Zero degrees is away
   *     from the driver and increases in the CCW direction.
   */
  public RotateToAngle(
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier targetAngleSupplier) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.targetAngleSupplier = targetAngleSupplier;
  }

  /**
   * This method is invoked once when this command is scheduled. It resets all the PID controller
   * and queries the target angle. It is critical that this initialization occurs in this method and
   * not the constructor as this object is constructed well before the command is scheduled and the
   * robot's pose will definitely have changed and the target angle may not be known until this
   * command is scheduled.
   */
  @Override
  public void initialize() {
    Logger.getInstance().recordOutput("ActiveCommands/RotateToAngle", true);

    Pose2d currentPose = drivetrain.getPose();
    thetaController.reset(currentPose.getRotation().getRadians());
    thetaController.setTolerance(thetaTolerance.get());

    // configure the controller such that the range of values is centered on the target angle
    thetaController.enableContinuousInput(
        Units.degreesToRadians(this.targetAngleSupplier.getAsDouble()) - Math.PI,
        Units.degreesToRadians(this.targetAngleSupplier.getAsDouble()) + Math.PI);

    Logger.getInstance().recordOutput("RotateToAngle/AngleDeg", targetAngleSupplier.getAsDouble());
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target rotation and invokes the drivetrain subsystem's
   * drive method.
   */
  @Override
  public void execute() {
    // update from tunable numbers
    if (thetaKp.hasChanged()
        || thetaKd.hasChanged()
        || thetaKi.hasChanged()
        || thetaMaxVelocity.hasChanged()
        || thetaMaxAcceleration.hasChanged()
        || thetaTolerance.hasChanged()) {
      thetaController.setP(thetaKp.get());
      thetaController.setI(thetaKi.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    Pose2d currentPose = drivetrain.getPose();
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(),
            Units.degreesToRadians(this.targetAngleSupplier.getAsDouble()));
    if (thetaController.atGoal()) {
      thetaVelocity = 0.0;
    }
    double xPercentage = TeleopSwerve.modifyAxis(translationXSupplier.getAsDouble(), 2.0);
    double yPercentage = TeleopSwerve.modifyAxis(translationYSupplier.getAsDouble(), 2.0);

    double xVelocity = xPercentage * RobotConfig.getInstance().getRobotMaxVelocity();
    double yVelocity = yPercentage * RobotConfig.getInstance().getRobotMaxVelocity();

    drivetrain.drive(xVelocity, yVelocity, thetaVelocity, true, false);
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). This command is considered finished if the
   * rotational controller is at its goal.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    return thetaController.atGoal();
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. The drivetrain is
   * left in whatever state it was in when the command finished.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    Logger.getInstance().recordOutput("ActiveCommands/RotateToAngle", false);
  }
}
