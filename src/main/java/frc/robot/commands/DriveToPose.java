package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.swerve_drivetrain.SwerveDrivetrain;
import frc.robot.Field2d;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the supplied pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to PathPlanner. For generating a path on the fly and
 * following that path, refer to Field2d's makePath method.
 *
 * <p>This command is highly customizable with a variety of suppliers. It can be further customized
 * by extending and overriding the desired methods.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When one of the following is true: it is specified that this command finishes when
 * the the robot is at the target pose and the robot is at the target pose (within the specified
 * tolerances), when the timeout occurs, when it is determined that the robot cannot reach the
 * target pose, or when the move-to-pose feature is disabled on the drivetrain subsystem.
 *
 * <p>At End: stops the drivetrain, disables acceleration limiting
 */
public class DriveToPose extends Command {
  private final SwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final BiFunction<Double, Double, Double> xSupplier;
  private final BiFunction<Double, Double, Double> ySupplier;
  private final BiFunction<Double, Double, Double> thetaSupplier;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter thetaLimiter;
  private Transform2d targetTolerance;
  private final boolean finishesWhenAtTarget;
  private final Consumer<Boolean> atTargetConsumer;
  private final Consumer<Transform2d> poseDifferenceConsumer;
  private double timeout;

  private Timer timer;
  private boolean firstRun = true;
  private Transform2d poseDifferenceInTargetFrame; // optimization to calculate once per loop

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * target pose.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param targetPoseSupplier a supplier that returns the pose to drive to. A pose supplier is
   *     specified instead of a pose since the target pose may not be known when this command is
   *     created. In addition, for more complex applications, this provides the opportunity for the
   *     target pose to change while this command executes.
   * @param xSupplier a function that takes the current and target x positions in the frame of the
   *     target pose and returns the x velocity in the frame of the target pose to drive at. The
   *     function should return a value in meters per second. Most commonly, this will be be
   *     calculated by a PID controller.
   * @param ySupplier a function that takes the current and target y positions in the frame of the
   *     target pose and returns the y velocity in the frame of the target pose to drive at. The
   *     function should return a value in meters per second. Most commonly, this will be be
   *     calculated by a PID controller. However, this enables other approaches such as fusing
   *     driver input (like was done in 2025 when aligning to the barge).
   * @param thetaSupplier a function that takes the current and target theta positions and returns
   *     the theta velocity to drive at. The function should return a value in radians per second.
   *     Most commonly, this will be be calculated by a PID controller.
   * @param xLimiter the slew rate limiter for the x velocity in the frame of the target pose
   * @param yLimiter the slew rate limiter for the y velocity in the frame of the target pose
   * @param thetaLimiter the slew rate limiter for the theta velocity
   * @param targetTolerance the tolerance for determining if the robot has reached the target pose
   *     specified in meters. This tolerance is specified in the frame of the target pose, not in
   *     the field frame. If more complex logic is needed, the isPoseWithinTolerance method can be
   *     overriden (e.g., for asymmetric tolerances).
   * @param finishesWhenAtTarget if true, this command will finish when the robot is at the target
   *     pose (within the specified tolerances). If false, this command will not finish when the
   *     robot is at the target pose, but will continue until another condition is met (e.g.,
   *     timeout, cannot reach target pose, move-to-pose feature disabled) or the command is
   *     canceled.
   * @param atTargetConsumer a consumer that is invoked with a boolean indicating whether the robot
   *     is at the target pose (within the specified tolerances). This can be used to change LEDs to
   *     indicate to the driver that the robot is at the target pose.
   * @param poseDifferenceConsumer a consumer that is invoked with the pose difference in the frame
   *     of the target pose. This can be used in more complicated collections of commands to control
   *     other subsystems as the robot moves towards the target pose.
   * @param timeout the timeout in seconds for this command. If the timeout is reached, this command
   *     will finish. If the timeout is less than or equal to zero, this command will not time out.
   *     This is useful for debugging purposes, but should be set to a reasonable value in
   *     production code to prevent the robot from driving indefinitely.
   */
  public DriveToPose(
      SwerveDrivetrain drivetrain,
      Supplier<Pose2d> targetPoseSupplier,
      BiFunction<Double, Double, Double> xSupplier,
      BiFunction<Double, Double, Double> ySupplier,
      BiFunction<Double, Double, Double> thetaSupplier,
      SlewRateLimiter xLimiter,
      SlewRateLimiter yLimiter,
      SlewRateLimiter thetaLimiter,
      Transform2d targetTolerance,
      boolean finishesWhenAtTarget,
      Consumer<Boolean> atTargetConsumer,
      Consumer<Transform2d> poseDifferenceConsumer,
      double timeout) {
    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.thetaSupplier = thetaSupplier;
    this.xLimiter = xLimiter;
    this.yLimiter = yLimiter;
    this.thetaLimiter = thetaLimiter;
    this.targetTolerance = targetTolerance;
    this.finishesWhenAtTarget = finishesWhenAtTarget;
    this.atTargetConsumer = atTargetConsumer;
    this.poseDifferenceConsumer = poseDifferenceConsumer;
    this.timeout = timeout;

    this.timer = new Timer();
    addRequirements(drivetrain);
  }

  /**
   * This method is invoked once when this command is scheduled. It enables acceleration limiting on
   * the drivetrain subsystem, initializes the timer, and sets the firstRun flag to true. This
   * operations need to occur in this method and not in the constructor as this command may be
   * scheduled multiple times.
   */
  @Override
  public void initialize() {
    Logger.recordOutput("DriveToPose/isFinished", false);
    Logger.recordOutput("DriveToPose/withinTolerance", false);

    this.timer.restart();
    this.firstRun = true;
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities by invoking the suppliers and passing the current and target poses. This method
   * accounts for the alliance color; so, the suppliers should not do this. The adjustVelocities
   * method is invoked (and can be overriden) to provide the opportunity to change the calculated
   * velocities if needed by the application (e.g., boosting the x velocity to ensure the robot
   * drives into a field element). After all calculations and adjustments, the drivetrain
   * subsystem's drive method is invoked.
   */
  @Override
  public void execute() {
    Pose2d targetPose = targetPoseSupplier.get();
    Pose2d currentPose = drivetrain.getPose();

    if (firstRun) {
      // reset the limiters on the first run to prevent sudden jumps in velocity
      ChassisSpeeds currentSpeeds = drivetrain.getRobotRelativeSpeeds();
      Translation2d velocitiesInTargetFrame =
          new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
              .rotateBy(targetPose.getRotation().unaryMinus());

      xLimiter.reset(velocitiesInTargetFrame.getX());
      yLimiter.reset(velocitiesInTargetFrame.getY());
      thetaLimiter.reset(currentSpeeds.omegaRadiansPerSecond);
    }

    // calculate the pose difference in the frame of the field
    Transform2d fieldRelativeDifference =
        new Transform2d(
            targetPose.getX() - currentPose.getX(),
            targetPose.getY() - currentPose.getY(),
            Rotation2d.fromRadians(
                targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));

    // transform the current pose into the frame of the target pose
    this.poseDifferenceInTargetFrame = new Transform2d(targetPose, currentPose);
    Pose2d currentPoseInTargetFrame =
        new Pose2d(
            this.poseDifferenceInTargetFrame.getX(),
            this.poseDifferenceInTargetFrame.getY(),
            this.poseDifferenceInTargetFrame.getRotation());

    // calculate new velocities in the frame of the target pose
    double xVelocityInTargetFrame = xSupplier.apply(currentPoseInTargetFrame.getX(), 0.0);
    double yVelocityInTargetFrame = ySupplier.apply(currentPoseInTargetFrame.getY(), 0.0);
    double thetaVelocity =
        thetaSupplier.apply(currentPoseInTargetFrame.getRotation().getRadians(), 0.0);

    // apply the slew rate limiters
    xVelocityInTargetFrame = xLimiter.calculate(xVelocityInTargetFrame);
    yVelocityInTargetFrame = yLimiter.calculate(yVelocityInTargetFrame);
    thetaVelocity = thetaLimiter.calculate(thetaVelocity);

    // opportunity to adjust velocities in the robot frame
    Translation2d velocitiesInTargetFrame =
        adjustVelocities(
            new Translation2d(xVelocityInTargetFrame, yVelocityInTargetFrame),
            this.poseDifferenceInTargetFrame);

    // convert the velocities in the frame of the target pose back into the field frame
    Translation2d fieldRelativeVelocities =
        velocitiesInTargetFrame.rotateBy(targetPose.getRotation());

    // account for which side of the field the driver station is on
    int allianceMultiplier = Field2d.getInstance().getAlliance() == Alliance.Blue ? 1 : -1;

    drivetrain.drive(
        allianceMultiplier * fieldRelativeVelocities.getX(),
        allianceMultiplier * fieldRelativeVelocities.getY(),
        thetaVelocity,
        true,
        true);

    Logger.recordOutput("DriveToPose/targetPose", targetPose);
    Logger.recordOutput("DriveToPose/x velocity (field frame)", fieldRelativeVelocities.getX());
    Logger.recordOutput("DriveToPose/y velocity (field frame)", fieldRelativeVelocities.getY());
    Logger.recordOutput("DriveToPose/theta velocity (field frame)", thetaVelocity);
    Logger.recordOutput("DriveToPose/pose difference (field frame)", fieldRelativeDifference);
    Logger.recordOutput(
        "DriveToPose/pose difference (target frame)", this.poseDifferenceInTargetFrame);
    Logger.recordOutput("DriveToPose/x velocity (target frame)", velocitiesInTargetFrame.getX());
    Logger.recordOutput("DriveToPose/y velocity (target frame)", velocitiesInTargetFrame.getY());
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). The isPoseWithinTolerance method is invoked
   * (and can be overriden) to provide custom definitions of "finished" (e.g., asymmetric
   * tolerances). The canReachTargetPose method is invoked (and can be overriden) to provide
   * specialized behavior to determine if the robot can physically reach the target pose (e.g., a
   * field element is in its path). This command is considered finished when one of the following is
   * true: it is specified that this command finishes when the the robot is at the target pose and
   * the robot is at the target pose (within the specified tolerances), when the timeout occurs, or
   * when it is determined that the robot cannot reach the target pose.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    poseDifferenceConsumer.accept(this.poseDifferenceInTargetFrame);

    boolean withinTolerance = isPoseWithinTolerance();
    atTargetConsumer.accept(withinTolerance);

    boolean canReachTargetPose = true;
    if (this.firstRun) {
      this.firstRun = false;
      canReachTargetPose = canReachTargetPose(this.poseDifferenceInTargetFrame);
      if (!canReachTargetPose) {
        drivetrain.setDriveToPoseCanceled(true);
      }
      Logger.recordOutput("DriveToPose/canReachTargetPose", canReachTargetPose);
    }

    Logger.recordOutput("DriveToPose/withinTolerance", withinTolerance);

    return (finishesWhenAtTarget && withinTolerance)
        || !canReachTargetPose
        || this.timer.hasElapsed(timeout);
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain and disables acceleration limiting.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    Logger.recordOutput("DriveToPose/isFinished", true);
  }

  /**
   * This method is invoked to adjust the velocities in the frame of the target pose. It can be
   * overridden to provide custom behavior for adjusting the velocities. By default, it returns the
   * velocities unchanged. This provides the opportunity to adjust the velocities based on the pose
   * difference in the target frame. For example, this can be used to boost the x velocity to ensure
   * the robot drives into a field element.
   *
   * @param velocitiesInTargetFrame the velocities in the robot frame, calculated in the frame of
   *     the target pose
   * @param poseDifferenceInTargetFrame the pose difference in the target frame, which can be used
   *     to adjust the velocities
   * @return the adjusted velocities in the of the target pose
   */
  public Translation2d adjustVelocities(
      Translation2d velocitiesInTargetFrame, Transform2d poseDifferenceInTargetFrame) {
    return velocitiesInTargetFrame;
  }

  /**
   * This method checks if the robot is within the specified tolerances of the target pose in the
   * frame of the target pose. This method can be overridden to provide custom behavior for
   * determining if the robot is at the target pose. For example, this can be used to provide
   * asymmetric tolerances.
   *
   * @return true if the robot is within the specified tolerances of the target pose (in the frame
   *     of the target pose), false otherwise
   */
  public boolean isPoseWithinTolerance() {
    return Math.abs(this.poseDifferenceInTargetFrame.getX()) < this.targetTolerance.getX()
        && Math.abs(this.poseDifferenceInTargetFrame.getY()) < this.targetTolerance.getY()
        && Math.abs(this.poseDifferenceInTargetFrame.getRotation().getRadians())
            < this.targetTolerance.getRotation().getRadians();
  }

  /**
   * This method checks if the robot can reach the target pose based on the pose difference in the
   * frame of the target pose. This method can be overridden to provide custom behavior for
   * determining if the robot can physically reach the target pose. For example, this can be used to
   * determine if a field element is in the robot's path.
   *
   * @param poseDifferenceInTargetFrame the pose difference in the frame of the target pose
   * @return true if the robot can reach the target pose, false otherwise
   */
  public boolean canReachTargetPose(Transform2d poseDifferenceInTargetFrame) {
    // by default, assume we can reach the target pose
    return true;
  }
}
