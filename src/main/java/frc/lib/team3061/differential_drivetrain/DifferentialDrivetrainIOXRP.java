package frc.lib.team3061.differential_drivetrain;

import static frc.lib.team3061.differential_drivetrain.DifferentialDrivetrainConstants.*;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;

public class DifferentialDrivetrainIOXRP implements DifferentialDrivetrainIO {

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive =
      new DifferentialDrive(leftMotor::set, rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  public DifferentialDrivetrainIOXRP() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  @Override
  public void updateInputs(DifferentialDrivetrainIOInputs inputs) {
    inputs.leftMotorSpeed = leftMotor.get();
    inputs.rightMotorSpeed = rightMotor.get();

    inputs.leftEncoderCount = leftEncoder.get();
    inputs.rightEncoderCount = rightEncoder.get();

    inputs.leftMotorPositionInches = leftEncoder.getDistance();
    inputs.rightMotorPositionInches = rightEncoder.getDistance();

    inputs.headingDeg = gyro.getAngleZ();
    inputs.pitchDeg = gyro.getAngleX();
    inputs.rollDeg = gyro.getAngleY();

    inputs.xAccelerationG = accelerometer.getX();
    inputs.yAccelerationG = accelerometer.getY();
    inputs.zAccelerationG = accelerometer.getZ();
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities are specified from the robot's frame of reference. In the
   * robot frame of reference, The origin of the robot is always the center of the robot. The
   * positive x direction is forward; the positive y direction, left. Zero degrees is aligned to the
   * positive x axis and increases in the CCW direction.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param rotationalVelocity the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */
  @Override
  public void driveRobotRelative(double xVelocity, double rotationalVelocity, boolean isOpenLoop) {
    diffDrive.arcadeDrive(xVelocity, rotationalVelocity);
  }

  private void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }
}
