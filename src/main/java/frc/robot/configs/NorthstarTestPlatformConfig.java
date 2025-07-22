package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve_drivetrain.swerve.SwerveConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class NorthstarTestPlatformConfig extends RobotConfig {
  private static final String CAMERA_NAME_0 = "northstar_0";
  private static final String CAMERA_NAME_1 = "northstar_1";

  // Front right camera
  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(
          new Translation3d(0.178, -0.268, 0.236),
          new Rotation3d(new Quaternion(-0.977, -0.032, 0.115, -0.177)));

  // Back right camera
  private static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          new Translation3d(-0.279, -0.273, 0.253),
          new Rotation3d(new Quaternion(-0.149, 0.161, 0.027, 0.975)));

  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

  @Override
  public Transform3d[] getRobotToCameraTransforms() {
    return new Transform3d[] {ROBOT_TO_CAMERA_0, ROBOT_TO_CAMERA_1};
  }

  @Override
  public String[] getCameraNames() {
    return new String[] {CAMERA_NAME_0, CAMERA_NAME_1};
  }

  @Override
  public double[] getCameraStdDevFactors() {
    return new double[] {1.0, 1.0};
  }

  @Override
  public double getOdometryUpdateFrequency() {
    return 250.0;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }

  @Override
  public LinearVelocity getRobotMaxVelocity() {
    return MetersPerSecond.of(0.0);
  }

  @Override
  public SwerveConstants getSwerveConstants() {
    return SwerveConstants.MK4I_L3_PLUS_CONSTANTS;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {};
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {};
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {};
  }

  @Override
  public Angle[] getSwerveSteerOffsets() {
    return new Angle[] {};
  }

  @Override
  public int getGyroCANID() {
    return 0;
  }

  @Override
  public Distance getTrackwidth() {
    return Meters.of(0.0);
  }

  @Override
  public Distance getWheelbase() {
    return Meters.of(0.0);
  }

  @Override
  public Distance getWheelRadius() {
    return Meters.of(0.0);
  }
}
