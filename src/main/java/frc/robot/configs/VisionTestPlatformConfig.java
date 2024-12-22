package frc.robot.configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class VisionTestPlatformConfig extends RobotConfig {
  private static final String CAMERA_NAME_0 = "OV2311";

  // Front right camera
  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(
          // FIXME: update based on CAD
          new Translation3d(
              Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(36.0)),
          new Rotation3d());

  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

  @Override
  public Transform3d[] getRobotToCameraTransforms() {
    return new Transform3d[] {ROBOT_TO_CAMERA_0};
  }

  @Override
  public String[] getCameraNames() {
    return new String[] {CAMERA_NAME_0};
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
  public double getAutoMaxAcceleration() {
    return 0.0;
  }

  @Override
  public double getAutoMaxSpeed() {
    return 0.0;
  }

  @Override
  public double getRobotMaxVelocity() {
    return 0.0;
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
  public double[] getSwerveSteerOffsets() {
    return new double[] {};
  }

  @Override
  public int getGyroCANID() {
    return 0;
  }

  @Override
  public double getTrackwidth() {
    return 0.0;
  }

  @Override
  public double getWheelbase() {
    return 0.0;
  }

  @Override
  public double getWheelDiameterMeters() {
    return 0.0;
  }
}
