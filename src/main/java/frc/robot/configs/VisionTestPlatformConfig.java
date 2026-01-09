package frc.robot.configs;

import static edu.wpi.first.units.Units.*;

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
public class VisionTestPlatformConfig extends RobotConfig {
  private static final String CAMERA_NAME_0 = "photonvisionVTP";

  // Front right camera
  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(new Translation3d(), new Rotation3d());

  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

  @Override
  public CameraConfig[] getCameraConfigs() {
    return new CameraConfig[] {
      CameraConfig.builder()
          .robotToCameraTransform(ROBOT_TO_CAMERA_0)
          .id(CAMERA_NAME_0)
          .location("")
          .width(1600)
          .height(1200)
          .stdDevFactor(1.0)
          .build(),
    };
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
