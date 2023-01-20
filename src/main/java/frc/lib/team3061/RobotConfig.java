package frc.lib.team3061;

import frc.lib.team3061.swerve.SwerveModuleConstants;

public abstract class RobotConfig {

  private static RobotConfig robotConfig;

  public static RobotConfig getInstance() {
    return robotConfig;
  }

  protected RobotConfig() {
    RobotConfig.robotConfig = this;
  }

  // Swerve Module PID accessors
  public double getSwerveAngleKP() {
    return 0.0;
  }

  public double getSwerveAngleKI() {
    return 0.0;
  }

  public double getSwerveAngleKD() {
    return 0.0;
  }

  public double getSwerveAngleKF() {
    return 0.0;
  }

  public double getSwerveDriveKP() {
    return 0.0;
  }

  public double getSwerveDriveKI() {
    return 0.0;
  }

  public double getSwerveDriveKD() {
    return 0.0;
  }

  public double getSwerveDriveKF() {
    return 0.0;
  }

  // Drive Characterization accessors
  public double getDriveKS() {
    return 0.0;
  }

  public double getDriveKV() {
    return 0.0;
  }

  public double getDriveKA() {
    return 0.0;
  }

  // Swerve Module CAN IDs (FL, FR, BL, BR)
  public abstract int[] getSwerveDriveMotorCANIDs();

  public abstract int[] getSwerveSteerMotorCANIDs();

  public abstract int[] getSwerveSteerEncoderCANIDs();

  public double[] getSwerveSteerOffsets() {
    return new double[] {0.0, 0.0, 0.0, 0.0};
  }

  public abstract int getPigeonCANID();

  // robot dimensions accessors
  public abstract double getTrackwidth();

  public abstract double getWheelbase();

  public double getRobotWidthWithBumpers() {
    return 0.0;
  }

  public double getRobotLengthWithBumpers() {
    return 0.0;
  }

  // robot maximum velcoity
  public double getRobotMaxVelocity() {
    return 6380.0
        / 60.0
        / SwerveModuleConstants.DRIVE_GEAR_RATIO
        * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
  }

  // auto path PIDs
  public double getAutoDriveKP() {
    return 0.0;
  }

  public double getAutoDriveKI() {
    return 0.0;
  }

  public double getAutoDriveKD() {
    return 0.0;
  }

  public double getAutoTurnKP() {
    return 0.0;
  }

  public double getAutoTurnKI() {
    return 0.0;
  }

  public double getAutoTurnKD() {
    return 0.0;
  }

  public String getCANBusName() {
    return "";
  }

  public String getCameraName() {
    return "";
  }
}
