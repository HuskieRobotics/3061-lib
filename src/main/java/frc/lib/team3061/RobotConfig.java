package frc.lib.team3061;

public abstract class RobotConfig {

  private static RobotConfig robotConfig;

  public static RobotConfig getRobotConfig() {
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
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {1, 2, 3, 4};
  }

  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {5, 6, 7, 8};
  }

  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {9, 10, 11, 12};
  }

  public double[] getSwerveSteerOffsets() {
    return new double[] {0.0, 0.0, 0.0, 0.0};
  }

  public int getPigeonCANID() {
    return 1;
  }

  // robot dimensions accessors
  public double getTrackwidth() {
    return 0.0;
  }

  public double getWheelbase() {
    return 0.0;
  }

  public double getRobotWidthWithBumpers() {
    return 0.0;
  }

  public double getRobotLengthWithBumpers() {
    return 0.0;
  }

  public double getRobotMaxVelocity() {
    return 0.0;
  }

  // FIXME: tune PID values for auto paths

  public static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
  public static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  public static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  public static final double AUTO_TURN_P_CONTROLLER = 10.0;
  public static final double AUTO_TURN_I_CONTROLLER = 0.0;
  public static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // FIXME: an empty string uses the CAN bus; specify the name of the CANivore as
  // appropriate
  public static final String CAN_BUS_NAME = "";

  // FIXME: specify the name of the camera used for detecting AprilTags
  public static final String CAMERA_NAME = "ov9268";
}
