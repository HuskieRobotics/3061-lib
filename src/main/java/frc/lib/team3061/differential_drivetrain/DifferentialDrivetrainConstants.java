package frc.lib.team3061.differential_drivetrain;

public class DifferentialDrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private DifferentialDrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Drivetrain";

  // XRP Differential Drivetrain Constants
  public static final double XRP_GEAR_RATIO =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  public static final double XRP_COUNTS_PER_MOTOR_SHAFT_REV = 12.0;
  public static final double XRP_COUNTS_PER_REVOLUTION =
      XRP_COUNTS_PER_MOTOR_SHAFT_REV * XRP_GEAR_RATIO; // 585.0
}
