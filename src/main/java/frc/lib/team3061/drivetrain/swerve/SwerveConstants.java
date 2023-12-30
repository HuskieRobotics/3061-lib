/*
 * Initially from https://github.com/Team364/BaseFalconSwerve
 */

package frc.lib.team3061.drivetrain.swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;

public final class SwerveConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private SwerveConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public enum SwerveType {
    MK4,
    MK4I
  }

  /* MK4i L2 */

  /*
  	Wheel diameter is best determined empirically. Refer to this document for more information: !!!
  */
  public static final double MK4I_L2_WHEEL_DIAMETER_METERS = 0.09659072671;

  public static final double MK4I_L2_WHEEL_CIRCUMFERENCE = MK4I_L2_WHEEL_DIAMETER_METERS * Math.PI;
  public static final double MK4I_L2_DRIVE_GEAR_RATIO =
      1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  public static final boolean MK4I_L2_DRIVE_MOTOR_INVERTED = true;
  public static final double MK4I_L2_ANGLE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
  public static final boolean MK4I_L2_ANGLE_MOTOR_INVERTED = true;
  public static final boolean MK4I_L2_CAN_CODER_INVERTED = false;

  /* MK4 L2 */

  /*
  	Wheel diameter is best determined empirically. Refer to this document for more information: !!!
  */
  public static final double MK4_L2_WHEEL_DIAMETER_METERS = 0.10033;
  public static final double MK4_L2_WHEEL_CIRCUMFERENCE = MK4_L2_WHEEL_DIAMETER_METERS * Math.PI;
  public static final double MK4_L2_DRIVE_GEAR_RATIO =
      1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  public static final boolean MK4_L2_DRIVE_MOTOR_INVERTED = true;
  public static final double MK4_L2_ANGLE_GEAR_RATIO = 1 / ((15.0 / 32.0) * (10.0 / 60.0));
  public static final boolean MK4_L2_ANGLE_MOTOR_INVERTED = false;
  public static final boolean MK4_L2_CAN_CODER_INVERTED = false;

  public static final double OPEN_LOOP_RAMP = 0.25;
  public static final double CLOSED_LOOP_RAMP = 0.0;

  /* Swerve Current Limiting */
  public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
  public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
  public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

  public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 50;
  public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
  public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

  /* PID SLOT INDEX */
  public static final int SLOT_INDEX = 0;

  /* Neutral Modes */
  public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
  public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
}
