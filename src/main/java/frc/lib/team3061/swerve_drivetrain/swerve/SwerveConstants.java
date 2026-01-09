/*
 * Initially from https://github.com/Team364/BaseFalconSwerve
 */

package frc.lib.team3061.swerve_drivetrain.swerve;

public abstract class SwerveConstants {
  public abstract double getDriveGearRatio();

  public abstract boolean isDriveMotorInverted();

  public abstract double getAngleGearRatio();

  public abstract boolean isAngleMotorInverted();

  public abstract boolean isCanCoderInverted();

  public static final SwerveConstants MK4_L2_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK4_L2_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK4_L2_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK4_L2_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK4_L2_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK4_L2_CAN_CODER_INVERTED;
        }
      };

  public static final SwerveConstants MK4I_L2_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK4I_L2_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK4I_L2_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK4I_L2_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK4I_L2_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK4I_L2_CAN_CODER_INVERTED;
        }
      };

  public static final SwerveConstants MK4I_L3_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK4I_L3_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK4I_L3_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK4I_L3_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK4I_L3_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK4I_L3_CAN_CODER_INVERTED;
        }
      };

  public static final SwerveConstants MK4I_L3_PLUS_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK4I_L3_PLUS_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK4I_L3_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK4I_L3_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK4I_L3_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK4I_L3_CAN_CODER_INVERTED;
        }
      };

  public static final SwerveConstants MK4N_L3_PLUS_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK4N_L3_PLUS_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK4N_L3_PLUS_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK4N_L3_PLUS_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK4N_L3_PLUS_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK4N_L3_PLUS_CAN_CODER_INVERTED;
        }
      };

  public static final SwerveConstants MK4N_L2_PLUS_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK4N_L2_PLUS_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK4N_L2_PLUS_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK4N_L2_PLUS_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK4N_L2_PLUS_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK4N_L2_PLUS_CAN_CODER_INVERTED;
        }
      };

  public static final SwerveConstants MK5N_R1_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK5N_R1_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK5N_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK5N_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK5N_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK5N_CAN_CODER_INVERTED;
        }
      };

  public static final SwerveConstants MK5N_R2_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK5N_R2_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK5N_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK5N_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK5N_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK5N_CAN_CODER_INVERTED;
        }
      };

  public static final SwerveConstants MK5N_R3_CONSTANTS =
      new SwerveConstants() {
        @Override
        public double getDriveGearRatio() {
          return MK5N_R3_DRIVE_GEAR_RATIO;
        }

        @Override
        public boolean isDriveMotorInverted() {
          return MK5N_DRIVE_MOTOR_INVERTED;
        }

        @Override
        public double getAngleGearRatio() {
          return MK5N_ANGLE_GEAR_RATIO;
        }

        @Override
        public boolean isAngleMotorInverted() {
          return MK5N_ANGLE_MOTOR_INVERTED;
        }

        @Override
        public boolean isCanCoderInverted() {
          return MK5N_CAN_CODER_INVERTED;
        }
      };

  /* MK5n */
  private static final double MK5N_R1_DRIVE_GEAR_RATIO =
      1.0 / ((12.0 / 54.0) * (32.0 / 25.0) * (15.0 / 30.0));
  private static final double MK5N_R2_DRIVE_GEAR_RATIO =
      1.0 / ((14.0 / 54.0) * (32.0 / 25.0) * (15.0 / 30.0));
  private static final double MK5N_R3_DRIVE_GEAR_RATIO =
      1.0 / ((16.0 / 54.0) * (32.0 / 25.0) * (15.0 / 30.0));
  private static final boolean MK5N_DRIVE_MOTOR_INVERTED = false;
  private static final double MK5N_ANGLE_GEAR_RATIO = 287.0 / 11.0;
  private static final boolean MK5N_ANGLE_MOTOR_INVERTED = false;
  private static final boolean MK5N_CAN_CODER_INVERTED = false;

  /* MK4n L3 Plus */
  private static final double MK4N_L3_PLUS_DRIVE_GEAR_RATIO =
      1 / ((16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0));
  private static final boolean MK4N_L3_PLUS_DRIVE_MOTOR_INVERTED = true;
  private static final double MK4N_L3_PLUS_ANGLE_GEAR_RATIO = 1 / ((16.0 / 50.0) * (10.0 / 60.0));
  private static final boolean MK4N_L3_PLUS_ANGLE_MOTOR_INVERTED = true;
  private static final boolean MK4N_L3_PLUS_CAN_CODER_INVERTED = false;

  /* MK4n L2 Plus */
  // we might have to have differing inversions based on the shape of the mk4n's on the new robot
  private static final double MK4N_L2_PLUS_DRIVE_GEAR_RATIO =
      1 / ((16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  private static final boolean MK4N_L2_PLUS_DRIVE_MOTOR_INVERTED = true;
  private static final double MK4N_L2_PLUS_ANGLE_GEAR_RATIO =
      1 / ((16.0 / 50.0) * (10.0 / 60.0)); // same as L3+
  private static final boolean MK4N_L2_PLUS_ANGLE_MOTOR_INVERTED = true;
  private static final boolean MK4N_L2_PLUS_CAN_CODER_INVERTED = false;

  /* MK4i L3+ */

  private static final double MK4I_L3_PLUS_DRIVE_GEAR_RATIO =
      1 / ((16.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0));

  /* MK4i L3 */

  private static final double MK4I_L3_DRIVE_GEAR_RATIO =
      1 / ((14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0));
  private static final boolean MK4I_L3_DRIVE_MOTOR_INVERTED = true;
  private static final double MK4I_L3_ANGLE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
  private static final boolean MK4I_L3_ANGLE_MOTOR_INVERTED = true;
  private static final boolean MK4I_L3_CAN_CODER_INVERTED = false;

  /* MK4i L2 */

  private static final double MK4I_L2_DRIVE_GEAR_RATIO =
      1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  private static final boolean MK4I_L2_DRIVE_MOTOR_INVERTED = true;
  private static final double MK4I_L2_ANGLE_GEAR_RATIO = 1 / ((14.0 / 50.0) * (10.0 / 60.0));
  private static final boolean MK4I_L2_ANGLE_MOTOR_INVERTED = true;
  private static final boolean MK4I_L2_CAN_CODER_INVERTED = false;

  /* MK4 L2 */

  private static final double MK4_L2_DRIVE_GEAR_RATIO =
      1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
  private static final boolean MK4_L2_DRIVE_MOTOR_INVERTED = true;
  private static final double MK4_L2_ANGLE_GEAR_RATIO = 1 / ((15.0 / 32.0) * (10.0 / 60.0));
  private static final boolean MK4_L2_ANGLE_MOTOR_INVERTED = false;
  private static final boolean MK4_L2_CAN_CODER_INVERTED = false;

  public static final double OPEN_LOOP_RAMP = 0.0;
  public static final double CLOSED_LOOP_RAMP = 0.0;

  /* Swerve Current Limiting */
  public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
  public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
  public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

  public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final int DRIVE_PEAK_CURRENT_LIMIT = 80;
  public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
}
