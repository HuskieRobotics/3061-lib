package frc.robot.subsystems.manipulator;

public class ManipulatorConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ManipulatorConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Manipulator";

  // invert motors
  public static final boolean MANIPULATOR_MOTOR_INVERTED = false;

  public static final int MANIPULATOR_MOTOR_ID = 56;
  public static final int MANIPULATOR_IR_SENSOR_ID = 0;
  public static final int MANIPULATOR_IR_BACKUP_SENSOR_ID = 1;

  public static final double MANIPULATOR_GEAR_RATIO = 1.0;

  public static final double MANIPULATOR_KP = 8.0;
  public static final double MANIPULATOR_KI = 0;
  public static final double MANIPULATOR_KD = 0;
  public static final double MANIPULATOR_KS = 0.5;
  public static final double MANIPULATOR_KV = 0.05;
  public static final double MANIPULATOR_KA = 0.01;

  public static final double MANIPULATOR_COLLECTION_VOLTAGE = 4.0;
  public static final double MANIPULATOR_RELEASE_VOLTAGE = 4.0;
  public static final double MANIPULATOR_EJECT_VOLTAGE = -12.0;

  // used for timer
  public static final double COLLECTION_TIME_OUT = 4.0;
  public static final double FIRST_EJECT_DURATION_SECONDS = 0.5;
  public static final double SECOND_INTAKE_SECONDS = 1.5;
  public static final double FINAL_EJECT_DURATION_SECONDS = 3.5;

  // current limits and spike thresholds
  public static final double MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT = 40;
  public static final double COLLECTION_CURRENT_SPIKE_THRESHOLD = 35.0;
}
