package frc.robot.subsystems.manipulator;

public class ManipulatorConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ManipulatorConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final String SUBSYSTEM_NAME = "Manipulator";

  public static final int MANIPULATOR_MOTOR_ID = 56;
  public static final int MANIPULATOR_IR_SENSOR_ID = 0;
  public static final int MANIPULATOR_IR_BACKUP_SENSOR_ID = 1;

  // the following are determined based on the mechanical design of the arm
  public static final boolean MANIPULATOR_MOTOR_INVERTED = false;
  public static final double MANIPULATOR_GEAR_RATIO = 1.0;

  // voltages are determined empirically through tuning
  public static final double MANIPULATOR_COLLECTION_VOLTAGE = 4.0;
  public static final double MANIPULATOR_RELEASE_VOLTAGE = 4.0;
  public static final double MANIPULATOR_EJECT_VOLTAGE = -12.0;

  // used to trigger state transitions
  public static final double COLLECTION_TIME_OUT = 2.0;
  public static final double EJECT_DURATION_SECONDS = 0.5;

  // current limits are determined based on current budget for the robot
  public static final double MANIPULATOR_MOTOR_PEAK_CURRENT_LIMIT = 40;
  public static final double COLLECTION_CURRENT_SPIKE_THRESHOLD_AMPS = 35.0;
  public static final double COLLECTION_CURRENT_TIME_THRESHOLD_SECONDS = 0.1;
}
