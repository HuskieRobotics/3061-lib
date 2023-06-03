package frc.robot.subsystems.subsystem;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class SubsystemConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private SubsystemConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = true;
  public static final boolean TESTING = true;
  public static final String SUBSYSTEM_NAME = "Subsystem";

  public static final int MOTOR_CAN_ID = 6;
  public static final double GEAR_RATIO = 100.0;
  public static final boolean MOTOR_INVERTED = false;

  public static final double POSITION_PID_F = 0.0;
  public static final double POSITION_PID_P = 0.0;
  public static final double POSITION_PID_I = 0;
  public static final double POSITION_PID_D = 0;
  public static final double POSITION_PID_I_ZONE = 0;
  public static final double POSITION_PID_PEAK_OUTPUT = 1.0;
  public static final double POSITION_FEEDFORWARD = 0;

  public static final double CURRENT_LIMIT = 40;

  public static final StatorCurrentLimitConfiguration CURRENT_LIMIT_CONFIG =
      new StatorCurrentLimitConfiguration(true, CURRENT_LIMIT, 50, 0.5);

  public static final int TIMEOUT_MS = 30;
  public static final int SLOT_INDEX = 0;
}
