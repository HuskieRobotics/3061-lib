package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {

  public static final String SUBSYSTEM_NAME = "Elevator";

  public static final int LEAD_MOTOR_ID = 10;
  public static final int FOLLOWER_MOTOR_ID = 11;

  // PID constants are determined empirically through tuning
  public static final double KP_SLOT0 = 40.0;
  public static final double KI_SLOT0 = 0;
  public static final double KD_SLOT0 = 0;

  // feed forward constants are determined through running SysId commands and analyzing the results
  // in SysId
  public static final double KS_SLOT0 = 0.01;
  public static final double KV_SLOT0 = 0.67505;
  public static final double KA_SLOT0 = 0.027564;
  public static final double KG_SLOT0 = 0.33833;

  // Motion magic constants are determined empirically through tuning
  public static final double KV_EXPO = 0.6;
  public static final double KA_EXPO = 0.15;
  public static final double CRUISE_VELOCITY = 0; // don't limit the cruise velocity

  // the following are determined based on the mechanical design of the elevator
  public static final boolean IS_INVERTED = true;
  public static final Distance PULLEY_CIRCUMFERENCE = Inches.of(5.9055);
  public static final int GEAR_RATIO = 5;
  public static final double ELEVATOR_MASS_KG = 4.5;
  public static final Distance MAX_HEIGHT = Inches.of(74);
  public static final Distance MIN_HEIGHT = Inches.of(0.0);
  public static final Distance LINEAR_POSITION_TOLERANCE = Inches.of(0.25);

  public static final Voltage ELEVATOR_RAISE_SLOW_VOLTAGE = Volts.of(2.0);
  public static final Voltage ELEVATOR_LOWERING_SLOW_VOLTAGE = Volts.of(-2.0);

  // This is the current we watch for to detect that the elevator is jammed and needs to be stopped.
  public static final double JAMMED_CURRENT_AMPS = 59.0;
  public static final double JAMMED_TIME_THRESHOLD_SECONDS = 0.1;

  // Supply current limits are determined based on current budget for the robot and stator current
  // limits are determined based on protecting the mechanism (e.g., don't break the elevator belt).
  public static final double ELEVATOR_PEAK_CURRENT_LIMIT = 60.0;

  // example elevator presets of an enumerated value and the associated Distance
  public enum Positions {
    BOTTOM,
    MIDDLE,
    TOP
  }

  public static final Distance BOTTOM_HEIGHT = Inches.of(0.0);
  public static final Distance MIDDLE_HEIGHT = Inches.of(45.0);
  public static final Distance TOP_HEIGHT = Inches.of(70.0);
}
