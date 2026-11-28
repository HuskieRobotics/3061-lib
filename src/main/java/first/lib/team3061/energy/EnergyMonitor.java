// inspired by 6328's BatteryLogger class
// (https://github.com/Mechanical-Advantage/RobotCode2026Public)

package first.lib.team3061.energy;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import first.lib.team254.Phoenix6Util;
import first.lib.team3061.bfg.BFG;
import first.lib.team6328.energy.BreakerModel;
import first.robot.Constants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Current;

/**
 * Tracks the current, power, and energy used by groups of motors (e.g., a subsystem's motors) to
 * help tune current limits. It also models the main breaker.
 *
 * <p>Every TalonFX registered with {@link first.lib.team3015.subsystem.FaultReporter} is tracked;
 * no additional code is required in subsystems. By default, a motor's group is the name of the
 * subsystem with which it is registered. The swerve drivetrain places its drive and steer motors in
 * separate groups ("Drive" and "Steer").
 *
 * <p>The following are logged as inputs every robot loop:
 *
 * <ul>
 *   <li>Energy/ControllerVoltageVolts: the battery voltage measured by the robot controller
 *   <li>Energy/[group]/Amps: the sum of the supply currents of the group's motors; supply current
 *       is negative while a motor is regenerating
 *   <li>Energy/[group]/AtSupplyLimit: true if any of the group's motors is limiting its supply
 *       current
 *   <li>Energy/[group]/AtStatorLimit: true if any of the group's motors is limiting its stator
 *       current
 *   <li>Energy/[group]/MotorCount: the number of motors in the group
 * </ul>
 *
 * <p>The following are calculated and logged as outputs every robot loop:
 *
 * <ul>
 *   <li>Energy/VoltageVolts: the voltage used to calculate power: the voltage at the battery's
 *       terminals measured by the BFG if it is connected; otherwise, the robot controller's
 *       voltage. The robot controller measures the voltage after the main wires, main breaker, and
 *       power distribution panel; so, it is lower than the battery's voltage when current is high,
 *       and power calculated from it doesn't include the power lost in the wiring.
 *   <li>Energy/[group]/Watts: the group's power (amps * Energy/VoltageVolts)
 *   <li>Energy/[group]/Joules: the group's energy since the robot code started. This total never
 *       resets; analyze the increase over a period of time (e.g., a match).
 *   <li>Energy/Unaccounted/Amps, Watts, and Joules: when the Battery Fuel Gauge (BFG) is connected,
 *       the current measured at the battery less the sum of the groups' currents. This is the
 *       current used by everything that isn't a registered TalonFX (e.g., the robot controller,
 *       radio, coprocessors, LEDs, and other motor controllers). It may be briefly negative since
 *       the BFG and the motors measure current at slightly different times.
 *   <li>Energy/BreakerAmps: the current used to model the main breaker: the BFG's current if it is
 *       connected; otherwise, the sum of the groups' currents, which underestimates the total.
 *   <li>Energy/BreakerDamage: the main breaker's modeled damage, from 0 (cool) to 1 (the breaker
 *       would trip)
 * </ul>
 *
 * <p>The following estimates are also logged as outputs. They hold their previous values while the
 * current doesn't vary enough to estimate them (e.g., while the robot is idle). EstimateValid is
 * false until the first estimate, and the other values aren't logged until then. See {@link
 * ResistanceEstimator}.
 *
 * <ul>
 *   <li>Energy/Estimates/BatteryAndWiring/ResistanceOhms, RestingVoltageVolts, and EstimateValid:
 *       the resistance of the battery and wiring combined and the battery's resting voltage, from
 *       the robot controller's voltage and the total current (the BFG's current if it is connected;
 *       otherwise, the sum of the groups' currents). Brownouts depend on these. Without the BFG,
 *       loads that aren't measured shift the resting voltage but not the resistance.
 *   <li>Energy/Estimates/Battery/ResistanceOhms, RestingVoltageVolts, and EstimateValid: the
 *       battery's internal resistance and resting voltage, from the BFG's voltage and current. The
 *       battery is identified by the BFG's inputs (BFG/Battery/SerialNumber and Nickname).
 *   <li>Energy/Estimates/Wiring/ResistanceOhms and EstimateValid: the resistance of the main wires,
 *       main breaker, and power distribution panel, from the difference between the BFG's and the
 *       robot controller's voltages and the BFG's current.
 * </ul>
 *
 * <p>A current budget and a dynamic drive current limit are calculated and logged, but not applied
 * to any motors:
 *
 * <ul>
 *   <li>Energy/Budget/BatteryMaxAmps: the current at which the robot controller's voltage would
 *       fall to the minimum allowed voltage, based on the battery-and-wiring estimate
 *   <li>Energy/Budget/BreakerMaxAmps: the current that the main breaker can carry for the next few
 *       seconds without its modeled damage reaching the maximum allowed
 *   <li>Energy/Budget/TotalAmps: the lower of these two, with headroom, and capped
 *   <li>Energy/Budget/DriveAmps: the budget less the current of everything except the drive motors
 *   <li>Energy/Budget/DriveLimitAmps: the drive budget divided among the drive motors, rounded down
 *       to a step and no less than a minimum; compare it to the drive motors' supply current limit
 * </ul>
 *
 * <p>Since every group's energy is logged as Energy/[group]/Joules, the sum of these entries is the
 * total energy used by the robot. To keep that true, nothing else is logged as
 * Energy/[name]/Joules. When the BFG is connected, this sum is the energy delivered by the battery
 * (the BFG's current * the BFG's voltage).
 *
 * <p>The motors' currents are logged as inputs so that a replayed log recalculates the same power
 * and energy. When replaying a log, the groups are determined from the log.
 *
 * <p>Like {@link BFG}, the energy monitor is a singleton subsystem so that the command scheduler
 * invokes its periodic method every robot loop. It must be created after the BFG so that the BFG's
 * current is updated first.
 */
public class EnergyMonitor extends SubsystemBase {

  /**
   * The name of the group of a swerve drivetrain's drive motors, which the dynamic drive current
   * limit is for.
   */
  public static final String DRIVE_GROUP = "Drive";

  /** The name of the group of a swerve drivetrain's steer motors. */
  public static final String STEER_GROUP = "Steer";

  private static final String SUBSYSTEM_NAME = "Energy";
  private static final String UNACCOUNTED_GROUP = "Unaccounted";
  private static final String ESTIMATES = SUBSYSTEM_NAME + "/Estimates/";
  private static final String BUDGET = SUBSYSTEM_NAME + "/Budget/";

  // resistance estimates are fit over this many seconds of samples, and only when the current's
  // standard deviation within that window is at least this large
  private static final double ESTIMATOR_WINDOW_SECS = 2.0;
  private static final double ESTIMATOR_MIN_CURRENT_STD_DEV_AMPS = 5.0;
  private static final double MAX_BATTERY_RESISTANCE_OHMS = 0.1;
  private static final double MAX_WIRING_RESISTANCE_OHMS = 0.05;

  /*
   * Budget parameters, from 6328's 2026 robot code. The minimum allowed voltage should be above the
   * brownout voltage set for the robot controller (RobotController.setBrownoutVoltages), which
   * can't be read back.
   */
  private static final double MIN_ALLOWED_VOLTAGE_VOLTS = 7.0;
  private static final double BREAKER_HORIZON_SECS = 3.0;
  private static final double MAX_BREAKER_DAMAGE = 0.95;
  private static final double BUDGET_HEADROOM = 0.9;
  private static final double MAX_BUDGET_AMPS = 190.0;
  private static final double MIN_DRIVE_LIMIT_AMPS = 5.0;
  private static final double DRIVE_LIMIT_STEP_AMPS = 0.5;

  /*
   * By default, the current-limit signals are only sent at 4 Hz on any bus. This is too slow to
   * measure how often current is limited; so, their update frequency is increased to at least this
   * value. On SystemCore's CAN FD buses, the supply current is sent at 100 Hz by default; so, it
   * isn't changed.
   */
  private static final double MIN_SIGNAL_UPDATE_FREQUENCY_HZ = 50.0;

  private static EnergyMonitor instance;

  /**
   * Returns the singleton instance of this class, creating it on the first call.
   *
   * @return the singleton instance of this class
   */
  public static EnergyMonitor getInstance() {
    if (instance == null) {
      instance = new EnergyMonitor();
    }
    return instance;
  }

  /** The status signals of a registered motor and the group to which it belongs. */
  private static class MotorSignals {
    private final String group;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Boolean> supplyCurrentLimitFault;
    private final StatusSignal<Boolean> statorCurrentLimitFault;

    private MotorSignals(String group, TalonFX motor) {
      this.group = group;
      this.supplyCurrent = motor.getSupplyCurrent();
      this.supplyCurrentLimitFault = motor.getFault_SupplyCurrLimit();
      this.statorCurrentLimitFault = motor.getFault_StatorCurrLimit();
    }
  }

  /** The inputs of a group of motors. */
  private static class GroupInputs {
    private double amps = 0.0;
    private boolean atSupplyLimit = false;
    private boolean atStatorLimit = false;
    private int motorCount = 0;
  }

  /**
   * The energy monitor's inputs. The groups aren't known at compile time; so, these inputs are
   * logged with a custom implementation of LoggableInputs rather than with the AutoLog annotation.
   */
  private static class EnergyMonitorInputs implements LoggableInputs {
    private static final String AMPS = "Amps";
    private static final String AT_SUPPLY_LIMIT = "AtSupplyLimit";
    private static final String AT_STATOR_LIMIT = "AtStatorLimit";
    private static final String MOTOR_COUNT = "MotorCount";

    private double controllerVoltageVolts = 0.0;
    private final Map<String, GroupInputs> groups = new TreeMap<>();

    @Override
    public void toLog(LogTable table) {
      table.put("ControllerVoltageVolts", controllerVoltageVolts);
      for (Map.Entry<String, GroupInputs> entry : groups.entrySet()) {
        LogTable groupTable = table.getSubtable(entry.getKey());
        groupTable.put(AMPS, entry.getValue().amps);
        groupTable.put(AT_SUPPLY_LIMIT, entry.getValue().atSupplyLimit);
        groupTable.put(AT_STATOR_LIMIT, entry.getValue().atStatorLimit);
        groupTable.put(MOTOR_COUNT, entry.getValue().motorCount);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      controllerVoltageVolts = table.get("ControllerVoltageVolts", controllerVoltageVolts);

      // determine the groups from the log
      String ampsSuffix = "/" + AMPS;
      for (String key : table.getAll(true).keySet()) {
        if (key.endsWith(ampsSuffix)) {
          String group = key.substring(0, key.length() - ampsSuffix.length());
          GroupInputs groupInputs = groups.computeIfAbsent(group, g -> new GroupInputs());
          LogTable groupTable = table.getSubtable(group);
          groupInputs.amps = groupTable.get(AMPS, groupInputs.amps);
          groupInputs.atSupplyLimit = groupTable.get(AT_SUPPLY_LIMIT, groupInputs.atSupplyLimit);
          groupInputs.atStatorLimit = groupTable.get(AT_STATOR_LIMIT, groupInputs.atStatorLimit);
          groupInputs.motorCount = groupTable.get(MOTOR_COUNT, groupInputs.motorCount);
        }
      }
    }
  }

  private final List<MotorSignals> motors = new ArrayList<>();

  private final EnergyMonitorInputs inputs = new EnergyMonitorInputs();
  private final Map<String, Double> joules = new HashMap<>();
  private final BreakerModel breakerModel = new BreakerModel();
  private long lastTimestampNanos = -1;
  private double totalGroupAmps = 0.0;

  private static final int ESTIMATOR_WINDOW_SIZE =
      (int) Math.round(ESTIMATOR_WINDOW_SECS / Constants.LOOP_PERIOD_SECS);
  private final ResistanceEstimator batteryAndWiringEstimator =
      new ResistanceEstimator(
          ESTIMATOR_WINDOW_SIZE,
          ESTIMATOR_MIN_CURRENT_STD_DEV_AMPS,
          MAX_BATTERY_RESISTANCE_OHMS + MAX_WIRING_RESISTANCE_OHMS);
  private final ResistanceEstimator batteryEstimator =
      new ResistanceEstimator(
          ESTIMATOR_WINDOW_SIZE, ESTIMATOR_MIN_CURRENT_STD_DEV_AMPS, MAX_BATTERY_RESISTANCE_OHMS);
  private final ResistanceEstimator wiringEstimator =
      new ResistanceEstimator(
          ESTIMATOR_WINDOW_SIZE, ESTIMATOR_MIN_CURRENT_STD_DEV_AMPS, MAX_WIRING_RESISTANCE_OHMS);

  private EnergyMonitor() {
    super(SUBSYSTEM_NAME);
  }

  /**
   * Registers a motor whose current, power, and energy will be included in the specified group.
   * This method is invoked by {@link first.lib.team3015.subsystem.FaultReporter} when a TalonFX is
   * registered; it should not need to be invoked directly.
   *
   * <p>If the motor's current-limit signals are sent less frequently than 50 Hz, their update
   * frequency is increased to 50 Hz.
   *
   * @param group the name of the group; it is used as a single level in the logged entries' names;
   *     so, any '/' characters are replaced with '-'
   * @param motor the motor to register
   */
  public void registerMotor(String group, TalonFX motor) {
    if (Constants.getMode() == Constants.Mode.REPLAY) {
      return;
    }

    String groupName = group.replace('/', '-');
    MotorSignals motorSignals = new MotorSignals(groupName, motor);
    motors.add(motorSignals);
    inputs.groups.computeIfAbsent(groupName, g -> new GroupInputs()).motorCount++;

    for (BaseStatusSignal signal :
        List.of(motorSignals.supplyCurrentLimitFault, motorSignals.statorCurrentLimitFault)) {
      if (signal.getAppliedUpdateFrequency() < MIN_SIGNAL_UPDATE_FREQUENCY_HZ) {
        signal.setUpdateFrequency(MIN_SIGNAL_UPDATE_FREQUENCY_HZ);
      }
    }

    // these signals are refreshed with all other registered signals at the start of each robot
    // loop, before the command scheduler invokes this subsystem's periodic method
    Phoenix6Util.registerSignals(
        true,
        motorSignals.supplyCurrent,
        motorSignals.supplyCurrentLimitFault,
        motorSignals.statorCurrentLimitFault);
  }

  /**
   * Updates and logs the inputs and calculates and logs the power, energy, and breaker damage. The
   * command scheduler invokes this method every robot loop.
   */
  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    // use the actual time since the previous loop, which is longer than the nominal loop period if
    // the loop overruns
    long timestampNanos = Logger.getTimestamp();
    double periodSecs =
        lastTimestampNanos < 0 ? 0.0 : (timestampNanos - lastTimestampNanos) / 1.0e9;
    lastTimestampNanos = timestampNanos;

    // the BFG measures the voltage at the battery's terminals, which, unlike the robot controller's
    // voltage, includes the voltage dropped across the main wires, breaker, and power distribution
    BFG bfg = BFG.getInstance();
    double voltageVolts = bfg.isConnected() ? bfg.getVoltageVolts() : inputs.controllerVoltageVolts;
    Logger.recordOutput(SUBSYSTEM_NAME + "/VoltageVolts", voltageVolts);

    totalGroupAmps = 0.0;
    for (Map.Entry<String, GroupInputs> entry : inputs.groups.entrySet()) {
      totalGroupAmps += entry.getValue().amps;
      logPowerAndEnergy(entry.getKey(), entry.getValue().amps, voltageVolts, periodSecs);
    }

    double breakerAmps;
    if (bfg.isConnected()) {
      double unaccountedAmps = bfg.getCurrentAmps() - totalGroupAmps;
      Logger.recordOutput(SUBSYSTEM_NAME + "/" + UNACCOUNTED_GROUP + "/Amps", unaccountedAmps);
      logPowerAndEnergy(UNACCOUNTED_GROUP, unaccountedAmps, voltageVolts, periodSecs);
      breakerAmps = bfg.getCurrentAmps();
    } else {
      breakerAmps = totalGroupAmps;
    }

    breakerModel.update(breakerAmps, periodSecs);
    Logger.recordOutput(SUBSYSTEM_NAME + "/BreakerAmps", breakerAmps);
    Logger.recordOutput(SUBSYSTEM_NAME + "/BreakerDamage", breakerModel.getDamage());

    updateEstimatesAndBudget(bfg, breakerAmps);
  }

  /**
   * Updates and logs the resistance estimates, the current budget, and the dynamic drive current
   * limit. The budget and limit are only logged; they aren't applied to any motors.
   *
   * @param bfg the BFG
   * @param totalAmps the robot's total current: the BFG's current if it is connected; otherwise,
   *     the sum of the groups' currents
   */
  private void updateEstimatesAndBudget(BFG bfg, double totalAmps) {
    // the robot controller's voltage depends on the resistance of both the battery and the wiring,
    // which is what determines brownouts; this estimate doesn't require the BFG
    batteryAndWiringEstimator.update(totalAmps, inputs.controllerVoltageVolts);

    if (bfg.isConnected()) {
      batteryEstimator.update(bfg.getCurrentAmps(), bfg.getVoltageVolts());
      // controller voltage = BFG voltage - wiring resistance * current; so, fitting the difference
      // between the voltages against the current yields the wiring resistance
      wiringEstimator.update(
          bfg.getCurrentAmps(), inputs.controllerVoltageVolts - bfg.getVoltageVolts());
    }

    logEstimate("BatteryAndWiring", batteryAndWiringEstimator, true);
    logEstimate("Battery", batteryEstimator, true);
    logEstimate("Wiring", wiringEstimator, false);

    // the current at which the controller's voltage would fall to the minimum allowed voltage
    double batteryMaxAmps =
        batteryAndWiringEstimator.isValid()
            ? Math.max(
                0.0,
                (batteryAndWiringEstimator.getRestingVoltageVolts() - MIN_ALLOWED_VOLTAGE_VOLTS)
                    / batteryAndWiringEstimator.getResistanceOhms())
            : MAX_BUDGET_AMPS;
    double breakerMaxAmps =
        breakerModel.calculateMaxCurrent(BREAKER_HORIZON_SECS, MAX_BREAKER_DAMAGE);
    double budgetAmps =
        Math.min(Math.min(batteryMaxAmps, breakerMaxAmps) * BUDGET_HEADROOM, MAX_BUDGET_AMPS);

    Logger.recordOutput(BUDGET + "BatteryMaxAmps", batteryMaxAmps);
    Logger.recordOutput(BUDGET + "BreakerMaxAmps", breakerMaxAmps);
    Logger.recordOutput(BUDGET + "TotalAmps", budgetAmps);

    // the drive motors get whatever is left after everything else's current in this loop
    GroupInputs drive = inputs.groups.get(DRIVE_GROUP);
    if (drive != null && drive.motorCount > 0) {
      double driveBudgetAmps = Math.max(0.0, budgetAmps - (totalAmps - drive.amps));
      double driveLimitAmps =
          Math.floor(
                  Math.max(driveBudgetAmps / drive.motorCount, MIN_DRIVE_LIMIT_AMPS)
                      / DRIVE_LIMIT_STEP_AMPS)
              * DRIVE_LIMIT_STEP_AMPS;
      Logger.recordOutput(BUDGET + "DriveAmps", driveBudgetAmps);
      Logger.recordOutput(BUDGET + "DriveLimitAmps", driveLimitAmps);
    }
  }

  /**
   * Logs the specified resistance estimate.
   *
   * @param name the name of the estimate
   * @param estimator the estimator
   * @param logRestingVoltage whether to log the resting voltage
   */
  private static void logEstimate(
      String name, ResistanceEstimator estimator, boolean logRestingVoltage) {
    Logger.recordOutput(ESTIMATES + name + "/EstimateValid", estimator.isValid());

    // the estimates aren't logged until they are valid so that log analysis (e.g., the mean over a
    // match) isn't skewed by the placeholder values
    if (estimator.isValid()) {
      Logger.recordOutput(ESTIMATES + name + "/ResistanceOhms", estimator.getResistanceOhms());
      if (logRestingVoltage) {
        Logger.recordOutput(
            ESTIMATES + name + "/RestingVoltageVolts", estimator.getRestingVoltageVolts());
      }
    }
  }

  /**
   * Returns the sum of the groups' currents in the most recent robot loop. This is used by the
   * simulated BFG.
   *
   * @return the sum of the groups' currents in amps
   */
  public double getMotorAmps() {
    return totalGroupAmps;
  }

  /**
   * Returns the modeled damage of the main breaker, from 0 (cool) to 1 (the breaker would trip).
   *
   * @return the modeled damage of the main breaker
   */
  public double getBreakerDamage() {
    return breakerModel.getDamage();
  }

  /** Reads the registered motors' signals and sums them for each group. */
  private void updateInputs() {
    if (Constants.getMode() == Constants.Mode.REPLAY) {
      return;
    }

    inputs.controllerVoltageVolts = RobotController.getBatteryVoltage();

    for (GroupInputs groupInputs : inputs.groups.values()) {
      groupInputs.amps = 0.0;
      groupInputs.atSupplyLimit = false;
      groupInputs.atStatorLimit = false;
    }

    for (MotorSignals motor : motors) {
      GroupInputs groupInputs = inputs.groups.get(motor.group);
      groupInputs.amps += motor.supplyCurrent.getValueAsDouble();
      groupInputs.atSupplyLimit |= motor.supplyCurrentLimitFault.getValue();
      groupInputs.atStatorLimit |= motor.statorCurrentLimitFault.getValue();
    }
  }

  /**
   * Calculates and logs the power and energy of the specified group.
   *
   * @param group the name of the group
   * @param amps the group's current in this robot loop
   * @param voltageVolts the voltage with which to calculate power
   * @param periodSecs the time since the previous robot loop
   */
  private void logPowerAndEnergy(
      String group, double amps, double voltageVolts, double periodSecs) {
    double watts = amps * voltageVolts;
    double totalJoules = joules.merge(group, watts * periodSecs, Double::sum);
    Logger.recordOutput(SUBSYSTEM_NAME + "/" + group + "/Watts", watts);
    Logger.recordOutput(SUBSYSTEM_NAME + "/" + group + "/Joules", totalJoules);
  }
}
