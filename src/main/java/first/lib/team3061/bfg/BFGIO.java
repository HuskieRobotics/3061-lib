package first.lib.team3061.bfg;

import org.littletonrobotics.junction.AutoLog;

@java.lang.SuppressWarnings({"java:S1104"})

/**
 * Battery Fuel Gauge (BFG) hardware abstraction interface.
 *
 * <p>The Playing With Fusion BFG stays with its battery. It measures the battery's terminal voltage
 * and current, and it stores data about the battery, its most recent charge cycle, and the most
 * recent match. Data about the battery and its charge cycle doesn't change while the robot is
 * running, and data about the match is only complete after the match ends; so, each is read once,
 * into its own set of inputs, rather than every robot loop.
 *
 * <p>Units are in the names of the fields so that they are in the names of the logged entries.
 */
public interface BFGIO {
  /** Inputs that change while the robot is running. These are updated every robot loop. */
  @AutoLog
  public static class BFGIOInputs {
    public boolean connected = false;
    public double currentAmps = 0.0;
    public double voltageVolts = 0.0;
    public double dodAh = 0.0;
    public double dodWh = 0.0;
    public double remainingChargePercent = 0.0;
    public double effectiveCapacityAh = 0.0;
    public String chargeState = "";
  }

  /** Inputs about the battery and its most recent charge cycle. These are updated once. */
  @AutoLog
  public static class BFGBatteryIOInputs {
    public long serialNumber = 0;
    public long firmwareVersion = 0;
    public String nickname = "";
    public String manufacturer = "";
    public double ageDays = 0.0;
    public int numCycles = 0;
    public double capacityAh = 0.0;
    public double ratedCapacityAh = 0.0;

    public double cycleChargeTimeSecs = 0.0;
    public double cycleDischargeTimeSecs = 0.0;
    public double cycleMaxCurrentAmps = 0.0;
    public double cycleMinCurrentAmps = 0.0;
    public double cycleRmsDischargeCurrentAmps = 0.0;
    public double cycleMaxVoltageVolts = 0.0;
    public double cycleMinVoltageVolts = 0.0;
    public double cycleStartingVoltageVolts = 0.0;
    public double cycleMinDodAh = 0.0;
    public double cycleMaxDodAh = 0.0;
    public double cycleMinDodWh = 0.0;
    public double cycleMaxDodWh = 0.0;
  }

  /** Inputs about the most recent match. These are updated once, after the match ends. */
  @AutoLog
  public static class BFGMatchIOInputs {
    public double durationSecs = 0.0;
    public double chargeTimeSecs = 0.0;
    public double dischargeTimeSecs = 0.0;
    public double maxCurrentAmps = 0.0;
    public double minCurrentAmps = 0.0;
    public double rmsDischargeCurrentAmps = 0.0;
    public double maxVoltageVolts = 0.0;
    public double minVoltageVolts = 0.0;
    public double startingVoltageVolts = 0.0;
    public double minDodAh = 0.0;
    public double maxDodAh = 0.0;
    public double startingChargeAh = 0.0;
    public double endingChargeAh = 0.0;
    public double startingEnergyWh = 0.0;
    public double endingEnergyWh = 0.0;
  }

  /**
   * Updates the inputs that change while the robot is running.
   *
   * @param inputs the inputs to update
   */
  public default void updateInputs(BFGIOInputs inputs) {}

  /**
   * Updates the inputs about the battery and its most recent charge cycle.
   *
   * @param inputs the inputs to update
   */
  public default void updateBatteryInputs(BFGBatteryIOInputs inputs) {}

  /**
   * Updates the inputs about the most recent match.
   *
   * @param inputs the inputs to update
   */
  public default void updateMatchInputs(BFGMatchIOInputs inputs) {}
}
