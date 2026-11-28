package first.lib.bfg;

import java.util.function.DoubleSupplier;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.system.Timer;

/**
 * Simulated BFG IO, which models the battery and the robot's wiring with known resistances.
 *
 * <p>The current drawn from the battery is the sum of the simulated motors' supply currents plus a
 * constant current for everything else (the robot controller, radio, etc.). The battery's terminal
 * voltage is its resting voltage less the voltage dropped across its internal resistance, and the
 * robot controller's voltage is the terminal voltage less the voltage dropped across the wiring.
 * The robot controller's simulated voltage is set to the latter; so, simulated motors see the
 * battery's voltage sag under load, and the energy monitor's resistance estimates can be checked
 * against the known resistances.
 *
 * <p>The resting voltage doesn't change as the battery discharges, and the motors' current is from
 * the previous robot loop since the BFG is updated before the motors.
 */
public class BFGIOSim implements BFGIO {
  private static final double RESTING_VOLTAGE_VOLTS = 12.8;
  private static final double BATTERY_RESISTANCE_OHMS = 0.015;
  private static final double WIRING_RESISTANCE_OHMS = 0.005;
  private static final double OTHER_LOADS_AMPS = 3.0;
  private static final double CAPACITY_AH = 18.0;
  private static final double SECONDS_PER_HOUR = 3600.0;

  private final DoubleSupplier motorAmpsSupplier;

  private double dodAh = 0.0;
  private double dodWh = 0.0;
  private double lastTimestampSecs = -1.0;

  /**
   * Creates a new simulated BFG IO.
   *
   * @param motorAmpsSupplier supplies the sum of the simulated motors' supply currents in amps
   */
  public BFGIOSim(DoubleSupplier motorAmpsSupplier) {
    this.motorAmpsSupplier = motorAmpsSupplier;
  }

  @Override
  public void updateInputs(BFGIOInputs inputs) {
    double currentAmps = motorAmpsSupplier.getAsDouble() + OTHER_LOADS_AMPS;
    double batteryVoltageVolts = RESTING_VOLTAGE_VOLTS - BATTERY_RESISTANCE_OHMS * currentAmps;
    double controllerVoltageVolts = batteryVoltageVolts - WIRING_RESISTANCE_OHMS * currentAmps;
    RoboRioSim.setVInVoltage(Math.max(0.0, controllerVoltageVolts));

    double timestampSecs = Timer.getTimestamp();
    if (lastTimestampSecs >= 0.0) {
      double periodSecs = timestampSecs - lastTimestampSecs;
      dodAh += currentAmps * periodSecs / SECONDS_PER_HOUR;
      dodWh += batteryVoltageVolts * currentAmps * periodSecs / SECONDS_PER_HOUR;
    }
    lastTimestampSecs = timestampSecs;

    inputs.connected = true;
    inputs.currentAmps = currentAmps;
    inputs.voltageVolts = batteryVoltageVolts;
    inputs.dodAh = dodAh;
    inputs.dodWh = dodWh;
    inputs.effectiveCapacityAh = CAPACITY_AH;
    inputs.remainingChargePercent = (CAPACITY_AH - dodAh) / CAPACITY_AH * 100.0;
    inputs.chargeState = "Discharge";
  }

  @Override
  public void updateBatteryInputs(BFGBatteryIOInputs inputs) {
    inputs.nickname = "Simulated";
    inputs.manufacturer = "Energizer";
    inputs.capacityAh = CAPACITY_AH;
    inputs.ratedCapacityAh = CAPACITY_AH;
  }
}
