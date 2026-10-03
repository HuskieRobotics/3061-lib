package frc.lib.team3061.bfg;

import com.playingwithfusion.BattFuelGauge;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class BFG {

  private static BFG instance;

  /**
   * Returns the singleton instance of this class, creating it with a sensor CAN ID of 0 on the
   * first call.
   *
   * @return the singleton instance of this class
   */
  public static BFG getInstance() {
    if (instance == null) {
      instance = new BFG(0);
    }
    return instance;
  }

  private static final String SUBSYSTEM_NAME = "BFG";

  private BattFuelGauge sensor;
  private boolean hasLoggedInitialData = false;
  private boolean hasLoggedFinalData = false;
  private boolean hasTeleopStarted = false;

  /**
   * Creates a new BFG object that reads data from the Playing With Fusion Battery Fuel Gauge with
   * the specified CAN ID.
   *
   * @param sensorId the CAN ID of the Battery Fuel Gauge
   */
  public BFG(int sensorId) {
    sensor = new BattFuelGauge(sensorId);
  }

  /**
   * Logs the battery data. This method should be invoked every robot loop. Data is only logged when
   * running on a real robot.
   */
  public void logData() {
    if (Constants.getMode() == Mode.REAL) {
      logInitialData();
      logFinalData();
      logPeriodicData();
    }
  }

  /**
   * Logs data about the battery and its most recent charge cycle. This data doesn't change while
   * the robot is running, so it is logged only once, the first time the sensor is connected.
   */
  private void logInitialData() {
    if (!hasLoggedInitialData && sensor.isConnected()) {
      hasLoggedInitialData = true;

      Logger.recordOutput(SUBSYSTEM_NAME + "/batteryAgeDays", sensor.getBatteryAgeDays());
      Logger.recordOutput(SUBSYSTEM_NAME + "/capacityAh", sensor.getCapacityAh());
      Logger.recordOutput(SUBSYSTEM_NAME + "/cycle/chargeTime", sensor.getCycleChargeTime());
      Logger.recordOutput(SUBSYSTEM_NAME + "/cycle/dischargeTime", sensor.getCycleDischargeTime());
      Logger.recordOutput(SUBSYSTEM_NAME + "/cycle/maxCurrent", sensor.getCycleMaxCurrent());
      Logger.recordOutput(SUBSYSTEM_NAME + "/cycle/maxVoltage", sensor.getCycleMaxVoltage());
      Logger.recordOutput(SUBSYSTEM_NAME + "/cycle/minCurrent", sensor.getCycleMinCurrent());
      Logger.recordOutput(SUBSYSTEM_NAME + "/cycle/minDodAh", sensor.getCycleMinDodAh());
      Logger.recordOutput(SUBSYSTEM_NAME + "/cycle/minDodWh", sensor.getCycleMinDodWh());
      Logger.recordOutput(SUBSYSTEM_NAME + "/cycle/minVoltage", sensor.getCycleMinVoltage());
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/cycle/rmsDischargeCurrent", sensor.getCycleRmsDischargeCurrent());
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/cycle/startingVoltage", sensor.getCycleStartingVoltage());
      Logger.recordOutput(SUBSYSTEM_NAME + "/effectiveCapacityAh", sensor.getEffectiveCapacityAh());
      Logger.recordOutput(SUBSYSTEM_NAME + "/firmwareVersion", sensor.getFirmwareVersion());
      Logger.recordOutput(SUBSYSTEM_NAME + "/manufacturer", sensor.getManufacturer());
      Logger.recordOutput(SUBSYSTEM_NAME + "/nickname", sensor.getNickname());
      Logger.recordOutput(SUBSYSTEM_NAME + "/numCycles", sensor.getNumCycles());
      Logger.recordOutput(SUBSYSTEM_NAME + "/ratedCapacityAh", sensor.getRatedCapacityAh());
      Logger.recordOutput(SUBSYSTEM_NAME + "/remainingChargePct", sensor.getRemainingChargePct());
      Logger.recordOutput(SUBSYSTEM_NAME + "/serialNumber", sensor.getSerialNumber());
    }
  }

  /**
   * Logs data about the most recent match. This data is logged only once: when the robot is
   * disabled after teleop has been enabled while connected to the FMS.
   */
  private void logFinalData() {
    if (DriverStation.isFMSAttached() && !hasTeleopStarted && DriverStation.isTeleopEnabled()) {
      hasTeleopStarted = true;
    }

    if (DriverStation.isDisabled()
        && DriverStation.isFMSAttached()
        && hasTeleopStarted
        && !hasLoggedFinalData
        && sensor.isConnected()) {
      hasLoggedFinalData = true;

      Logger.recordOutput(SUBSYSTEM_NAME + "/match/chargeTime", sensor.getMatchChargeTime());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/dischargeTime", sensor.getMatchDischargeTime());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/duration", sensor.getMatchDuration());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/endingCharge", sensor.getMatchEndingCharge());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/endingEnergy", sensor.getMatchEndingEnergy());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/maxCurrent", sensor.getMatchMaxCurrent());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/maxDodAh", sensor.getMatchMaxDodAh());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/maxVoltage", sensor.getMatchMaxVoltage());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/minCurrent", sensor.getMatchMinCurrent());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/minDodAh", sensor.getMatchMinDodAh());
      Logger.recordOutput(SUBSYSTEM_NAME + "/match/minVoltage", sensor.getMatchMinVoltage());
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/match/rmsDischargeCurrent", sensor.getMatchRmsDischargeCurrent());
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/match/startingCharge", sensor.getMatchStartingCharge());
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/match/startingEnergy", sensor.getMatchStartingEnergy());
      Logger.recordOutput(
          SUBSYSTEM_NAME + "/match/startingVoltage", sensor.getMatchStartingVoltage());
    }
  }

  /**
   * Logs data that changes while the robot is running (for example, voltage, current, and remaining
   * charge). This data is logged every time this method is invoked.
   */
  private void logPeriodicData() {
    Logger.recordOutput(SUBSYSTEM_NAME + "/isConnected", sensor.isConnected());
    Logger.recordOutput(SUBSYSTEM_NAME + "/current", sensor.getCurrent());
    Logger.recordOutput(SUBSYSTEM_NAME + "/dodAh", sensor.getDodAh());
    Logger.recordOutput(SUBSYSTEM_NAME + "/dodWh", sensor.getDodWh());
    Logger.recordOutput(SUBSYSTEM_NAME + "/remainingChargePct", sensor.getRemainingChargePct());
    Logger.recordOutput(SUBSYSTEM_NAME + "/voltage", sensor.getVoltage());
  }

  /**
   * Returns whether the Battery Fuel Gauge is connected.
   *
   * @return true if the Battery Fuel Gauge is connected; false otherwise
   */
  public boolean isConnected() {
    return sensor.isConnected();
  }
}
