package first.lib.team3061.bfg;

import org.wpilib.hardware.bus.CAN;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.CANAPITypes.CANDeviceType;
import org.wpilib.hardware.hal.CANAPITypes.CANManufacturer;
import org.wpilib.hardware.hal.can.CANReceiveMessage;

/**
 * BFG IO implementation that reads the Playing With Fusion Battery Fuel Gauge's CAN frames directly
 * with WPILib's {@link CAN} class.
 *
 * <p>This is an interim implementation until Playing With Fusion releases a 2027 vendordep that
 * supports SystemCore. It decodes the frames documented in the "CAN Messages" section of the
 * ROB-70001 Battery Fuel Gauge User Manual (revision 2/27/2026). It logs the same units and the
 * same names for the charge state and battery manufacturer as the 2026 vendordep so that logs don't
 * change when this class is replaced by one that uses the vendordep.
 *
 * <p>The BFG's frames use WPILib's standard CAN identifiers: device type 10 (miscellaneous),
 * manufacturer 11 (Playing With Fusion), a 10-bit API ID, and a 6-bit device ID. The manual writes
 * each frame's identifier as 0x0A0Bxxss, where xx selects the frame and ss is the device ID; so,
 * the API ID is xx shifted left by two bits, and the device ID must be between 0 and 63. The
 * heartbeat frame uses device type 31 (firmware update) instead.
 *
 * <p>The manual is ambiguous about a few fields. Where it is, this class follows the most likely
 * interpretation, noted where the field is decoded; these should be verified on a robot.
 */
public class BFGIOCAN implements BFGIO {

  // API IDs (the manual's frame selector shifted left by two bits)
  private static final int BATTERY_POWER_API = 0x00 << 2;
  private static final int BATTERY_HEALTH_API = 0x01 << 2;
  private static final int MATCH_CURRENT_API = 0x02 << 2;
  private static final int MATCH_CHARGE_API = 0x03 << 2;
  private static final int MATCH_TIME_API = 0x05 << 2;
  private static final int RMS_CURRENT_API = 0x06 << 2;
  private static final int STATE_OF_CHARGE_API = 0x07 << 2;
  private static final int LAST_CYCLE_CURRENT_API = 0x08 << 2;
  private static final int LAST_CYCLE_CHARGE_API = 0x09 << 2;
  private static final int LAST_CYCLE_TIME_API = 0x0A << 2;
  private static final int LAST_CYCLE_ENERGY_API = 0x0C << 2;
  private static final int MATCH_DELTA_ENERGY_API = 0x0D << 2;
  private static final int MATCH_DELTA_CHARGE_API = 0x0E << 2;
  private static final int NICKNAME_1_API = 0x0F << 2;
  private static final int NICKNAME_2_API = 0x10 << 2;
  private static final int HEARTBEAT_API = 0x01 << 2;

  // the battery power and state of charge frames are sent every 10 ms; if none has been received
  // for this long, the BFG is considered disconnected
  private static final int CONNECTED_TIMEOUT_MS = 100;

  // depth of discharge energy in the match and last cycle frames has a resolution of 128 J
  private static final double JOULES_PER_ENERGY_BIT = 128.0;
  private static final double JOULES_PER_WATT_HOUR = 3600.0;
  private static final double SECONDS_PER_DAY = 86400.0;

  // the 2026 vendordep's names for the values of the charge state byte, in order
  private static final String[] CHARGE_STATES = {
    "Init", "Discharge", "ConstICharge", "ConstVCharge", "TrickleCharge", "Idle", "MeasureOCV"
  };

  // the 2026 vendordep's names for the values of the battery manufacturer byte, in order
  private static final String[] MANUFACTURERS = {
    "Duracell", "Energizer", "Interstate", "MightyMax", "MKPowered", "PowerSonic"
  };

  private final CAN bfg;
  private final CAN bfgHeartbeat;
  private final CANReceiveMessage message = new CANReceiveMessage();

  /**
   * Creates a new BFG IO that reads the BFG's CAN frames.
   *
   * @param port the CAN port to which the BFG is connected
   * @param deviceId the BFG's CAN device ID (0-63)
   */
  public BFGIOCAN(CANPort port, int deviceId) {
    bfg = new CAN(port, deviceId, CANManufacturer.PWF.id, CANDeviceType.MISCELLANEOUS.id);
    bfgHeartbeat =
        new CAN(port, deviceId, CANManufacturer.PWF.id, CANDeviceType.FIRMWARE_UPDATE.id);
  }

  @Override
  public void updateInputs(BFGIOInputs inputs) {
    // battery power: current (positive is discharge), voltage, and depth of discharge
    inputs.connected = bfg.readPacketTimeout(BATTERY_POWER_API, message, CONNECTED_TIMEOUT_MS);
    if (inputs.connected) {
      inputs.currentAmps = int32(message.data, 0) / 1000.0;
      inputs.voltageVolts = uint16(message.data, 4) / 1000.0;
      inputs.dodAh = int16(message.data, 6) / 1000.0;
    }

    // state of charge: charge remaining, effective capacity, charge state, and depth of discharge
    if (bfg.readPacketTimeout(STATE_OF_CHARGE_API, message, CONNECTED_TIMEOUT_MS)) {
      double chargeRemainingAh = uint16(message.data, 0) / 1000.0;
      inputs.effectiveCapacityAh = uint16(message.data, 2) / 1000.0;
      inputs.remainingChargePercent =
          inputs.effectiveCapacityAh > 0.0
              ? chargeRemainingAh / inputs.effectiveCapacityAh * 100.0
              : 0.0;
      inputs.chargeState = name(CHARGE_STATES, uint8(message.data, 4));
      // the manual describes bytes 5-7 as "depth of discharge in Joules (Wh / 3600)"; this assumes
      // a signed value in joules, consistent with the other energy fields
      inputs.dodWh = int24(message.data, 5) / JOULES_PER_WATT_HOUR;
    }
  }

  @Override
  public void updateBatteryInputs(BFGBatteryIOInputs inputs) {
    if (bfgHeartbeat.readPacketLatest(HEARTBEAT_API, message)) {
      inputs.serialNumber = uint24(message.data, 1);
      inputs.firmwareVersion = uint16(message.data, 6);
    }

    if (bfg.readPacketLatest(NICKNAME_1_API, message)) {
      String first = ascii(message.data, 0, 8);
      String last = "";
      if (bfg.readPacketLatest(NICKNAME_2_API, message)) {
        last = ascii(message.data, 0, 3);
      }
      inputs.nickname = first.length() < 8 ? first : first + last;
    }

    if (bfg.readPacketLatest(MATCH_DELTA_ENERGY_API, message)) {
      inputs.manufacturer = name(MANUFACTURERS, uint8(message.data, 6));
    }

    if (bfg.readPacketLatest(BATTERY_HEALTH_API, message)) {
      inputs.ageDays = uint32(message.data, 0) / SECONDS_PER_DAY;
      // the manual lists capacity in bytes 4-6 and cycles in bytes 6-7, which overlap; this follows
      // the Set Health message, which places capacity in bytes 4-5 and cycles in bytes 6-7
      inputs.capacityAh = uint16(message.data, 4) / 1000.0;
      inputs.numCycles = uint16(message.data, 6);
    }

    if (bfg.readPacketLatest(LAST_CYCLE_TIME_API, message)) {
      inputs.cycleChargeTimeSecs = uint32(message.data, 0);
      inputs.cycleDischargeTimeSecs = uint32(message.data, 4);
    }

    if (bfg.readPacketLatest(LAST_CYCLE_CURRENT_API, message)) {
      inputs.cycleMinCurrentAmps = int32(message.data, 0) / 1000.0;
      inputs.cycleMaxCurrentAmps = int32(message.data, 4) / 1000.0;
    }

    if (bfg.readPacketLatest(RMS_CURRENT_API, message)) {
      inputs.cycleRmsDischargeCurrentAmps = int32(message.data, 4) / 1000.0;
    }

    if (bfg.readPacketLatest(LAST_CYCLE_CHARGE_API, message)) {
      inputs.cycleMinVoltageVolts = uint16(message.data, 0) / 1000.0;
      inputs.cycleMaxVoltageVolts = uint16(message.data, 2) / 1000.0;
      inputs.cycleMinDodAh = int16(message.data, 4) / 1000.0;
      inputs.cycleMaxDodAh = int16(message.data, 6) / 1000.0;
    }

    if (bfg.readPacketLatest(LAST_CYCLE_ENERGY_API, message)) {
      inputs.cycleMinDodWh = int16(message.data, 0) * JOULES_PER_ENERGY_BIT / JOULES_PER_WATT_HOUR;
      inputs.cycleMaxDodWh = int16(message.data, 2) * JOULES_PER_ENERGY_BIT / JOULES_PER_WATT_HOUR;
      inputs.cycleStartingVoltageVolts = uint16(message.data, 4) / 1000.0;
      // the manual doesn't state the units of the design capacity; this assumes milliamp-hours,
      // like the other capacities
      inputs.ratedCapacityAh = uint16(message.data, 6) / 1000.0;
    }
  }

  @Override
  public void updateMatchInputs(BFGMatchIOInputs inputs) {
    if (bfg.readPacketLatest(MATCH_DELTA_CHARGE_API, message)) {
      inputs.startingChargeAh = int16(message.data, 0) / 1000.0;
      inputs.endingChargeAh = int16(message.data, 2) / 1000.0;
      // the manual doesn't state the units of the match duration; this assumes seconds, like the
      // other times
      inputs.durationSecs = uint32(message.data, 4);
    }

    if (bfg.readPacketLatest(MATCH_TIME_API, message)) {
      inputs.chargeTimeSecs = uint32(message.data, 0);
      inputs.dischargeTimeSecs = uint32(message.data, 4);
    }

    if (bfg.readPacketLatest(MATCH_CURRENT_API, message)) {
      inputs.minCurrentAmps = int32(message.data, 0) / 1000.0;
      inputs.maxCurrentAmps = int32(message.data, 4) / 1000.0;
    }

    if (bfg.readPacketLatest(RMS_CURRENT_API, message)) {
      inputs.rmsDischargeCurrentAmps = int32(message.data, 0) / 1000.0;
    }

    // the manual describes bytes 4-7 of this match frame as the depth of discharge "during the last
    // cycle"; this assumes it is the depth of discharge during the last match
    if (bfg.readPacketLatest(MATCH_CHARGE_API, message)) {
      inputs.minVoltageVolts = uint16(message.data, 0) / 1000.0;
      inputs.maxVoltageVolts = uint16(message.data, 2) / 1000.0;
      inputs.minDodAh = int16(message.data, 4) / 1000.0;
      inputs.maxDodAh = int16(message.data, 6) / 1000.0;
    }

    if (bfg.readPacketLatest(MATCH_DELTA_ENERGY_API, message)) {
      inputs.startingEnergyWh =
          int16(message.data, 0) * JOULES_PER_ENERGY_BIT / JOULES_PER_WATT_HOUR;
      inputs.endingEnergyWh = int16(message.data, 2) * JOULES_PER_ENERGY_BIT / JOULES_PER_WATT_HOUR;
      inputs.startingVoltageVolts = uint16(message.data, 4) / 1000.0;
    }
  }

  // The BFG's multi-byte fields are little-endian (least significant byte first).

  private static int uint8(byte[] data, int offset) {
    return data[offset] & 0xFF;
  }

  private static int uint16(byte[] data, int offset) {
    return uint8(data, offset) | (uint8(data, offset + 1) << 8);
  }

  private static int int16(byte[] data, int offset) {
    return (short) uint16(data, offset);
  }

  private static int uint24(byte[] data, int offset) {
    return uint16(data, offset) | (uint8(data, offset + 2) << 16);
  }

  private static int int24(byte[] data, int offset) {
    // shift the sign bit (bit 23) into bit 31 and back to sign-extend the value
    return (uint24(data, offset) << 8) >> 8;
  }

  private static int int32(byte[] data, int offset) {
    return uint16(data, offset) | (uint16(data, offset + 2) << 16);
  }

  private static long uint32(byte[] data, int offset) {
    return int32(data, offset) & 0xFFFFFFFFL;
  }

  private static String ascii(byte[] data, int offset, int maxLength) {
    StringBuilder text = new StringBuilder();
    for (int i = offset; i < offset + maxLength && data[i] != 0; i++) {
      text.append((char) data[i]);
    }
    return text.toString();
  }

  private static String name(String[] names, int value) {
    return value < names.length ? names[value] : "Unknown (" + value + ")";
  }
}
