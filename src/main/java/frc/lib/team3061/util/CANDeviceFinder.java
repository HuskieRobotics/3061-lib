/*
 * Initially from https://github.com/FRC3620/FRC3620_2020_GalacticSenate
 */

package frc.lib.team3061.util;

import edu.wpi.first.hal.can.CANJNI;
import frc.lib.team3061.util.CANDeviceId.CANDeviceType;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;

public class CANDeviceFinder {
  Set<CANDeviceId> deviceSet = new TreeSet<>();

  /*
   * this is a map, keyed by CANDeviceType, whose values are sets contained
   * the device numbers for all the present devices of that type.
   */
  Map<CANDeviceType, Set<Integer>> byDeviceType = new TreeMap<>();

  public CANDeviceFinder() {
    super();
    find();
  }

  public boolean isDevicePresent(CANDeviceType deviceType, int id) {
    return isDevicePresent(deviceType, id, null);
  }

  public boolean isDevicePresent(CANDeviceType deviceType, int id, String whatItIs) {
    boolean rv = false;
    Set<Integer> deviceTypeSet = byDeviceType.get(deviceType);
    if (deviceTypeSet != null) {
      rv = deviceTypeSet.contains(id);
    }
    if (!rv) {
      if (whatItIs == null) {
        Alert motor =
            new Alert(
                " " + deviceType + "(" + id + ")" + " is missing From the CAN bus",
                AlertType.WARNING);
        motor.set(true);
      } else {
        Alert motor =
            new Alert(
                " " + deviceType + "(" + id + ")" + whatItIs + " is missing From the CAN bus",
                AlertType.WARNING);
        motor.set(true);
      }
    }
    return rv;
  }

  /**
   * @return ArrayList of strings holding the names of devices we've found.
   */
  public Set<CANDeviceId> getDeviceSet() {
    return deviceSet;
  }

  abstract class CanFinder {
    int[] ids;
    long[] ts0;
    Set<Integer> idsPresent = new TreeSet<>();

    void pass1() {
      ts0 = new long[ids.length];
      for (int i = 0; i < ids.length; ++i) {
        ts0[i] = checkMessage(ids[i]);
      }
    }

    void pass2() {
      long[] ts1 = new long[ids.length];
      for (int i = 0; i < ids.length; ++i) {
        ts1[i] = checkMessage(ids[i]);
      }
      for (int i = 0; i < ids.length; ++i) {
        if (ts0[i] >= 0 && ts1[i] >= 0 && ts0[i] != ts1[i]) {
          idsPresent.add(ids[i]);
        }
      }
    }

    private long checkMessage(int id) {
      try {
        targetID.clear();
        targetID.order(ByteOrder.LITTLE_ENDIAN);
        targetID.asIntBuffer().put(0, id);

        timeStamp.clear();
        timeStamp.order(ByteOrder.LITTLE_ENDIAN);
        timeStamp.asIntBuffer().put(0, 0x00000000);

        CANJNI.FRCNetCommCANSessionMuxReceiveMessage(targetID.asIntBuffer(), 0x1fffffff, timeStamp);

        long retval = timeStamp.getInt();
        retval &= 0xFFFFFFFF; /* undo sign-extension */
        return retval;
      } catch (Exception e) {
        return -1;
      }
    }

    abstract void report();
  }

  class DeviceFinder extends CanFinder {
    Set<CANDeviceId> deviceSet;
    CANDeviceType canDeviceType;

    DeviceFinder(
        int devType,
        int mfg,
        int apiId,
        int maxDevices,
        Set<CANDeviceId> deviceSet,
        CANDeviceType canDeviceType) {
      super();

      this.deviceSet = deviceSet;
      this.canDeviceType = canDeviceType;

      ids = new int[maxDevices];
      for (int i = 0; i < maxDevices; i++) {
        ids[i] = canBusId(devType, mfg, apiId, i);
      }
    }

    @Override
    void report() {
      for (int id : idsPresent) {
        int deviceId = extractDeviceId(id);
        deviceSet.add(new CANDeviceId(canDeviceType, deviceId)); // NOPMD
      }
    }

    private int extractDeviceId(int canId) {
      return canId & 0x3f;
    }
  }

  class APIFinder extends CanFinder {
    CANDeviceType canDeviceType;

    APIFinder(int devType, int mfg, int deviceId, CANDeviceType canDeviceType) {
      super();
      this.canDeviceType = canDeviceType;

      ids = new int[1024];
      for (int i = 0; i < 1024; i++) {
        ids[i] = canBusId(devType, mfg, i, deviceId);
      }
    }

    @Override
    void report() {
      for (int id : idsPresent) {
        extractApiId(id);
      }
    }
  }

  /**
   * polls for received framing to determine if a device is deviceSet. This is meant to be used once
   * initially (and not periodically) since this steals cached messages from the robot API.
   */
  public void find() {
    List<CanFinder> finders = new ArrayList<>();

    /*
     * PDPs used to be 0x08041400.
     * 2019.02.09: PDPs respond to APIs 0x50 0x51 0x52 0x59 0x5d
     */
    finders.add(new DeviceFinder(8, 4, extractApiId(0x08041400), 1, deviceSet, CANDeviceType.PDP));

    /*
     * SRX used to be 0x02041400.

    As of 2019.02.08: (SRX @ devid 1)
     7 0x007 = 020401C1
    81 0x051 = 02041441 ** using this?
    82 0x052 = 02041481
    83 0x053 = 020414C1
    87 0x057 = 020415C1
    91 0x05B = 020416C1
    92 0x05C = 02041701
    93 0x05D = 02041741
    94 0x05E = 02041781

    2020.01.20 Device id is 0x0204 (https://github.com/CrossTheRoadElec/Phoenix-api/blob/master/src/main/java/com/ctre/phoenix/motorcontrol/can/TalonSRX.java)

    Talon FX and SRX are the same.
    */
    finders.add(
        new DeviceFinder(2, 4, extractApiId(0x02041441), 64, deviceSet, CANDeviceType.TALON));

    /*
    SPX used to be 0x01041400.

    As of 2019.02.08:  (SPX @ devid 2)
     7 0x007 = 010401C2
    81 0x051 = 01041442 ** using this
    83 0x053 = 010414C2
    91 0x05B = 010416C2
    92 0x05C = 01041702
    93 0x05D = 01041742
    94 0x05E = 01041782

    2020.01.20 Device id is 0x0104 (https://github.com/CrossTheRoadElec/Phoenix-api/blob/master/src/main/java/com/ctre/phoenix/motorcontrol/can/VictorSPX.java)
    */
    finders.add(
        new DeviceFinder(1, 4, extractApiId(0x01041442), 64, deviceSet, CANDeviceType.VICTOR_SPX));

    /* we always used 0x09041400 for PCMs */
    finders.add(new DeviceFinder(9, 4, extractApiId(0x09041400), 64, deviceSet, CANDeviceType.PCM));

    // per REV (0x02051800)
    finders.add(
        new DeviceFinder(2, 5, extractApiId(0x02051800), 64, deviceSet, CANDeviceType.SPARK_MAX));

    findDetails(finders);
  }

  public void research() {
    List<CanFinder> finders = new ArrayList<>();

    finders.add(new APIFinder(9, 4, 0, CANDeviceType.PCM)); // PCM
    finders.add(new APIFinder(8, 4, 0, CANDeviceType.PDP)); // PDP
    finders.add(new APIFinder(2, 4, 1, CANDeviceType.TALON)); // SRX #1
    finders.add(new APIFinder(1, 4, 2, CANDeviceType.VICTOR_SPX)); // SPX #2

    findDetails(finders);
  }

  void findDetails(List<CanFinder> finders) {
    deviceSet.clear();
    byDeviceType.clear();

    for (CanFinder finder : finders) {
      finder.pass1();
    }

    /* wait 200ms */
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
      e.printStackTrace(); // NOPMD
      Thread.currentThread().interrupt();
    }

    for (CanFinder finder : finders) {
      finder.pass2();
    }

    for (CanFinder finder : finders) {
      finder.report();
    }

    /*
    fill in the byDeviceType map.
    */
    for (CANDeviceId canDeviceId : deviceSet) {
      CANDeviceType canDeviceType = canDeviceId.getDeviceType();

      Set<Integer> deviceNumberSet =
          byDeviceType.computeIfAbsent(canDeviceType, k -> new TreeSet<>());
      deviceNumberSet.add(canDeviceId.getDeviceNumber());
    }
  }

  /* help to calculate the CAN bus ID for a devType|mfg|api|dev.
  total of 32 bits: 8 bit devType, 8 bit mfg, 10 bit API, 6 bit device id.
  */
  boolean logCanBusIds = false;

  private int canBusId(int devType, int mfg, int apiId, int devId) {
    return ((devType & 0xff) << 24)
        | ((mfg & 0xff) << 16)
        | ((apiId & 0x3ff) << 6)
        | (devId & 0x3f);
  }

  private int extractApiId(int canId) {
    return (canId & 0xffc0) >> 6;
  }

  /** helper routine to get last received message for a given ID */
  private ByteBuffer targetID = ByteBuffer.allocateDirect(4);

  private ByteBuffer timeStamp = ByteBuffer.allocateDirect(4);
}
