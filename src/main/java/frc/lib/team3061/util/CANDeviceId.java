/*
 * Initially from https://github.com/FRC3620/FRC3620_2020_GalacticSenate
 */

package frc.lib.team3061.util;

/**
 * An object representing a CAN device on the bus. These can be put in Collections, are Hashable,
 * and are Comparable.
 */
public class CANDeviceId implements Comparable<CANDeviceId> {

  public enum CANDeviceType {
    PDP,
    PCM,
    TALON,
    VICTOR_SPX,
    SPARK_MAX
  }

  private CANDeviceType deviceType;
  private int deviceNumber;

  public CANDeviceType getDeviceType() {
    return deviceType;
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  private String toString;

  public CANDeviceId(CANDeviceType deviceType, int deviceNumber) {
    this.deviceType = deviceType;
    this.deviceNumber = deviceNumber;
    this.toString = deviceType.toString() + " " + Integer.toString(deviceNumber);
  }

  @Override
  public boolean equals(Object object) {
    if (this == object) return true;
    if (object == null || getClass() != object.getClass()) return false;
    if (!super.equals(object)) return false;

    CANDeviceId that = (CANDeviceId) object;

    if (deviceNumber != that.deviceNumber) return false;
    return deviceType == that.deviceType;
  }

  @Override
  public int hashCode() {
    int result = super.hashCode();
    result = 31 * result + deviceType.hashCode();
    result = 31 * result + deviceNumber;
    return result;
  }

  @Override
  public java.lang.String toString() {
    return this.toString;
  }

  @Override
  public int compareTo(CANDeviceId canDeviceId) {
    int rv = Integer.compare(this.deviceType.ordinal(), canDeviceId.deviceType.ordinal());
    if (rv == 0) rv = Integer.compare(this.deviceNumber, canDeviceId.deviceNumber);
    return rv;
  }
}
