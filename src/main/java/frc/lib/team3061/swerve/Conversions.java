/*
 * Initially from https://github.com/Team364/BaseFalconSwerve
 */

package frc.lib.team3061.swerve;

public class Conversions {

  /**
   * @param counts Falcon Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    return motorRPM / gearRatio;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double rpmToFalcon(double rpm, double gearRatio) {
    double motorRPM = rpm * gearRatio;
    return motorRPM * (2048.0 / 600.0);
  }

  /**
   * @param counts Falcon counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double falconToMeters(double counts, double circumference, double gearRatio) {
    double motorRotations = counts / 2048.0;
    double wheelRotations = motorRotations / gearRatio;
    return (wheelRotations * circumference);
  }

  /**
   * @param meters meters
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Counts
   */
  public static double metersToFalcon(double meters, double circumference, double gearRatio) {
    double wheelRotations = meters / circumference;
    double motorRotations = wheelRotations * gearRatio;
    return motorRotations * 2048.0;
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double falconToMPS(double velocityCounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocityCounts, gearRatio);
    return (wheelRPM * circumference) / 60;
  }

  /**
   * @param velocity Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double mpsToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    return rpmToFalcon(wheelRPM, gearRatio);
  }

  private Conversions() {}
}
