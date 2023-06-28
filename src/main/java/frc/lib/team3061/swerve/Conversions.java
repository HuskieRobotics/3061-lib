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
   * @param falconRotations Falcon rotations
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return degrees of rotation of mechanism
   */
  public static double falconRotationsToMechanismDegrees(double falconRotations, double gearRatio) {
    return falconRotations * 360.0 / gearRatio;
  }

  /**
   * @param degrees degrees of rotation of mechanism
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees Degrees of rotation of mechanism
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon rotations
   */
  public static double degreesToFalconRotations(double degrees, double gearRatio) {
    return (degrees / 360.0) * gearRatio;
  }

  /**
   * @param velocityCounts Falcon velocity counts
   * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
   * @return RPM of mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    return motorRPM / gearRatio;
  }

  /**
   * @param rps Falcon rotations per second
   * @param gearRatio gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)
   * @return RPM of mechanism
   */
  public static double falconRPSToMechanismRPM(double falconRPS, double gearRatio) {
    double motorRPM = falconRPS * 60.0;
    return motorRPM / gearRatio;
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon velocity counts
   */
  public static double rpmToFalcon(double rpm, double gearRatio) {
    double motorRPM = rpm * gearRatio;
    return motorRPM * (2048.0 / 600.0);
  }

  /**
   * @param RPM RPM of mechanism
   * @param gearRatio Gear ratio between Falcon and mechanism (set to 1 for Falcon RPS)
   * @return Falcon rotations per second
   */
  public static double rpmToFalconRPS(double rpm, double gearRatio) {
    double motorRPM = rpm * gearRatio;
    return motorRPM / 60.0;
  }

  /**
   * @param counts Falcon counts
   * @param circumference Circumference of wheel
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return linear distance traveled by wheel in meters
   */
  public static double falconToMeters(double counts, double circumference, double gearRatio) {
    double motorRotations = counts / 2048.0;
    double wheelRotations = motorRotations / gearRatio;
    return (wheelRotations * circumference);
  }

  /**
   * @param falconRotations Falcon rotations
   * @param circumference circumference of wheel
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return linear distance traveled by wheel in meters
   */
  public static double falconRotationsToMechanismMeters(
      double falconRotations, double circumference, double gearRatio) {
    double wheelRotations = falconRotations / gearRatio;
    return (wheelRotations * circumference);
  }

  /**
   * @param meters meters
   * @param circumference circumference of wheel
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon counts
   */
  public static double metersToFalcon(double meters, double circumference, double gearRatio) {
    double wheelRotations = meters / circumference;
    double motorRotations = wheelRotations * gearRatio;
    return motorRotations * 2048.0;
  }

  /**
   * @param velocityCounts Falcon velocity counts
   * @param circumference circumference of wheel
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return mechanism linear velocity in meters per second
   */
  public static double falconToMPS(double velocityCounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocityCounts, gearRatio);
    return (wheelRPM * circumference) / 60;
  }

  /**
   * @param falconRPS Falcon rotations per second
   * @param circumference circumference of wheel
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return mechanism linear velocity in meters per second
   */
  public static double falconRPSToMechanismMPS(
      double falconRPS, double circumference, double gearRatio) {
    double wheelRPM = falconRPSToMechanismRPM(falconRPS, gearRatio);
    return (wheelRPM * circumference) / 60;
  }

  /**
   * @param velocity velocity in meters per second
   * @param circumference circumference of wheel
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon velocity counts
   */
  public static double mpsToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    return rpmToFalcon(wheelRPM, gearRatio);
  }

  /**
   * @param velocity velocity in meters per second
   * @param circumference circumference of wheel
   * @param gearRatio gear ratio between Falcon and mechanism
   * @return Falcon rotations per second
   */
  public static double mpsToFalconRPS(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    return rpmToFalconRPS(wheelRPM, gearRatio);
  }

  private Conversions() {}
}
