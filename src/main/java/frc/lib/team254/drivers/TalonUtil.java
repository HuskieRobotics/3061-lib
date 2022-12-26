/*
 * Initially from https://github.com/frc1678/C2022
 */

package frc.lib.team254.drivers;

import com.ctre.phoenix.ErrorCode;
import edu.wpi.first.wpilibj.DriverStation;

public class TalonUtil {
  /**
   * checks the specified error code for issues
   *
   * @param errorCode error code
   * @param message message to print if error happens
   */
  public static void checkError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(message + errorCode, false);
    }
  }

  private TalonUtil() {}
}
