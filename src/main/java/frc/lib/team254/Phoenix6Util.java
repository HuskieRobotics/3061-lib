// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Adapted for 3061-lib

package frc.lib.team254;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import java.util.function.Supplier;

@java.lang.SuppressWarnings({"java:S112"})
public class Phoenix6Util {

  private Phoenix6Util() {}

  /**
   * checks the specified error code for issues
   *
   * @param statusCode error code
   * @param message message to print if error happens
   */
  public static void checkError(StatusCode statusCode, String message, Alert alert) {
    if (statusCode != StatusCode.OK) {
      alert.setText(message + " " + statusCode);
      alert.set(true);
    }
  }

  public static boolean checkErrorAndRetry(
      Supplier<StatusCode> function, Alert alert, int numTries) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      alert.setText("Retrying CTRE Device Config " + code.getName());
      alert.set(true);
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      alert.setText(
          "Failed to execute phoenix 6 api call after "
              + numTries
              + " attempts. "
              + code.getDescription());
      alert.set(true);
      return false;
    }
    return true;
  }

  /**
   * checks the specified error code and throws an exception if there are any issues
   *
   * @param statusCode error code
   * @param message message to print if error happens
   */
  public static void checkErrorWithThrow(StatusCode statusCode, String message) {
    if (statusCode != StatusCode.OK) {
      throw new RuntimeException(message + " " + statusCode);
    }
  }

  public static boolean checkErrorAndRetry(Supplier<StatusCode> function, Alert alert) {
    return checkErrorAndRetry(function, alert, 5);
  }

  public static boolean applyAndCheckConfiguration(
      TalonFX talon, TalonFXConfiguration config, Alert alert, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (checkErrorAndRetry(() -> talon.getConfigurator().apply(config), alert)) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(talon, config, alert)) {
          return true;
        } else {
          alert.setText(
              "Failed to verify config for talon ["
                  + talon.getDescription()
                  + "] (attempt "
                  + (i + 1)
                  + " of "
                  + numTries
                  + ")");
          alert.set(true);
        }
      } else {
        alert.setText(
            "Failed to apply config for talon ["
                + talon.getDescription()
                + "] (attempt "
                + (i + 1)
                + " of "
                + numTries
                + ")");
        alert.set(true);
      }
    }
    alert.setText("Failed to apply config for talon after " + numTries + " attempts");
    alert.set(true);
    return false;
  }

  public static boolean readAndVerifyConfiguration(
      TalonFX talon, TalonFXConfiguration config, Alert alert) {
    TalonFXConfiguration readConfig = new TalonFXConfiguration();
    if (!checkErrorAndRetry(() -> talon.getConfigurator().refresh(readConfig), alert)) {
      // could not get config!
      alert.setText("Failed to read config for talon [" + talon.getDescription() + "]");
      alert.set(true);
      return false;
    } else if (!TalonFXConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      alert.setText("Configuration verification failed for talon [" + talon.getDescription() + "]");
      alert.set(true);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  public static boolean applyAndCheckConfiguration(
      TalonFX talon, TalonFXConfiguration config, Alert alert) {
    return applyAndCheckConfiguration(talon, config, alert, 5);
  }
}
