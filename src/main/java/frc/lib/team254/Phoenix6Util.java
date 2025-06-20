// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Adapted for 3061-lib

package frc.lib.team254;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Alert;
import java.util.function.Supplier;

@java.lang.SuppressWarnings({"java:S112"})
public class Phoenix6Util {

  private static Alert canivoreSignalsAlert = new Alert("", Alert.AlertType.kError);
  private static Alert rioSignalsAlert = new Alert("", Alert.AlertType.kError);

  private Phoenix6Util() {}

  /**
   * Checks the specified status code and sets the specified alert to the specified message if the
   * status code is not OK.
   *
   * @param statusCode status code to check
   * @param message message to set in the alert if the status code is not OK
   * @param alert alert to set if the status code is not OK
   */
  public static void checkError(StatusCode statusCode, String message, Alert alert) {
    if (statusCode != StatusCode.OK) {
      alert.setText(message + " " + statusCode);
      alert.set(true);
    } else {
      alert.set(false);
    }
  }

  /**
   * Invokes the specified CTRE function until it is successful or the number of tries is exceeded.
   * Sets the specified alert if the function fails.
   *
   * @param function CTRE function to invoke
   * @param alert alert to set if the function fails
   * @param numTries number of times to try the function
   * @return true if the function was successful, false otherwise
   */
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
    alert.set(false);
    return true;
  }

  /**
   * Checks the specified status code and throws an exception with the specified message of it is
   * not OK.
   *
   * @param statusCode status code to check
   * @param message message to include with the exception if the status code is not OK
   */
  public static void checkErrorWithThrow(StatusCode statusCode, String message) {
    if (statusCode != StatusCode.OK) {
      throw new RuntimeException(message + " " + statusCode);
    }
  }

  /**
   * Invokes the specified CTRE function until it is successful or five tries are exceeded.
   *
   * @param function CTRE function to invoke
   * @param alert Alert to set if the function fails
   * @return true if the function was successful, false otherwise
   */
  public static boolean checkErrorAndRetry(Supplier<StatusCode> function, Alert alert) {
    return checkErrorAndRetry(function, alert, 5);
  }

  /**
   * Applies the specified configuration to the specified TalonFX and checks that the configuration
   * was applied successfully. If not, retries the specified number of tries; eventually, setting
   * the specified alert if the number of tries is exceeded.
   *
   * @param talon TalonFX to which to apply the configuration
   * @param config TalonFX configuration to apply
   * @param alert alert to set if the configuration is not applied successfully
   * @param numTries number of times to try to apply the configuration
   * @return true if the configuration was applied successfully, false otherwise
   */
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

  /**
   * Reads the configuration from the specified TalonFX and verifies that it matches the specified
   * configuration. If the configuration does not match, sets the specified alert.
   *
   * @param talon TalonFX from which to read the configuration
   * @param config TalonFX configuration to verify
   * @param alert alert to set if the configuration does not match
   * @return true if the configuration was read and matched, false otherwise
   */
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

  /**
   * Applies the specified configuration to the specified CANcoder and checks that the configuration
   * was applied successfully. If not, retries five times; eventually, setting the specified alert
   * if the number of tries is exceeded.
   *
   * @param talon TalonFX to which to apply the configuration
   * @param config TalonFX configuration to apply
   * @param alert alert to set if the configuration is not applied successfully
   * @return true if the configuration was applied successfully, false otherwise
   */
  public static boolean applyAndCheckConfiguration(
      TalonFX talon, TalonFXConfiguration config, Alert alert) {
    return applyAndCheckConfiguration(talon, config, alert, 5);
  }

  /**
   * Applies the specified configuration to the specified TalonFX and checks that the configuration
   * was applied successfully. If not, retries the specified number of tries; eventually, setting
   * the specified alert if the number of tries is exceeded.
   *
   * @param canCoder CANcoder to which to apply the configuration
   * @param config CANcoder configuration to apply
   * @param alert alert to set if the configuration is not applied successfully
   * @param numTries number of times to try to apply the configuration
   * @return true if the configuration was applied successfully, false otherwise
   */
  public static boolean applyAndCheckConfiguration(
      CANcoder canCoder, CANcoderConfiguration config, Alert alert, int numTries) {
    for (int i = 0; i < numTries; i++) {
      if (checkErrorAndRetry(() -> canCoder.getConfigurator().apply(config), alert)) {
        // API says we applied config, lets make sure it's right
        if (readAndVerifyConfiguration(canCoder, config, alert)) {
          return true;
        } else {
          alert.setText(
              "Failed to verify config for CANcoder ["
                  + canCoder.getDeviceID()
                  + "] (attempt "
                  + (i + 1)
                  + " of "
                  + numTries
                  + ")");
          alert.set(true);
        }
      } else {
        alert.setText(
            "Failed to apply config for CANcoder ["
                + canCoder.getDeviceID()
                + "] (attempt "
                + (i + 1)
                + " of "
                + numTries
                + ")");
        alert.set(true);
      }
    }
    alert.setText("Failed to apply config for CANcoder after " + numTries + " attempts");
    alert.set(true);
    return false;
  }

  /**
   * Reads the configuration from the specified CANcoder and verifies that it matches the specified
   * configuration. If the configuration does not match, sets the specified alert.
   *
   * @param canCoder CANcoder to which to apply the configuration
   * @param config CANcoder configuration to apply
   * @param alert alert to set if the configuration does not match
   * @return true if the configuration was read and matched, false otherwise
   */
  public static boolean readAndVerifyConfiguration(
      CANcoder canCoder, CANcoderConfiguration config, Alert alert) {
    CANcoderConfiguration readConfig = new CANcoderConfiguration();
    if (!checkErrorAndRetry(() -> canCoder.getConfigurator().refresh(readConfig), alert)) {
      // could not get config!
      alert.setText("Failed to read config for CANcoder [" + canCoder.getDeviceID() + "]");
      alert.set(true);
      return false;
    } else if (!CANcoderConfigEquality.isEqual(config, readConfig)) {
      // configs did not match
      alert.setText(
          "Configuration verification failed for CANcoder [" + canCoder.getDeviceID() + "]");
      alert.set(true);
      return false;
    } else {
      // configs read and match, Talon OK
      return true;
    }
  }

  /**
   * Applies the specified configuration to the specified CANcoder and checks that the configuration
   * was applied successfully. If not, retries five times; eventually, setting the specified alert
   * if the number of tries is exceeded.
   *
   * @param canCoder CANcoder to which to apply the configuration
   * @param config CANcoder configuration to apply
   * @param alert alert to set if the configuration is not applied successfully
   * @return true if the configuration was applied successfully, false otherwise
   */
  public static boolean applyAndCheckConfiguration(
      CANcoder canCoder, CANcoderConfiguration config, Alert alert) {
    return applyAndCheckConfiguration(canCoder, config, alert, 5);
  }

  // Copyright (c) 2025 FRC 6328
  // http://github.com/Mechanical-Advantage
  //
  // Use of this source code is governed by an MIT-style
  // license that can be found in the LICENSE file at
  // the root directory of this project.

  /** Signals for synchronized refresh. */
  private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

  private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
    if (canivore) {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[canivoreSignals.length + signals.length];
      System.arraycopy(canivoreSignals, 0, newSignals, 0, canivoreSignals.length);
      System.arraycopy(signals, 0, newSignals, canivoreSignals.length, signals.length);
      canivoreSignals = newSignals;
    } else {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
      System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
      System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
      rioSignals = newSignals;
    }
  }

  /** Refresh all registered signals. */
  public static void refreshAll() {
    if (canivoreSignals.length > 0) {
      checkError(
          BaseStatusSignal.refreshAll(canivoreSignals),
          "failed to refresh signals on CANivore:",
          canivoreSignalsAlert);
    }
    if (rioSignals.length > 0) {
      checkError(
          BaseStatusSignal.refreshAll(rioSignals),
          "failed to refresh signals on RIO:",
          rioSignalsAlert);
    }
  }
}
