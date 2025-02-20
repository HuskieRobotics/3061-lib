// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// adapted for 3061-lib

package frc.lib.team6328.util;

import static frc.robot.Constants.*;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Class for a tunable boolean. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableBoolean implements BooleanSupplier {
  private static final String TABLE_KEY = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private boolean defaultValue;
  private LoggedNetworkBoolean dashboardBoolean;
  private boolean readAndWriteAlways = true;
  private Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableBoolean
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableBoolean(String dashboardKey) {
    this.key = TABLE_KEY + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableBoolean with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Create a new LoggedTunableBoolean with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue the default value
   * @param readAndWrite if the value can be read/written to depending on tuning
   */
  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue, boolean readAndWrite) {
    this.key = TABLE_KEY + "/" + dashboardKey;
    this.readAndWriteAlways = readAndWrite;
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the boolean. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(boolean defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;

      if (TUNING_MODE || this.readAndWriteAlways) {
        dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public boolean get() {
    if (!hasDefault) {
      return false;
    }
    return (TUNING_MODE || this.readAndWriteAlways) ? dashboardBoolean.get() : defaultValue;
  }

  /**
   * Set the value, to dashboard if available and in tuning mode.
   *
   * @param value The value to set
   */
  public void set(boolean value) {
    if (dashboardBoolean != null) {
      dashboardBoolean.set(value);
    }
  }

  /**
   * Checks whether the boolean value has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the boolean has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    boolean currentValue = get();
    Boolean lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableBooleans have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable booleans have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableBooleans All tunable booleans to check
   */
  public static void ifChanged(
      int id, Consumer<Boolean[]> action, LoggedTunableBoolean... tunableBooleans) {

    if (Arrays.stream(tunableBooleans).anyMatch(tunableBoolean -> tunableBoolean.hasChanged(id))) {
      action.accept(
          Arrays.stream(tunableBooleans).map(LoggedTunableBoolean::get).toArray(Boolean[]::new));
    }
  }

  /**
   * Runs action if any of the tunableBooleans have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   * @param action Callback to run when any of the tunable booleans have changed. Access tunable
   * @param tunableBooleans All tunable booleans to check
   */
  public static void ifChanged(int id, Runnable action, LoggedTunableBoolean... tunableBooleans) {
    ifChanged(id, values -> action.run(), tunableBooleans);
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }
}
