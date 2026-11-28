// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// adapted for 3061-lib

package first.lib.util;

import static first.robot.Constants.*;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String TABLE_KEY = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public LoggedTunableNumber(String dashboardKey) {
    this.key = TABLE_KEY + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (TUNING_MODE) {
        dashboardNumber = new LoggedNetworkNumber(key, defaultValue);
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return TUNING_MODE ? dashboardNumber.get() : defaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action once if any of the tunableNumbers have changed. The action is passed the values of
   * all of the tunable numbers; so, it should apply all of them.
   *
   * <p>Each tunable number tracks its last value separately for each id. So, the same id may be
   * used for multiple calls to this method as long as each tunable number is only passed to one of
   * those calls.
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    // check every tunable number, rather than stopping at the first one that changed, so that each
    // records its current value; otherwise, the action would be run again in each of the following
    // robot loops for each of the other tunable numbers that changed
    boolean changed = false;
    for (LoggedTunableNumber tunableNumber : tunableNumbers) {
      changed |= tunableNumber.hasChanged(id);
    }
    if (changed) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
