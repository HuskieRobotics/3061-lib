// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// adapted for 3061-lib

package first.lib.energy;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Thermal model of the robot's 120 A main breaker using Miner's-rule damage accumulation.
 *
 * <p>The damage is 0 for a cool breaker and 1 when the breaker would trip. While the current
 * exceeds the breaker's rating, damage accumulates at a rate based on the breaker's datasheet trip
 * curve (e.g., at twice the rated current, the breaker trips after 70 seconds; so, each second adds
 * 1/70 of damage). While the current is at or below the breaker's rating, the damage decays
 * exponentially as the breaker cools.
 */
public class BreakerModel {
  private static final double RATED_CURRENT_AMPS = 120.0;
  private static final double COOLING_TIME_CONSTANT_SECS = 60.0;

  // maximum hold times from the breaker's datasheet, as multiples of the rated current
  private static final double[] NORMALIZED_CURRENT_POINTS = {1.35, 2.0, 2.25, 2.5, 3.0, 4.0, 5.0};
  private static final double[] TRIP_TIME_POINTS_SECS = {
    30.0 * 60.0, 70.0, 38.0, 25.0, 15.0, 10.0, 7.0
  };
  private static final double SENTINEL_TRIP_TIME_SECS = 1.0e6;

  // the trip curve (and its inverse) are interpolated in log-log space
  private static final InterpolatingDoubleTreeMap logTripTimeMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap logInverseTripTimeMap =
      new InterpolatingDoubleTreeMap();

  static {
    logTripTimeMap.put(Math.log(1.0), Math.log(SENTINEL_TRIP_TIME_SECS));
    logInverseTripTimeMap.put(Math.log(SENTINEL_TRIP_TIME_SECS), Math.log(1.0));
    for (int i = 0; i < NORMALIZED_CURRENT_POINTS.length; i++) {
      logTripTimeMap.put(
          Math.log(NORMALIZED_CURRENT_POINTS[i]), Math.log(TRIP_TIME_POINTS_SECS[i]));
      logInverseTripTimeMap.put(
          Math.log(TRIP_TIME_POINTS_SECS[i]), Math.log(NORMALIZED_CURRENT_POINTS[i]));
    }
  }

  private double damage = 0.0;

  /**
   * Updates the damage based on the current through the breaker over the specified period.
   *
   * @param currentAmps the current through the breaker in amps; its sign is ignored
   * @param periodSecs the duration of the current in seconds
   */
  public void update(double currentAmps, double periodSecs) {
    double normalizedCurrent = Math.abs(currentAmps) / RATED_CURRENT_AMPS;

    if (normalizedCurrent > 1.0) {
      damage += periodSecs / getTripTimeSecs(normalizedCurrent);
    } else {
      damage *= Math.exp(-periodSecs / COOLING_TIME_CONSTANT_SECS);
    }
    damage = Math.clamp(damage, 0.0, 1.0);
  }

  /**
   * Returns the accumulated damage, from 0 (cool) to 1 (the breaker would trip).
   *
   * @return the accumulated damage
   */
  public double getDamage() {
    return damage;
  }

  /**
   * Returns the largest constant current that the breaker can carry for the specified period
   * without its damage exceeding the specified maximum. This is the current whose trip time is the
   * period divided by the remaining damage.
   *
   * <p>A current at or below the breaker's rating never adds damage; so, if the remaining damage
   * allows only currents at or just above the rating, the rated current is returned. If the damage
   * already exceeds the maximum, zero is returned.
   *
   * @param periodSecs the period over which the current is carried, in seconds
   * @param maxDamage the maximum damage, from 0 to 1 (e.g., 0.95 to stay 5% below tripping)
   * @return the maximum current in amps
   */
  public double calculateMaxCurrent(double periodSecs, double maxDamage) {
    double remainingDamage = maxDamage - damage;
    if (remainingDamage <= 0.0) {
      return 0.0;
    }

    double requiredTripTimeSecs = periodSecs / remainingDamage;
    if (requiredTripTimeSecs >= SENTINEL_TRIP_TIME_SECS) {
      return RATED_CURRENT_AMPS;
    }
    double minTripTimeSecs = TRIP_TIME_POINTS_SECS[TRIP_TIME_POINTS_SECS.length - 1];
    if (requiredTripTimeSecs <= minTripTimeSecs) {
      return NORMALIZED_CURRENT_POINTS[NORMALIZED_CURRENT_POINTS.length - 1] * RATED_CURRENT_AMPS;
    }

    return Math.exp(logInverseTripTimeMap.get(Math.log(requiredTripTimeSecs))) * RATED_CURRENT_AMPS;
  }

  /**
   * Returns the time for the breaker to trip at the specified constant current, interpolated from
   * the datasheet's trip curve.
   *
   * @param normalizedCurrent the current as a multiple of the breaker's rated current
   * @return the time to trip in seconds
   */
  private static double getTripTimeSecs(double normalizedCurrent) {
    return Math.exp(logTripTimeMap.get(Math.log(Math.max(normalizedCurrent, 1.0))));
  }
}
