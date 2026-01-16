// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Modeled after 254's TalonFXConfigEquality class

package frc.lib.team254;

import com.ctre.phoenix6.configs.*;
import edu.wpi.first.math.MathUtil;

@java.lang.SuppressWarnings({"java:S106"})
public class CANrangeConfigEquality {

  public static final double PHOENIX_CONFIG_EPSILON = 0.05;

  /**
   * Returns true if the two specified CANrange configurations are equal; otherwise, false.
   *
   * @param a first CANrange configuration to compare
   * @param b second CANrange configuration to compare
   * @return true if the two specified CANrange configurations are equal; otherwise, false
   */
  public static boolean isEqual(CANrangeConfiguration a, CANrangeConfiguration b) {
    return isEqual(a.FovParams, b.FovParams)
        && isEqual(a.ProximityParams, b.ProximityParams)
        && isEqual(a.ToFParams, b.ToFParams);
  }

  /**
   * Returns true if the two specified FoV params configurations are equal; otherwise, false.
   *
   * @param a first FoV params configuration to compare
   * @param b second FoV params configuration to compare
   * @return true if the two specified FoV params configurations are equal; otherwise, false
   */
  public static boolean isEqual(FovParamsConfigs a, FovParamsConfigs b) {
    return MathUtil.isNear(a.FOVCenterX, b.FOVCenterX, PHOENIX_CONFIG_EPSILON)
        && MathUtil.isNear(a.FOVCenterY, b.FOVCenterY, PHOENIX_CONFIG_EPSILON)
        && MathUtil.isNear(a.FOVRangeX, b.FOVRangeX, PHOENIX_CONFIG_EPSILON)
        && MathUtil.isNear(a.FOVRangeY, b.FOVRangeY, PHOENIX_CONFIG_EPSILON);
  }
  /**
   * Returns true if the two specified Proximity params configurations are equal; otherwise, false.
   *
   * @param a first Proximity params configuration to compare
   * @param b second Proximity params configuration to compare
   * @return true if the two specified Proximity params configurations are equal; otherwise, false
   */
  public static boolean isEqual(ProximityParamsConfigs a, ProximityParamsConfigs b) {
    return MathUtil.isNear(
            a.MinSignalStrengthForValidMeasurement,
            b.MinSignalStrengthForValidMeasurement,
            PHOENIX_CONFIG_EPSILON)
        && MathUtil.isNear(a.ProximityHysteresis, b.ProximityHysteresis, PHOENIX_CONFIG_EPSILON)
        && MathUtil.isNear(a.ProximityThreshold, b.ProximityThreshold, PHOENIX_CONFIG_EPSILON);
  }

  /**
   * Returns true if the two specified ToF params configurations are equal; otherwise, false.
   *
   * @param a first ToF params configuration to compare
   * @param b second ToF params configuration to compare
   * @return true if the two specified ToF params configurations are equal; otherwise, false
   */
  public static boolean isEqual(ToFParamsConfigs a, ToFParamsConfigs b) {
    return MathUtil.isNear(a.UpdateFrequency, b.UpdateFrequency, PHOENIX_CONFIG_EPSILON)
        && a.UpdateMode == b.UpdateMode;
  }

  private CANrangeConfigEquality() {}
}
