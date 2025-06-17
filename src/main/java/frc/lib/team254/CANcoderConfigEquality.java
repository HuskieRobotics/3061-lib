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
public class CANcoderConfigEquality {

  public static final double PHOENIX_CONFIG_EPSILON = 0.05;

  /**
   * Returns true if the two specified CANcoder configurations are equal; otherwise, false.
   *
   * @param a first CANcoder configuration to compare
   * @param b second CANcoder configuration to compare
   * @return true if the two specified CANcoder configurations are equal; otherwise, false
   */
  public static boolean isEqual(CANcoderConfiguration a, CANcoderConfiguration b) {
    return isEqual(a.MagnetSensor, b.MagnetSensor);
  }

  /**
   * Returns true if the two specified magnet sensor configurations are equal; otherwise, false.
   *
   * @param a first magnet sensor configuration to compare
   * @param b second magnet sensor configuration to compare
   * @return true if the two specified magnet sensor configurations are equal; otherwise, false
   */
  public static boolean isEqual(MagnetSensorConfigs a, MagnetSensorConfigs b) {
    return MathUtil.isNear(
            a.AbsoluteSensorDiscontinuityPoint,
            b.AbsoluteSensorDiscontinuityPoint,
            PHOENIX_CONFIG_EPSILON)
        && MathUtil.isNear(a.MagnetOffset, b.MagnetOffset, PHOENIX_CONFIG_EPSILON)
        && a.SensorDirection == b.SensorDirection;
  }

  private CANcoderConfigEquality() {}
}
