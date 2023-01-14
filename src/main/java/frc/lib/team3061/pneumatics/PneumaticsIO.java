// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.lib.team3061.pneumatics;

import org.littletonrobotics.junction.AutoLog;

@java.lang.SuppressWarnings({"java:S1104"})

/**
 * Pneumatics subsystem hardware abstraction interface.
 *
 * <p>The pneumatics hardware abstraction is designed to for two pressure sensors. One measures the
 * pressure upstream of the regulator (high pressure); the other, pressure downstream of the
 * regulator (low pressure). In addition, there is a flow sensor. (We use the SMC PFM711-N7-C-R.)
 */
public interface PneumaticsIO {
  @AutoLog
  public static class PneumaticsIOInputs {
    double highPressurePSI = 0.0;
    double lowPressurePSI = 0.0;
    boolean compressorActive = false;
    double compressorCurrentAmps = 0.0;
    double flowLPM = 0.0;
    double volumeL = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PneumaticsIOInputs inputs) {}

  /**
   * Updates the compressor threshold.
   *
   * <p>This method enables different pressures to be used for different periods of the robot's
   * operation. For example, the end-game may have different requirements than the rest of the
   * match.
   *
   * @param useLow if true, use the low pressure thresholds defined in PneumaticsConstants;
   *     otherwise, use the high pressure thresholds
   */
  public default void useLowClosedLoopThresholds(boolean useLow) {}
}
