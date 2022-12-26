// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.lib.team3061.pneumatics;

import static frc.lib.team3061.pneumatics.PneumaticsConstants.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticHub;

/**
 * Implementation of PneumaticsIO for REV Robotics Pneumatics Hub, dual REV Robotics pressure
 * sensors, and an analog flow sensor (i.e., SMC PFM711-N7-C-R.)
 */
public class PneumaticsIORev implements PneumaticsIO {

  private final PneumaticHub pneumatics;
  private final AnalogInput flowSensor;

  public PneumaticsIORev() {
    pneumatics = new PneumaticHub(PNEUMATICS_HUB_ID);
    flowSensor = new AnalogInput(FLOW_SENSOR_CHANNEL);
    useLowClosedLoopThresholds(false);
  }

  @Override
  public void updateInputs(PneumaticsIOInputs inputs) {
    inputs.highPressurePSI = pneumatics.getPressure(REV_HIGH_PRESSURE_SENSOR_CHANNEL);
    inputs.lowPressurePSI = pneumatics.getPressure(REV_LOW_PRESSURE_SENSOR_CHANNEL);
    inputs.compressorActive = pneumatics.getCompressor();
    inputs.compressorCurrentAmps = pneumatics.getCompressorCurrent();

    /*
     * Our SMC flow sensor (PFM711-N7-C-R) provides analog output from 1V to 5V.
     * 1V corresponds to 0 L/min; 5V corresponds to 100 L/min.
     */
    inputs.flowLPM = ((flowSensor.getAverageVoltage() * 25) - 25);
  }

  @Override
  public void useLowClosedLoopThresholds(boolean useLow) {
    if (useLow) {
      pneumatics.enableCompressorAnalog(MIN_LOW_PRESSURE, MAX_LOW_PRESSURE);
    } else {
      pneumatics.enableCompressorAnalog(MIN_HIGH_PRESSURE, MAX_HIGH_PRESSURE);
    }
  }
}
