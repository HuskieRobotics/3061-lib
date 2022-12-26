// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.lib.team3061.pneumatics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** The pneumatics subsystem. */
public class Pneumatics extends SubsystemBase {
  private static final int NORMAL_AVERAGING_TAPS = 25;
  private static final int COMPRESSOR_AVERAGING_TAPS = 50;
  private static final double COMPRESSOR_RATE_PSI_PER_SEC = 1.5;

  private final PneumaticsIO io;
  private final PneumaticsIOInputsAutoLogged inputs = new PneumaticsIOInputsAutoLogged();

  private final List<Double> filterData = new ArrayList<>();
  private double pressureSmoothedPsi = 0.0;

  private double lastPressurePsi = 0.0;
  private boolean lastPressureIncreasing = false;
  private double compressorMaxPoint = 0.0;
  private double compressorMinPoint = 0.0;

  private Timer noPressureTimer = new Timer();
  private Timer compressorEnabledTimer = new Timer();
  private Alert dumpValveAlert =
      new Alert("Cannot build pressure. Is the dump value open?", AlertType.WARNING);

  /**
   * Create a new pneumatics subsystem.
   *
   * @param io the hardware abstracted pneumatics object
   */
  public Pneumatics(PneumaticsIO io) {
    this.io = io;
    noPressureTimer.start();
    compressorEnabledTimer.start();

    Shuffleboard.getTab("MAIN").addNumber("Pressure", this::getPressure);
  }

  public double getPressure() {
    return pressureSmoothedPsi;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Pneumatics", inputs);

    calculateAveragePressure();

    // Log pressure
    Logger.getInstance().recordOutput("PressurePsi", pressureSmoothedPsi);

    // Detect if dump value is open
    if (inputs.highPressurePSI > 3) {
      noPressureTimer.reset();
    }
    if (!inputs.compressorActive) {
      compressorEnabledTimer.reset();
    }
    dumpValveAlert.set(noPressureTimer.hasElapsed(5) && compressorEnabledTimer.hasElapsed(5));
  }

  private void calculateAveragePressure() {
    // Calculate input pressure for averaging filter
    double limitedPressure = inputs.highPressurePSI < 0.0 ? 0.0 : inputs.highPressurePSI;
    double processedPressure;
    if (inputs.compressorActive) {
      // When compressor is active, average the most recent min and max points
      processedPressure = averagePressures(limitedPressure);
    } else {
      // When compressor is inactive, reset min/max status and use normal pressure
      processedPressure = resetPressures(limitedPressure);
    }

    // Run averaging filter
    filterData.add(processedPressure);
    while (filterData.size()
        > (inputs.compressorActive ? COMPRESSOR_AVERAGING_TAPS : NORMAL_AVERAGING_TAPS)) {
      filterData.remove(0);
    }
    pressureSmoothedPsi = filterData.stream().mapToDouble(a -> a).summaryStatistics().getAverage();
  }

  private double averagePressures(double limitedPressure) {
    if (limitedPressure != lastPressurePsi) {
      boolean increasing = limitedPressure > lastPressurePsi;
      if (increasing != lastPressureIncreasing) {
        if (increasing) {
          compressorMinPoint = lastPressurePsi;
        } else {
          compressorMaxPoint = lastPressurePsi;
        }
      }
      lastPressurePsi = limitedPressure;
      lastPressureIncreasing = increasing;
    }
    double processedPressure = (compressorMinPoint + compressorMaxPoint) / 2.0;

    // Apply latency compensation
    processedPressure +=
        (COMPRESSOR_AVERAGING_TAPS / 2.0)
            * Constants.LOOP_PERIOD_SECS
            * COMPRESSOR_RATE_PSI_PER_SEC;

    return processedPressure;
  }

  private double resetPressures(double limitedPressure) {
    lastPressurePsi = limitedPressure;
    lastPressureIncreasing = false;
    compressorMaxPoint = limitedPressure;
    compressorMinPoint = limitedPressure;
    return limitedPressure;
  }
}
