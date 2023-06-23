// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.ejml.simple.SimpleMatrix;

@java.lang.SuppressWarnings({"java:S106", "java:S107"})
public class FeedForwardCharacterization extends CommandBase {
  private static final double START_DELAY_SECS = 2.0;
  // FIXME: tune such that we hit 12 V before we run out of carpet
  private static final double RAMP_RATE_VOLTS_PER_SECOND = 0.5;

  private final boolean forwards;
  private final boolean isDrive;

  private final FeedForwardCharacterizationData dataPrimary;
  private final FeedForwardCharacterizationData dataSecondary;
  private final Consumer<Double> voltageConsumerSimple;
  private final BiConsumer<Double, Double> voltageConsumerDrive;
  private final Supplier<Double> velocitySupplierPrimary;
  private final Supplier<Double> velocitySupplierSecondary;
  private final Supplier<Double> accelerationSupplierPrimary;
  private final Supplier<Double> accelerationSupplierSecondary;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization for a drive. */
  public FeedForwardCharacterization(
      Subsystem drive,
      boolean forwards,
      FeedForwardCharacterizationData leftData,
      FeedForwardCharacterizationData rightData,
      BiConsumer<Double, Double> voltageConsumer,
      Supplier<Double> leftVelocitySupplier,
      Supplier<Double> rightVelocitySupplier,
      Supplier<Double> leftAccelerationSupplier,
      Supplier<Double> rightAccelerationSupplier) {
    addRequirements(drive);
    this.forwards = forwards;
    this.isDrive = true;
    this.dataPrimary = leftData;
    this.dataSecondary = rightData;
    this.voltageConsumerSimple = null;
    this.voltageConsumerDrive = voltageConsumer;
    this.velocitySupplierPrimary = leftVelocitySupplier;
    this.velocitySupplierSecondary = rightVelocitySupplier;
    this.accelerationSupplierPrimary = leftAccelerationSupplier;
    this.accelerationSupplierSecondary = rightAccelerationSupplier;
  }

  /** Creates a new FeedForwardCharacterization for a simple subsystem. */
  public FeedForwardCharacterization(
      Subsystem subsystem,
      boolean forwards,
      FeedForwardCharacterizationData data,
      Consumer<Double> voltageConsumer,
      Supplier<Double> velocitySupplier,
      Supplier<Double> accelerationSupplier) {
    addRequirements(subsystem);
    this.forwards = forwards;
    this.isDrive = false;
    this.dataPrimary = data;
    this.dataSecondary = null;
    this.voltageConsumerSimple = voltageConsumer;
    this.voltageConsumerDrive = null;
    this.velocitySupplierPrimary = velocitySupplier;
    this.velocitySupplierSecondary = null;
    this.accelerationSupplierPrimary = accelerationSupplier;
    this.accelerationSupplierSecondary = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < START_DELAY_SECS) {
      if (isDrive) {
        voltageConsumerDrive.accept(0.0, 0.0);
      } else {
        voltageConsumerSimple.accept(0.0);
      }
    } else {
      double voltage =
          (timer.get() - START_DELAY_SECS) * RAMP_RATE_VOLTS_PER_SECOND * (forwards ? 1 : -1);
      if (isDrive) {
        voltageConsumerDrive.accept(voltage, voltage);
      } else {
        voltageConsumerSimple.accept(voltage);
      }
      dataPrimary.add(accelerationSupplierPrimary.get(), velocitySupplierPrimary.get(), voltage);
      if (isDrive) {
        dataSecondary.add(
            accelerationSupplierSecondary.get(), velocitySupplierSecondary.get(), voltage);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (isDrive) {
      voltageConsumerDrive.accept(0.0, 0.0);
    } else {
      voltageConsumerSimple.accept(0.0);
    }
    timer.stop();
    dataPrimary.print();
    if (isDrive) {
      dataSecondary.print();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double voltage =
        (timer.get() - START_DELAY_SECS) * RAMP_RATE_VOLTS_PER_SECOND * (forwards ? 1 : -1);
    return voltage >= 12.0;
  }

  public static class FeedForwardCharacterizationData {
    private final String name;
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();
    private final List<Double> accelerationData = new LinkedList<>();

    public FeedForwardCharacterizationData(String name) {
      this.name = name;
    }

    public void add(double acceleration, double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        accelerationData.add(Math.abs(acceleration));
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {

      // We would prefer to use wpilib Matrix class, but it doesn't support
      //  an arbitrary number of row (at least over 20). If it is updated in the future,
      //  we should refactor this to use it.
      SimpleMatrix aMatrix = new SimpleMatrix(accelerationData.size(), 3);
      SimpleMatrix bMatrix = new SimpleMatrix(accelerationData.size(), 1);
      SimpleMatrix xMatrix;

      for (int i = 0; i < accelerationData.size(); i++) {
        aMatrix.set(i, 0, accelerationData.get(i));
        aMatrix.set(i, 1, velocityData.get(i));
        aMatrix.set(i, 2, 1);
        bMatrix.set(i, 0, voltageData.get(i));
      }

      // WPIMathJNI.solveFullPivHouseholderQr(
      //     aMatrix.getDDRM().getData(),
      //     aMatrix.numRows(),
      //     aMatrix.numCols(),
      //     bMatrix.getDDRM().getData(),
      //     bMatrix.numRows(),
      //     bMatrix.numCols(),
      //     xMatrix.getDDRM().getData());

      xMatrix = aMatrix.transpose().mult(aMatrix).invert().mult(aMatrix.transpose()).mult(bMatrix);

      SimpleMatrix residualVector = bMatrix.minus(aMatrix.mult(xMatrix));
      SimpleMatrix residualsSquaredSum = residualVector.mult(residualVector.transpose());
      double meanSquaredError = residualsSquaredSum.get(0, 0) / accelerationData.size();

      System.out.println("FF Characterization Results (" + name + "):");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tMSE (V^2)=%.5f", meanSquaredError));
      System.out.println(String.format("\tkS (V)=%.5f", xMatrix.get(2, 0)));
      System.out.println(String.format("\tkV (V/(m/s))=%.5f", xMatrix.get(1, 0)));
      System.out.println(String.format("\tkA (V/(m/s^2))=%.5f", xMatrix.get(0, 0)));
    }
  }
}
