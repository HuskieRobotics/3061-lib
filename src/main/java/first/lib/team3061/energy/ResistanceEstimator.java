package first.lib.team3061.energy;

/**
 * Estimates the resistance and resting voltage of a voltage source (e.g., a battery) from samples
 * of its current and voltage, using the model V = V0 - R * I, where V0 is the resting
 * (open-circuit) voltage and R is the resistance.
 *
 * <p>The estimate is a least-squares fit of voltage against current over a sliding window of the
 * most recent samples. A fit is only accepted when the current varies enough within the window to
 * determine the slope (e.g., while the robot is accelerating and decelerating) and the resulting
 * resistance is plausible. Otherwise, the previous estimate is kept. So, the estimate updates while
 * the robot is being driven and holds while it is idle.
 *
 * <p>This simple model ignores a lead-acid battery's slower dynamics (e.g., its voltage continuing
 * to sag under a sustained load and recovering afterward), which a short window limits but doesn't
 * remove.
 */
public class ResistanceEstimator {
  private final double[] currentsAmps;
  private final double[] voltagesVolts;
  private final double minCurrentStdDevAmps;
  private final double maxResistanceOhms;

  private int sampleCount = 0;
  private int nextIndex = 0;

  private boolean valid = false;
  private double resistanceOhms = 0.0;
  private double restingVoltageVolts = 0.0;

  /**
   * Creates a new resistance estimator.
   *
   * @param windowSize the number of most recent samples to fit
   * @param minCurrentStdDevAmps the minimum standard deviation of the current within the window for
   *     a fit to be accepted
   * @param maxResistanceOhms the largest plausible resistance; fits with a larger (or a
   *     non-positive) resistance are rejected
   */
  public ResistanceEstimator(
      int windowSize, double minCurrentStdDevAmps, double maxResistanceOhms) {
    this.currentsAmps = new double[windowSize];
    this.voltagesVolts = new double[windowSize];
    this.minCurrentStdDevAmps = minCurrentStdDevAmps;
    this.maxResistanceOhms = maxResistanceOhms;
  }

  /**
   * Adds a sample and, once the window is full, fits the samples in the window.
   *
   * @param currentAmps the current drawn from the source, in amps (positive is discharge)
   * @param voltageVolts the voltage of the source, in volts
   */
  public void update(double currentAmps, double voltageVolts) {
    currentsAmps[nextIndex] = currentAmps;
    voltagesVolts[nextIndex] = voltageVolts;
    nextIndex = (nextIndex + 1) % currentsAmps.length;
    sampleCount = Math.min(sampleCount + 1, currentsAmps.length);

    if (sampleCount < currentsAmps.length) {
      return;
    }

    // two passes (means, then deviations) for numerical stability
    double meanCurrent = 0.0;
    double meanVoltage = 0.0;
    for (int i = 0; i < sampleCount; i++) {
      meanCurrent += currentsAmps[i];
      meanVoltage += voltagesVolts[i];
    }
    meanCurrent /= sampleCount;
    meanVoltage /= sampleCount;

    double currentVariance = 0.0;
    double covariance = 0.0;
    for (int i = 0; i < sampleCount; i++) {
      double currentDeviation = currentsAmps[i] - meanCurrent;
      currentVariance += currentDeviation * currentDeviation;
      covariance += currentDeviation * (voltagesVolts[i] - meanVoltage);
    }
    currentVariance /= sampleCount;
    covariance /= sampleCount;

    if (currentVariance < minCurrentStdDevAmps * minCurrentStdDevAmps) {
      return;
    }

    // the slope of voltage versus current is -R
    double fitResistanceOhms = -covariance / currentVariance;
    if (fitResistanceOhms <= 0.0 || fitResistanceOhms > maxResistanceOhms) {
      return;
    }

    valid = true;
    resistanceOhms = fitResistanceOhms;
    restingVoltageVolts = meanVoltage + fitResistanceOhms * meanCurrent;
  }

  /**
   * Returns whether a fit has been accepted since this estimator was created.
   *
   * @return true if the resistance and resting voltage have been estimated
   */
  public boolean isValid() {
    return valid;
  }

  /**
   * Returns the most recently accepted estimate of the resistance.
   *
   * @return the resistance in ohms, or 0 if no fit has been accepted
   */
  public double getResistanceOhms() {
    return resistanceOhms;
  }

  /**
   * Returns the most recently accepted estimate of the resting (open-circuit) voltage.
   *
   * @return the resting voltage in volts, or 0 if no fit has been accepted
   */
  public double getRestingVoltageVolts() {
    return restingVoltageVolts;
  }
}
