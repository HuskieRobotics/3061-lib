package frc.lib.team254;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class CurrentSpikeDetector implements BooleanSupplier {
  private double currentThresholdAmps;
  private double timeThresholdSeconds;
  private Timer currentOverThresholdTimer;
  private Trigger cachedTrigger;
  private boolean lastValue = false;

  public CurrentSpikeDetector(double currentThresholdAmps, double timeThresholdSeconds) {
    this.currentThresholdAmps = currentThresholdAmps;
    this.timeThresholdSeconds = timeThresholdSeconds;
    this.currentOverThresholdTimer = new Timer();
  }

  public boolean update(double current) {
    if (current > currentThresholdAmps) {
      currentOverThresholdTimer.start();

      boolean newValue = currentOverThresholdTimer.hasElapsed(timeThresholdSeconds);
      lastValue = newValue;
      return newValue;
    } else {
      currentOverThresholdTimer.stop();
      currentOverThresholdTimer.reset();
      lastValue = false;
      return false;
    }
  }

  public boolean getAsBoolean() {
    return lastValue;
  }

  public Trigger asTrigger() {
    if (cachedTrigger == null) {
      cachedTrigger = new Trigger(this);
    }
    return cachedTrigger;
  }
}
