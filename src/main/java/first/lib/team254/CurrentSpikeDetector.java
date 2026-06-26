package first.lib.team254;

import java.util.function.BooleanSupplier;
import org.wpilib.command2.button.Trigger;
import org.wpilib.system.Timer;

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
