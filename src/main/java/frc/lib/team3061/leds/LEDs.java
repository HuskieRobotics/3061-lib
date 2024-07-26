// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.team3061.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.RobotConfig;
import frc.robot.Constants;
import frc.robot.Field2d;
import java.util.List;

@java.lang.SuppressWarnings({"java:S6548"})
public abstract class LEDs extends SubsystemBase {

  private static LEDs instance;

  public static LEDs getInstance() {
    if (instance == null) {
      if (RobotConfig.getInstance().getLEDHardware() == RobotConfig.LED_HARDWARE.CANDLE) {
        instance = new LEDsCANdle();
      } else {
        instance = new LEDsRIO();
      }
    }
    return instance;
  }

  // Robot state tracking
  private int loopCycleCount = 0;
  private boolean fallen = false;
  private boolean endgameAlert = false;
  private boolean autoFinished = false;
  private double autoFinishedTime = 0.0;
  private boolean lowBatteryAlert = false;

  private boolean assignedAlliance = false;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  // LED IO
  private final Notifier loadingNotifier;

  // Constants

  /*
   * The LEDs are a continuous strand. However, we want to model them as two separate strands,
   * where the start and end of the single strand is modeled as the start of each separate strand.
   * This is handled by specifying the length as half the actual length and mirroring the buffer
   * before updating the LEDs.
   */

  protected static final boolean MIRROR_LEDS = true;
  protected static final int ACTUAL_LENGTH = RobotConfig.getInstance().getLEDCount();
  protected static final int LENGTH = MIRROR_LEDS ? ACTUAL_LENGTH / 2 : ACTUAL_LENGTH;
  private static final int STATIC_LENGTH = LENGTH / 2;
  private static final int STATIC_SECTION_LENGTH = STATIC_LENGTH / 3;
  private static final boolean PRIDE_LEDS = false;
  private static final int MIN_LOOP_CYCLE_COUNT = 10;
  private static final double STROBE_FAST_DURATION = 0.1;
  private static final double STROBE_SLOW_DURATION = 0.2;
  private static final double BREATH_DURATION = 1.0;
  private static final double PULSE_DURATION = 0.5;
  private static final double RAINBOW_CYCLE_LENGTH = 30.0;
  private static final double RAINBOW_DURATION = .25;
  private static final double WAVE_EXPONENT = 0.4;
  private static final double WAVE_FAST_CYCLE_LENGTH = 25.0;
  private static final double WAVE_FAST_DURATION = 0.25;
  private static final double WAVE_MEDIUM_DURATION = 0.75;
  private static final double WAVE_SLOW_CYCLE_LENGTH = 25.0;
  private static final double WAVE_SLOW_DURATION = 3.0;
  private static final double WAVE_ALLIANCE_CYCLE_LENGTH = 15.0;
  private static final double WAVE_ALLIANCE_DURATION = 2.0;
  private static final double AUTO_FADE_TIME = 2.5; // 3s nominal
  private static final double AUTO_FADE_MAX_TIME = 5.0; // Return to normal

  protected LEDs() {
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    Section.FULL, Color.kWhite, Color.kBlack, System.currentTimeMillis() / 1000.0);
                this.updateLEDs();
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  @Override
  public synchronized void periodic() {
    // update all state variables
    updateState();

    // exit during initial cycles
    if (++loopCycleCount < MIN_LOOP_CYCLE_COUNT) {
      return;
    }

    // stop loading notifier if running
    loadingNotifier.stop();

    // select LED mode
    updateLEDPattern();

    // Update LEDs
    this.updateLEDs();
  }

  private void updateLEDPattern() {
    // default to off
    solid(Section.FULL, Color.kBlack);

    if (estopped) {
      solid(Section.FULL, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < AUTO_FADE_MAX_TIME) {
        // Auto fade
        solid(1.0 - ((Timer.getFPGATimestamp() - lastEnabledTime) / AUTO_FADE_TIME), Color.kGreen);

      } else if (Constants.DEMO_MODE) {
        // Pride stripes
        updateToPridePattern();

      } else if (lowBatteryAlert) {
        // Low battery
        solid(Section.FULL, new Color(255, 20, 0));

      } else {
        // Default pattern
        updateToDisabledPattern();
      }
    } else if (fallen) {
      strobe(Section.FULL, Color.kWhite, STROBE_FAST_DURATION);
    } else if (DriverStation.isAutonomous()) {
      updateToAutoPattern();
    } else { // teleop

      updateToTeleopPattern();
    }
  }

  private void updateToTeleopPattern() {
    // FIXME: add other patterns here based specific to the game

    // Set special modes

    // Demo mode background
    if (Constants.DEMO_MODE) {
      wave(
          Section.FULL,
          new Color(255, 30, 0),
          Color.kDarkBlue,
          WAVE_SLOW_CYCLE_LENGTH,
          WAVE_MEDIUM_DURATION);
    }

    if (endgameAlert) {
      // Endgame alert
      strobe(Section.FULL, Color.kYellow, STROBE_SLOW_DURATION);
    }
  }

  private void updateToAutoPattern() {
    orangePulse(Section.FULL, PULSE_DURATION);

    // if (autoFinished) {
    //   double fullTime = LENGTH / WAVE_FAST_CYCLE_LENGTH * WAVE_FAST_DURATION;
    //   solid((Timer.getFPGATimestamp() - autoFinishedTime) / fullTime, Color.kGreen);
    // }
  }

  private void updateToDisabledPattern() {
    if (assignedAlliance) {
      if (Field2d.getInstance().getAlliance() == Alliance.Red) {
        wave(
            Section.FULL,
            Color.kRed,
            Color.kBlack,
            WAVE_ALLIANCE_CYCLE_LENGTH,
            WAVE_ALLIANCE_DURATION);
      } else {
        wave(
            Section.FULL,
            Color.kBlue,
            Color.kBlack,
            WAVE_ALLIANCE_CYCLE_LENGTH,
            WAVE_ALLIANCE_DURATION);
      }
    } else {
      wave(
          Section.FULL,
          new Color(255, 30, 0),
          Color.kDarkBlue,
          WAVE_SLOW_CYCLE_LENGTH,
          WAVE_SLOW_DURATION);
    }
  }

  private void updateToPridePattern() {
    stripes(
        Section.FULL,
        List.of(
            Color.kBlack,
            Color.kRed,
            Color.kOrangeRed,
            Color.kYellow,
            Color.kGreen,
            Color.kBlue,
            Color.kPurple,
            Color.kBlack,
            new Color(0.15, 0.3, 1.0),
            Color.kDeepPink,
            Color.kWhite,
            Color.kDeepPink,
            new Color(0.15, 0.3, 1.0)),
        3,
        5.0);
    switch (Field2d.getInstance().getAlliance()) {
      case Red:
        solid(Section.STATIC_LOW, Color.kRed);
        break;
      case Blue:
        solid(Section.STATIC_LOW, Color.kBlue);
        break;
      default:
        break;
    }
  }

  private void updateState() {
    // check for alliance assignment when connected to FMS
    if (DriverStation.isFMSAttached()) {
      assignedAlliance = true;
    } else {
      assignedAlliance = false;
    }

    // Update auto state
    if (DriverStation.isDisabled()) {
      autoFinished = false;
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }
  }

  public void setFallen(boolean fallen) {
    this.fallen = fallen;
  }

  public void setEndgameAlert(boolean endgameAlert) {
    this.endgameAlert = endgameAlert;
  }

  public void setAutoFinished(boolean autoFinished) {
    this.autoFinished = autoFinished;
    if (autoFinished) {
      this.autoFinishedTime = Timer.getFPGATimestamp();
    }
  }

  public void setLowBatteryAlert(boolean lowBatteryAlert) {
    this.lowBatteryAlert = lowBatteryAlert;
  }

  protected abstract void updateLEDs();

  protected abstract void setLEDBuffer(int index, Color color);

  private void solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        setLEDBuffer(i, color);
      }
    }
  }

  private void solid(double percent, Color color) {
    if (color != null) {
      for (int i = 0; i < MathUtil.clamp(LENGTH * percent, 0, LENGTH); i++) {
        setLEDBuffer(i, color);
      }
    }
  }

  private void strobe(Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  private void breath(Section section, Color c1, Color c2) {
    breath(section, c1, c2, Timer.getFPGATimestamp());
  }

  private void breath(Section section, Color c1, Color c2, double timestamp) {
    double x = ((timestamp % BREATH_DURATION) / BREATH_DURATION) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start()) {
        setLEDBuffer(i, Color.fromHSV((int) x, 255, 255));
      }
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), WAVE_EXPONENT) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), WAVE_EXPONENT) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        setLEDBuffer(i, new Color(red, green, blue));
      }
    }
  }

  private void fire(Section section, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double[] heat = new double[section.end() - section.start()];
    double xDiffPerLed = (2.0 * Math.PI) / heat.length;

    for (int i = 0; i < heat.length; i++) {
      x += xDiffPerLed;
      heat[i] = (Math.sin(x) + 1.0) / 2.0; // Heat level between 0 and 1
    }

    for (int i = 0; i < heat.length; i++) {
      double ratio = heat[i];
      // Use shades of blue and orange for the fire effect
      int red = (int) (255 * ratio);
      int green = (int) (20 * ratio);
      int blue = 0; // Blend blue and orange

      // Simulate rising and falling effect
      double sinValue = Math.sin(x + (i * 0.2));
      int offset = (int) ((sinValue + 1) / 2 * 255); // Scale to 0-255

      // Apply the color and intensity to the LED
      setLEDBuffer(
          section.start() + i,
          new Color(
              Math.max(0, red - offset), Math.max(0, green - offset), Math.max(0, blue - offset)));
    }
  }

  private void orangePulse(Section section, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double[] heat = new double[section.end() - section.start()];
    double xDiffPerLed = (2.0 * Math.PI) / heat.length;

    for (int i = 0; i < heat.length; i++) {
      x += xDiffPerLed;
      heat[i] = (Math.sin(x) + 1.0) / 2.0; // Heat level between 0 and 1
    }

    for (int i = 0; i < heat.length; i++) {
      double ratio = heat[i];

      // Orange color
      int red = (int) (255 * ratio);
      int green = (int) (30 * ratio);
      int blue = 0;
      // int blue = (int) (10 * ratio);

      // Simulate rising and falling effect
      int offset = (int) (2 * Math.sin(x + (i * 0.2)));

      // Apply the color to the LED
      setLEDBuffer(
          section.start() + i,
          new Color(
              Math.max(0, red - offset), Math.max(0, green - offset), Math.max(0, blue - offset)));
    }
  }

  private void stripes(Section section, List<Color> colors, int length, double duration) {
    int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
    for (int i = section.start(); i < section.end(); i++) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      setLEDBuffer(i, colors.get(colorIndex));
    }
  }

  private enum Section {
    STATIC,
    SHOULDER,
    FULL,
    STATIC_LOW,
    STATIC_MID,
    STATIC_HIGH;

    private int start() {
      switch (this) {
        case STATIC:
          return 0;
        case SHOULDER:
          return STATIC_LENGTH;
        case FULL:
          return 0;
        case STATIC_LOW:
          return 0;
        case STATIC_MID:
          return STATIC_SECTION_LENGTH;
        case STATIC_HIGH:
          return STATIC_LENGTH - STATIC_SECTION_LENGTH;
        default:
          return 0;
      }
    }

    private int end() {
      switch (this) {
        case STATIC:
          return STATIC_LENGTH;
        case SHOULDER:
          return LENGTH;
        case FULL:
          return LENGTH;
        case STATIC_LOW:
          return STATIC_SECTION_LENGTH;
        case STATIC_MID:
          return STATIC_LENGTH - STATIC_SECTION_LENGTH;
        case STATIC_HIGH:
          return STATIC_LENGTH;
        default:
          return LENGTH;
      }
    }
  }
}
