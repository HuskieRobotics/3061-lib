// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Adapted for 3061-lib

package frc.lib.team3061.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.LoggedTracer;
import frc.robot.Constants;
import frc.robot.Field2d;
import java.util.List;
import java.util.TreeSet;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("unused")
public abstract class LEDs extends SubsystemBase {

  private static LEDs instance;

  public static LEDs getInstance() {
    if (instance == null) {
      instance = new LEDsRIO();
    }
    return instance;
  }

  /* based on FRC 6995's use of TreeSet to prioritize LED states as shared on CD:
   * https://www.chiefdelphi.com/t/enums-and-subsytem-states/463974/31?u=gcschmit
   */

  private TreeSet<States> fullStates = new TreeSet<>();
  private TreeSet<States> shoulderStates = new TreeSet<>();
  private TreeSet<States> staticStates = new TreeSet<>();
  private TreeSet<States> staticLowStates = new TreeSet<>();
  private TreeSet<States> staticMidStates = new TreeSet<>();
  private TreeSet<States> staticHighStates = new TreeSet<>();

  /**
   * Enum for LED states. Each state has a lambda function that accepts an LED subsystem and section
   * of the LED strip on which to display the specified pattern corresponding to the state.
   *
   * <p>The order of the states in the enum is the priority order for the states. The first state in
   * the enum is the highest priority state.
   */
  public enum States {
    ESTOPPED((leds, section) -> leds.solid(section, Color.kRed)),
    FALLEN((leds, section) -> leds.strobe(Section.FULL, Color.kWhite, STROBE_FAST_DURATION)),
    AUTO_FADE(
        (leds, section) ->
            leds.solid(
                1.0 - ((Timer.getFPGATimestamp() - leds.lastEnabledTime) / AUTO_FADE_TIME),
                Color.kGreen)),
    LOW_BATTERY((leds, section) -> leds.solid(section, new Color(255, 20, 0))),
    DISABLED_DEMO_MODE((leds, section) -> leds.updateToPridePattern()),
    NO_AUTO_SELECTED((leds, section) -> leds.solid(section, Color.kYellow)),
    DISABLED(LEDs::updateToDisabledPattern),
    AUTO((leds, section) -> leds.orangePulse(section, PULSE_DURATION)),
    ENDGAME_ALERT((leds, section) -> leds.strobe(section, Color.kYellow, STROBE_SLOW_DURATION)),
    UNTILTING_ROBOT((leds, section) -> leds.strobe(section, Color.kRed, STROBE_SLOW_DURATION)),
    ELEVATOR_JAMMED((leds, section) -> leds.strobe(section, Color.kBlue, STROBE_SLOW_DURATION)),
    DRIVE_TO_POSE_CANCELED(
        (leds, section) -> leds.strobe(section, Color.kPink, STROBE_SLOW_DURATION)),
    EJECTING_GAME_PIECE(
        (leds, section) -> leds.strobe(section, new Color(255, 20, 0), STROBE_SLOW_DURATION)),
    RELEASING_GAME_PIECE(
        (leds, section) -> leds.strobe(section, Color.kGreen, STROBE_SLOW_DURATION)),
    AUTO_DRIVING_TO_POSE((leds, section) -> leds.orangePulse(section, PULSE_DURATION)),
    AT_POSE((leds, section) -> leds.solid(section, Color.kGreen)),
    HAS_GAME_PIECE((leds, section) -> leds.solid(section, Color.kBlue)),
    INDEXING_GAME_PIECE((leds, section) -> leds.strobe(section, Color.kBlue, STROBE_SLOW_DURATION)),
    WAITING_FOR_GAME_PIECE(
        (leds, section) ->
            leds.wave(
                section,
                Color.kBlue,
                new Color(255, 20, 0),
                WAVE_FAST_CYCLE_LENGTH,
                WAVE_SLOW_DURATION)),
    DEFAULT((leds, section) -> leds.solid(section, Color.kBlack));

    public final BiConsumer<LEDs, Section> setter;

    private States(BiConsumer<LEDs, Section> setter) {
      this.setter = setter;
    }
  }

  // robot state tracking
  private int loopCycleCount = 0;
  private boolean assignedAlliance = false;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;

  private final Notifier loadingNotifier;

  // Constants

  /*
   * The LEDs are a continuous strand. However, we want to model them as two separate strands,
   * where the start and end of the single strand is modeled as the start of each separate strand.
   * This is handled by specifying the length as half the actual length and mirroring the buffer
   * before updating the LEDs.
   */
  protected static final boolean MIRROR_LEDS = true;
  protected static final int ACTUAL_LENGTH = 42; // RobotConfig.getInstance().getLEDCount();
  protected static final int LENGTH = MIRROR_LEDS ? ACTUAL_LENGTH / 2 : ACTUAL_LENGTH;
  private static final int STATIC_LENGTH = LENGTH / 2;
  private static final int STATIC_SECTION_LENGTH = STATIC_LENGTH / 3;
  private static final int MIN_LOOP_CYCLE_COUNT = 10;
  private static final double STROBE_FAST_DURATION = 0.1;
  private static final double STROBE_SLOW_DURATION = 0.2;
  private static final double BREATH_DURATION = 1.0;
  private static final double PULSE_DURATION = 0.5;
  private static final double RAINBOW_CYCLE_LENGTH = 30.0;
  private static final double RAINBOW_DURATION = .25;
  private static final double WAVE_EXPONENT = 0.4;
  private static final double WAVE_FAST_CYCLE_LENGTH = 25.0;
  private static final double WAVE_FAST_DURATION = 0.5;
  private static final double WAVE_MEDIUM_DURATION = 0.75;
  private static final double WAVE_SLOW_CYCLE_LENGTH = 25.0;
  private static final double WAVE_SLOW_DURATION = 3.0;
  private static final double WAVE_ALLIANCE_CYCLE_LENGTH = 15.0;
  private static final double WAVE_ALLIANCE_DURATION = 2.0;
  private static final double AUTO_FADE_TIME = 2.5; // 3s nominal
  private static final double AUTO_FADE_MAX_TIME = 5.0; // Return to normal

  // display a pattern while the code is loading
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

  /**
   * Request a state to be displayed on the LEDs. This method will display the state on the entire
   * (full) LED strip if it is the highest priority state.
   *
   * @param state the state to display
   */
  public void requestState(States state) {
    fullStates.add(state);
  }

  /**
   * Request a state to be displayed on the LEDs. This method will display the state on the
   * specified section of the LED strip if it is the highest priority state for that section.
   *
   * @param section the section of the LED strip on which to display the state
   * @param state the state to display
   */
  public void requestState(Section section, States state) {
    switch (section) {
      case FULL:
        fullStates.add(state);
        break;
      case SHOULDER:
        shoulderStates.add(state);
        break;
      case STATIC:
        staticStates.add(state);
        break;
      case STATIC_LOW:
        staticLowStates.add(state);
        break;
      case STATIC_MID:
        staticMidStates.add(state);
        break;
      case STATIC_HIGH:
        staticHighStates.add(state);
        break;
    }
  }

  @Override
  public synchronized void periodic() {
    // exit during initial cycles
    if (++loopCycleCount < MIN_LOOP_CYCLE_COUNT) {
      return;
    }

    // stop loading notifier if running
    loadingNotifier.stop();

    /* currently, we always use the entire (full) LED strip for all states
     * if in the future we use smaller sections for some states, we will need
     * to update this to add the default state to the appropriate sections.
     */
    this.requestState(Section.FULL, States.DEFAULT);

    // update internal state
    updateInternalState();

    /*
     * select LED mode
     *   if there is a state requested that use the full (entire) LED strip, display it
     *   otherwise, display the shoulder state and then,
     *      if there is a state requested that uses the static section, display it
     *      otherwise, display the static low, mid, and high requested states
     */
    if (!fullStates.isEmpty()) {
      States fullState = fullStates.first();
      fullState.setter.accept(this, Section.FULL);
      Logger.recordOutput("LEDS/state", fullState);
    } else {
      States shoulderState = shoulderStates.first();
      shoulderState.setter.accept(this, Section.SHOULDER);
      Logger.recordOutput("LEDS/shoulder state", shoulderState);
      if (!staticStates.isEmpty()) {
        States staticState = staticStates.first();
        staticState.setter.accept(this, Section.STATIC);
        Logger.recordOutput("LEDS/static state", staticState);
      } else {
        States staticLowState = staticLowStates.first();
        staticLowState.setter.accept(this, Section.STATIC_LOW);
        Logger.recordOutput("LEDS/low state", staticLowState);
        States staticMidState = staticMidStates.first();
        staticMidState.setter.accept(this, Section.STATIC_MID);
        Logger.recordOutput("LEDS/mid state", staticMidState);
        States staticHighState = staticHighStates.first();
        staticHighState.setter.accept(this, Section.STATIC_HIGH);
        Logger.recordOutput("LEDS/high state", staticHighState);
      }
    }

    // update LEDs
    this.updateLEDs();

    fullStates.clear();
    shoulderStates.clear();
    staticStates.clear();
    staticLowStates.clear();
    staticMidStates.clear();
    staticHighStates.clear();

    // Record cycle time
    LoggedTracer.record("LEDs");
  }

  private void updateToDisabledPattern(Section section) {
    if (assignedAlliance) {
      if (Field2d.getInstance().getAlliance() == Alliance.Red) {
        wave(section, Color.kRed, Color.kBlack, WAVE_ALLIANCE_CYCLE_LENGTH, WAVE_ALLIANCE_DURATION);
      } else {
        wave(
            section, Color.kBlue, Color.kBlack, WAVE_ALLIANCE_CYCLE_LENGTH, WAVE_ALLIANCE_DURATION);
      }
    } else {
      wave(
          section,
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

  private void updateInternalState() {
    // check for alliance assignment when connected to FMS
    if (DriverStation.isFMSAttached()) {
      assignedAlliance = true;
    } else {
      assignedAlliance = false;
    }

    // Update based on robot state
    if (DriverStation.isDisabled()) {
      this.requestState(States.DISABLED);
      if (lastEnabledAuto && Timer.getFPGATimestamp() - lastEnabledTime < AUTO_FADE_MAX_TIME) {
        this.requestState(States.AUTO_FADE);
      }
    } else {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getFPGATimestamp();

      if (DriverStation.isAutonomous()) {
        this.requestState(States.AUTO);
      }
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      this.requestState(States.ESTOPPED);
    }

    // update for demo mode
    if (Constants.DEMO_MODE && DriverStation.isDisabled()) {
      this.requestState(States.DISABLED_DEMO_MODE);
    }
  }

  protected abstract void updateLEDs();

  protected abstract void setLEDBuffer(int index, Color color);

  public abstract Color8Bit getColor(int index);

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

  public enum Section {
    FULL,
    SHOULDER,
    STATIC,
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
