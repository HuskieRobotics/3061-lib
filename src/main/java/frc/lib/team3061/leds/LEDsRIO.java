package frc.lib.team3061.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class LEDsRIO extends LEDs {

  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private boolean isGRB;
  private boolean competitionBrightness;

  protected LEDsRIO() {
    leds = new AddressableLED(0);

    buffer = new AddressableLEDBuffer(ACTUAL_LENGTH);
    // leds.setBitTiming(500, 200, 1200, 1300);
    isGRB = Constants.getMode() != Constants.Mode.SIM;
    competitionBrightness = true;

    leds.setLength(ACTUAL_LENGTH);
    leds.setData(buffer);
    leds.start();
  }

  @Override
  protected void updateLEDs() {
    leds.setData(buffer);
  }

  @Override
  protected void setLEDBuffer(int index, Color color) {
    int h = 0;
    int s = 0;
    int v = 0;
    if (isGRB) {
      color = changeToGRB(color);
      int[] hsv =
          RGBToHSV((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
      h = hsv[0];
      s = hsv[1];
      v = hsv[2];
    }

    if (competitionBrightness) {
      buffer.setLED(index, color);
    } else {
      buffer.setHSV(index, h, s, v);
    }

    if (MIRROR_LEDS) {
      if (competitionBrightness) {
        buffer.setLED(ACTUAL_LENGTH - index - 1, color);
      } else {
        buffer.setHSV(ACTUAL_LENGTH - index - 1, h, s, v);
      }
    }
  }

  @Override
  public Color8Bit getColor(int index) {
    return new Color8Bit(buffer.getLED(index));
  }

  static Color changeToGRB(Color color) {
    return new Color(color.green, color.red, color.blue);
  }

  static int[] RGBToHSV(int r, int g, int b) {
    double red = r / 255.0;
    double green = g / 255.0;
    double blue = b / 255.0;

    double cMax = Math.max(red, Math.max(green, blue));
    double cMin = Math.min(red, Math.min(green, blue));

    double delta = cMax - cMin;

    // Hue
    int hue;

    if (delta == 0) {
      hue = 0;
    } else if (cMax == red) {
      hue = (int) Math.round(60 * (((green - blue) / delta) % 6));
    } else if (cMax == green) {
      hue = (int) Math.round(60 * (((blue - red) / delta) + 2));
    } else {
      hue = (int) Math.round(60 * (((red - green) / delta) + 4));
    }

    // Saturation
    double saturation = (cMax == 0) ? 0 : delta / cMax;

    // Convert final values to correct range

    return new int[] {
      hue / 2, (int) Math.round(saturation * 255 / 2), (int) Math.round(cMax * 255)
    };
  }
}
