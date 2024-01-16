package frc.lib.team3061.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team3061.RobotConfig;

public class LEDsCANdle extends LEDs {
  private final CANdle candle;
  private final Color8Bit[] ledBuffer;

  protected LEDsCANdle() {

    candle = new CANdle(22, RobotConfig.getInstance().getCANBusName());

    CANdleConfiguration configSettings = new CANdleConfiguration();
    configSettings.statusLedOffWhenActive = true;
    configSettings.disableWhenLOS = false;
    configSettings.stripType = LEDStripType.GRB;
    configSettings.brightnessScalar = .1;
    configSettings.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(configSettings, 100);

    ledBuffer = new Color8Bit[ACTUAL_LENGTH];
    for (int i = 0; i < ACTUAL_LENGTH; i++) {
      ledBuffer[i] = convertTo8BitColor(Color.kBlack);
    }
  }

  @Override
  protected void updateLEDs() {
    for (int i = 0; i < ACTUAL_LENGTH; i++) {
      int count = 1;
      while (i + count < ACTUAL_LENGTH && ledBuffer[i] == ledBuffer[i + count]) {
        count++;
      }
      candle.setLEDs(ledBuffer[i].red, ledBuffer[i].green, ledBuffer[i].blue, 0, i, count);
    }
  }

  @Override
  protected void setLEDBuffer(int index, Color color) {
    ledBuffer[index] = convertTo8BitColor(color);
    if (MIRROR_LEDS) {
      ledBuffer[ACTUAL_LENGTH - index - 1] = convertTo8BitColor(color);
    }
  }

  private static Color8Bit convertTo8BitColor(Color color) {
    return new Color8Bit(
        (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }
}
