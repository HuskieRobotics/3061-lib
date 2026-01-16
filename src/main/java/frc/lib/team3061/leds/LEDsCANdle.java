package frc.lib.team3061.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team3061.RobotConfig;

public class LEDsCANdle extends LEDs {
  private final CANdle candle;
  private final Color8Bit[] ledBuffer;

  protected LEDsCANdle() {

    candle = new CANdle(22, RobotConfig.getInstance().getCANBus());

    CANdleConfiguration configSettings = new CANdleConfiguration();
    configSettings.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
    configSettings.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;
    configSettings.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
    configSettings.LED.StripType = StripTypeValue.GRB;
    configSettings.LED.BrightnessScalar = 0.1;
    candle.getConfigurator().apply(configSettings, 100);

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
      candle.setControl(new SolidColor(i, i + count - 1).withColor(new RGBWColor(ledBuffer[i])));
      i += (count - 1);
    }
  }

  @Override
  protected void setLEDBuffer(int index, Color color) {
    ledBuffer[index] = convertTo8BitColor(color);
    if (MIRROR_LEDS) {
      ledBuffer[ACTUAL_LENGTH - index - 1] = convertTo8BitColor(color);
    }
  }

  @Override
  public Color8Bit getColor(int index) {
    return ledBuffer[index];
  }

  private static Color8Bit convertTo8BitColor(Color color) {
    return new Color8Bit(
        (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }
}
