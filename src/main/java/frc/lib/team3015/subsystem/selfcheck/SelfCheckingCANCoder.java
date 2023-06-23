package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderFaults;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingCANCoder implements SelfChecking {
  private final String label;
  private final CANCoder canCoder;

  public SelfCheckingCANCoder(String label, CANCoder canCoder) {
    this.label = label;
    this.canCoder = canCoder;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    CANCoderFaults f = new CANCoderFaults();
    canCoder.getFaults(f);

    if (f.HardwareFault) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (f.ResetDuringEn) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (f.MagnetTooWeak) {
      faults.add(new SubsystemFault(String.format("[%s]: Magnet too weak", label)));
    }

    ErrorCode err = canCoder.getLastError();
    if (err != ErrorCode.OK) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: Error Code (%s)", label, err.name()), err.value > 0));
    }

    return faults;
  }
}
