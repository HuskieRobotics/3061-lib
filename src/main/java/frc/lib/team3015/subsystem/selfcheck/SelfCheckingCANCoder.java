package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.hardware.CANcoder;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingCANCoder implements SelfChecking {
  private final String label;
  private final CANcoder canCoder;

  public SelfCheckingCANCoder(String label, CANcoder canCoder) {
    this.label = label;
    this.canCoder = canCoder;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (canCoder.getFault_Hardware().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (canCoder.getFault_BootDuringEnable().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (canCoder.getFault_BadMagnet().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Bad magnet", label)));
    }
    if (canCoder.getFault_Undervoltage().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device supply voltage near brownout", label)));
    }
    if (canCoder.getFault_UnlicensedFeatureInUse().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Unlicensed feature in use", label)));
    }

    return faults;
  }
}
