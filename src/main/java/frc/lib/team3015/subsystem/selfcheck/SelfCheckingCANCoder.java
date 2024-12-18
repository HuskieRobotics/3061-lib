package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingCANCoder implements SelfChecking {
  private final String label;
  private final CANcoder canCoder;
  private StatusSignal<Voltage> statusSignal;

  public SelfCheckingCANCoder(String label, CANcoder canCoder) {
    this.label = label;
    this.canCoder = canCoder;
    this.statusSignal = this.canCoder.getSupplyVoltage().clone();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (canCoder.getFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (canCoder.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (canCoder.getFault_BadMagnet().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Bad magnet", label)));
    }
    if (canCoder.getFault_Undervoltage().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device supply voltage near brownout", label)));
    }
    if (canCoder.getFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Unlicensed feature in use", label)));
    }

    this.statusSignal.refresh();
    if (this.statusSignal.getStatus() != StatusCode.OK) {
      faults.add(new SubsystemFault(String.format("[%s]: device is unreachable", label)));
    }

    return faults;
  }
}
