package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingCANRange implements SelfChecking {
  private final String label;
  private final CANrange canRange;
  private StatusSignal<Voltage> statusSignal;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final List<SubsystemFault> faults = new ArrayList<>();

  public SelfCheckingCANRange(String label, CANrange canRange) {
    this.label = label;
    this.canRange = canRange;
    this.statusSignal = this.canRange.getSupplyVoltage().clone();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    faults.clear();

    // faults
    if (canRange.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (canRange.getFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (canRange.getFault_Undervoltage().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device supply voltage near brownout", label)));
    }
    if (canRange.getFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Unlicensed feature in use", label)));
    }

    // sticky faults
    if (canRange.getStickyFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Device booted while enabled", label)));
    }
    if (canRange.getStickyFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Hardware fault detected", label)));
    }
    if (canRange.getStickyFault_Undervoltage().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] Device supply voltage near brownout", label)));
    }
    if (canRange.getStickyFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Unlicensed feature in use", label)));
    }

    this.statusSignal.refresh();
    if (!connectedDebounce.calculate(this.statusSignal.getStatus().isOK())) {
      faults.add(new SubsystemFault(String.format("[%s]: device is unreachable", label)));
    }

    return faults;
  }

  @Override
  public void clearStickyFaults() {
    canRange.clearStickyFaults();
  }
}
