package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPigeon2 implements SelfChecking {
  private final String label;
  private final Pigeon2 pigeon;
  private StatusSignal<Voltage> statusSignal;
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public SelfCheckingPigeon2(String label, Pigeon2 pigeon) {
    this.label = label;
    this.pigeon = pigeon;
    this.statusSignal = this.pigeon.getSupplyVoltage().clone();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (pigeon.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (pigeon.getFault_BootIntoMotion().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while in motion", label)));
    }
    if (pigeon.getFault_BootupAccelerometer().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Accelerometer fault detected", label)));
    }
    if (pigeon.getFault_BootupGyroscope().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Gyro fault detected", label)));
    }
    if (pigeon.getFault_BootupMagnetometer().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Magnetometer fault detected", label)));
    }
    if (pigeon.getFault_DataAcquiredLate().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: Motion stack data acquisition slower than expected", label)));
    }
    if (pigeon.getFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (pigeon.getFault_LoopTimeSlow().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: Motion stack loop time was slower than expected", label)));
    }
    if (pigeon.getFault_SaturatedAccelerometer().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Accelerometer values are saturated", label)));
    }
    if (pigeon.getFault_SaturatedGyroscope().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Gyro values are saturated", label)));
    }
    if (pigeon.getFault_SaturatedMagnetometer().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Magnetometer values are saturated", label)));
    }
    if (pigeon.getFault_Undervoltage().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device supply voltage near brownout", label)));
    }
    if (pigeon.getFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Unlicensed feature in use", label)));
    }

    this.statusSignal.refresh();
    if (!connectedDebounce.calculate(this.statusSignal.getStatus().isOK())) {
      faults.add(new SubsystemFault(String.format("[%s]: device is unreachable", label)));
    }

    return faults;
  }

  @Override
  public void clearStickyFaults() {
    pigeon.clearStickyFaults();
  }
}
