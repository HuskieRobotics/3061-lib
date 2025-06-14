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
  private final List<SubsystemFault> faults = new ArrayList<>();

  public SelfCheckingPigeon2(String label, Pigeon2 pigeon) {
    this.label = label;
    this.pigeon = pigeon;
    this.statusSignal = this.pigeon.getSupplyVoltage().clone();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    faults.clear();

    // faults
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

    // sticky faults
    if (pigeon.getStickyFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Device booted while enabled", label)));
    }
    if (pigeon.getStickyFault_BootIntoMotion().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Device booted while in motion", label)));
    }
    if (pigeon.getStickyFault_BootupAccelerometer().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Accelerometer fault detected", label)));
    }
    if (pigeon.getStickyFault_BootupGyroscope().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: [STICKY] Gyro fault detected", label)));
    }
    if (pigeon.getStickyFault_BootupMagnetometer().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Magnetometer fault detected", label)));
    }
    if (pigeon.getStickyFault_DataAcquiredLate().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: [STICKY] Motion stack data acquisition slower than expected", label)));
    }
    if (pigeon.getStickyFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Hardware fault detected", label)));
    }
    if (pigeon.getStickyFault_LoopTimeSlow().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: [STICKY] Motion stack loop time was slower than expected", label)));
    }
    if (pigeon.getStickyFault_SaturatedAccelerometer().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] Accelerometer values are saturated", label)));
    }
    if (pigeon.getStickyFault_SaturatedGyroscope().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Gyro values are saturated", label)));
    }
    if (pigeon.getStickyFault_SaturatedMagnetometer().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] Magnetometer values are saturated", label)));
    }
    if (pigeon.getStickyFault_Undervoltage().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] Device supply voltage near brownout", label)));
    }
    if (pigeon.getStickyFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Unlicensed feature in use", label)));
    }

    this.statusSignal.refresh();
    if (!connectedDebounce.calculate(this.statusSignal.getStatus().isOK())) {
      faults.add(new SubsystemFault(String.format("[%s]: [STICKY] device is unreachable", label)));
    }

    return faults;
  }

  @Override
  public void clearStickyFaults() {
    pigeon.clearStickyFaults();
  }
}
