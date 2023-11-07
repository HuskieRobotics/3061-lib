package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPigeon2 implements SelfChecking {
  private final String label;
  private final Pigeon2 pigeon;

  public SelfCheckingPigeon2(String label, Pigeon2 pigeon) {
    this.label = label;
    this.pigeon = pigeon;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (pigeon.getFault_Hardware().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (pigeon.getFault_BootDuringEnable().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (pigeon.getFault_BootIntoMotion().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while in motion", label)));
    }
    if (pigeon.getFault_BootupGyroscope().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Gyro fault detected", label)));
    }
    if (pigeon.getFault_BootupAccelerometer().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Accelerometer fault detected", label)));
    }
    if (pigeon.getFault_BootupMagnetometer().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Magnetometer fault detected", label)));
    }
    if (pigeon.getFault_DataAcquiredLate().getValue()) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: Motion stack data acquisition slower than expected", label)));
    }
    if (pigeon.getFault_Hardware().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware failure", label)));
    }
    if (pigeon.getFault_LoopTimeSlow().getValue()) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: Motion stack loop time was slower than expected", label)));
    }
    if (pigeon.getFault_SaturatedAccelometer().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Accelerometer values are saturated", label)));
    }
    if (pigeon.getFault_SaturatedGyroscope().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Gyro values are saturated", label)));
    }
    if (pigeon.getFault_SaturatedMagnetometer().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Magnetometer values are saturated", label)));
    }
    if (pigeon.getFault_Undervoltage().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device supply voltage near brownout", label)));
    }
    if (pigeon.getFault_UnlicensedFeatureInUse().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Unlicensed feature in use", label)));
    }

    return faults;
  }
}
