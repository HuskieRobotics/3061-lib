package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPhoenixMotor implements SelfChecking {
  private final String label;
  private final TalonFX motor;

  public SelfCheckingPhoenixMotor(String label, TalonFX motor) {
    this.label = label;
    this.motor = motor;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (motor.getFault_Hardware().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware failure detected", label)));
    }
    if (motor.getFault_BootDuringEnable().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (motor.getFault_DeviceTemp().getValue()) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: Device temperature exceeded limit", label), true));
    }
    if (motor.getFault_FusedSensorOutOfSync().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Remote sensor is out of sync", label)));
    }
    if (motor.getFault_OverSupplyV().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Supply voltage exceeded limit", label)));
    }
    if (motor.getFault_ProcTemp().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Processor temperature exceeded limit", label)));
    }
    if (motor.getFault_Undervoltage().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device supply voltage near brownout", label)));
    }
    if (motor.getFault_UnlicensedFeatureInUse().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Unlicensed feature in use", label)));
    }
    if (motor.getFault_UnstableSupplyV().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Supply voltage is unstable", label)));
    }

    return faults;
  }
}
