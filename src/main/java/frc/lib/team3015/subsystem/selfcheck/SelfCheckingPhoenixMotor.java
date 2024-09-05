package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPhoenixMotor implements SelfChecking {
  private final String label;
  private final TalonFX motor;
  private StatusSignal<Double> statusSignal;

  public SelfCheckingPhoenixMotor(String label, TalonFX motor) {
    this.label = label;
    this.motor = motor;
    this.statusSignal = this.motor.getSupplyVoltage().clone();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (motor.getFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware failure detected", label)));
    }
    if (motor.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (motor.getFault_DeviceTemp().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: Device temperature exceeded limit", label), true));
    }
    if (motor.getFault_FusedSensorOutOfSync().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Remote sensor is out of sync", label)));
    }
    if (motor.getFault_OverSupplyV().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Supply voltage exceeded limit", label)));
    }
    if (motor.getFault_ProcTemp().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Processor temperature exceeded limit", label)));
    }
    if (motor.getFault_Undervoltage().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device supply voltage near brownout", label)));
    }
    if (motor.getFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Unlicensed feature in use", label)));
    }
    if (motor.getFault_UnstableSupplyV().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Supply voltage is unstable", label)));
    }
    if (motor.getFault_BridgeBrownout().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Bridge brownout detected", label)));
    }
    if (motor.getFault_RemoteSensorReset().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Remote sensor reset", label)));
    }
    if (motor.getFault_MissingDifferentialFX().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Missing Differential FX", label)));
    }
    if (motor.getFault_RemoteSensorPosOverflow().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Remote sensor position overflow", label)));
    }
    if (motor.getFault_ReverseHardLimit().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Reverse hard limit reached", label)));
    }
    if (motor.getFault_ForwardHardLimit().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Forward hard limit reached", label)));
    }
    if (motor.getFault_ReverseSoftLimit().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Reverse soft limit reached", label)));
    }
    if (motor.getFault_ForwardSoftLimit().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Forward soft limit reached", label)));
    }
    if (motor.getFault_RemoteSensorDataInvalid().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Remote sensor data invalid", label)));
    }
    if (motor.getFault_StatorCurrLimit().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Stator current limit exceeded", label)));
    }
    if (motor.getFault_SupplyCurrLimit().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Supply current limit exceeded", label)));
    }
    if (motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Fused CANcoder used without license", label)));
    }
    if (motor.getFault_StaticBrakeDisabled().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Static brake disabled", label)));
    }

    this.statusSignal.refresh();
    if (this.statusSignal.getStatus() != StatusCode.OK) {
      faults.add(new SubsystemFault(String.format("[%s]: device is unreachable", label)));
    }

    return faults;
  }
}
