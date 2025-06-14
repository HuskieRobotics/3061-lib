package frc.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPhoenixMotor implements SelfChecking {
  private final String label;
  private final TalonFX motor;
  private StatusSignal<Voltage> statusSignal;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final List<SubsystemFault> faults = new ArrayList<>();

  public SelfCheckingPhoenixMotor(String label, TalonFX motor) {
    this.label = label;
    this.motor = motor;
    this.statusSignal = this.motor.getSupplyVoltage().clone();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    faults.clear();

    // faults
    if (motor.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (motor.getFault_BridgeBrownout().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: Bridge was disabled most likely due to supply voltage dropping too low",
                  label)));
    }
    if (motor.getFault_DeviceTemp().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: Device temperature exceeded limit", label), true));
    }
    if (motor.getFault_FusedSensorOutOfSync().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Remote sensor is out of sync", label)));
    }
    if (motor.getFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware failure detected", label)));
    }
    if (motor.getFault_MissingDifferentialFX().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: The remote Talon used for differential control is not present", label)));
    }
    if (motor.getFault_MissingHardLimitRemote().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: The remote limit switch device is not present", label)));
    }
    if (motor.getFault_MissingSoftLimitRemote().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: The remote soft limit device is not present", label)));
    }
    if (motor.getFault_OverSupplyV().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: Supply voltage exceeded limit", label)));
    }
    if (motor.getFault_ProcTemp().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Processor temperature exceeded limit", label)));
    }
    if (motor.getFault_RemoteSensorDataInvalid().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: The remote sensor's data is no longer trusted", label)));
    }
    if (motor.getFault_RemoteSensorPosOverflow().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: The remote sensor position has overflowed", label)));
    }
    if (motor.getFault_RemoteSensorReset().getValue() == Boolean.TRUE) {
      faults.add(new SubsystemFault(String.format("[%s]: The remote sensor has reset", label)));
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

    // sticky faults
    if (motor.getStickyFault_BootDuringEnable().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Device booted while enabled", label)));
    }
    if (motor.getStickyFault_BridgeBrownout().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: [STICKY] Bridge was disabled most likely due to supply voltage dropping too low",
                  label)));
    }
    if (motor.getStickyFault_DeviceTemp().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] Device temperature exceeded limit", label), true));
    }
    if (motor.getStickyFault_FusedSensorOutOfSync().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Remote sensor is out of sync", label)));
    }
    if (motor.getStickyFault_Hardware().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Hardware failure detected", label)));
    }
    if (motor.getStickyFault_MissingDifferentialFX().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: [STICKY] The remote Talon used for differential control is not present",
                  label)));
    }
    if (motor.getStickyFault_MissingHardLimitRemote().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: [STICKY] The remote limit switch device is not present", label)));
    }
    if (motor.getStickyFault_MissingSoftLimitRemote().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] The remote soft limit device is not present", label)));
    }
    if (motor.getStickyFault_OverSupplyV().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Supply voltage exceeded limit", label)));
    }
    if (motor.getStickyFault_ProcTemp().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] Processor temperature exceeded limit", label)));
    }
    if (motor.getStickyFault_RemoteSensorDataInvalid().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: [STICKY] The remote sensor's data is no longer trusted", label)));
    }
    if (motor.getStickyFault_RemoteSensorPosOverflow().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] The remote sensor position has overflowed", label)));
    }
    if (motor.getStickyFault_RemoteSensorReset().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] The remote sensor has reset", label)));
    }
    if (motor.getStickyFault_Undervoltage().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(
              String.format("[%s]: [STICKY] Device supply voltage near brownout", label)));
    }
    if (motor.getStickyFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Unlicensed feature in use", label)));
    }
    if (motor.getStickyFault_UnstableSupplyV().getValue() == Boolean.TRUE) {
      faults.add(
          new SubsystemFault(String.format("[%s]: [STICKY] Supply voltage is unstable", label)));
    }

    this.statusSignal.refresh();
    if (!connectedDebounce.calculate(this.statusSignal.getStatus().isOK())) {
      faults.add(new SubsystemFault(String.format("[%s]: [STICKY] device is unreachable", label)));
    }

    return faults;
  }

  @Override
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }
}
