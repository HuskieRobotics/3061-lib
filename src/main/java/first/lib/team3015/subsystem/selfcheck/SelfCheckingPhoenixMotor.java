package first.lib.team3015.subsystem.selfcheck;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import first.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Voltage;

public class SelfCheckingPhoenixMotor implements SelfChecking {
  // tolerance for comparing configuration values read from the motor
  private static final double CONFIG_TOLERANCE = 0.01;

  private final String label;
  private final TalonFX motor;
  private StatusSignal<Voltage> statusSignal;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final List<SubsystemFault> faults = new ArrayList<>();
  // null until the motor's current limits have been read
  private List<SubsystemFault> currentLimitFaults = null;

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

    faults.addAll(checkCurrentLimits());

    return faults;
  }

  /**
   * Checks that the motor's supply and stator current limits are enabled and have been set. Phoenix
   * enables both limits by default; so, a motor whose current limits were never configured has
   * Phoenix's default limits rather than no limits. Disabled limits are reported as errors, and
   * limits equal to Phoenix's defaults are reported as warnings since they were probably never set.
   * The stator current limit also applies to torque current control (in addition to the peak torque
   * currents); so, these checks apply to every control mode.
   *
   * <p>The configuration is read from the motor, which blocks until the motor responds. So, it is
   * only read until it is read successfully; after that, the same faults are reported for every
   * check. Faults are only checked while the robot is disabled.
   *
   * @return the faults related to the motor's current limits
   */
  private List<SubsystemFault> checkCurrentLimits() {
    if (currentLimitFaults != null) {
      return currentLimitFaults;
    }

    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    if (!motor.getConfigurator().refresh(config).isOK()) {
      // try again during the next check; if the motor is unreachable, that is reported separately
      return List.of();
    }

    CurrentLimitsConfigs defaults = new CurrentLimitsConfigs();
    currentLimitFaults = new ArrayList<>();

    if (!config.SupplyCurrentLimitEnable) {
      currentLimitFaults.add(
          new SubsystemFault(String.format("[%s]: Supply current limit is disabled", label)));
    } else if (MathUtil.isNear(
            defaults.SupplyCurrentLimit, config.SupplyCurrentLimit, CONFIG_TOLERANCE)
        && MathUtil.isNear(
            defaults.SupplyCurrentLowerLimit, config.SupplyCurrentLowerLimit, CONFIG_TOLERANCE)
        && MathUtil.isNear(
            defaults.SupplyCurrentLowerTime, config.SupplyCurrentLowerTime, CONFIG_TOLERANCE)) {
      currentLimitFaults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: Supply current limit is Phoenix's default (%.0f A, %.0f A after %.1f s);"
                      + " was it set?",
                  label,
                  defaults.SupplyCurrentLimit,
                  defaults.SupplyCurrentLowerLimit,
                  defaults.SupplyCurrentLowerTime),
              true));
    }

    if (!config.StatorCurrentLimitEnable) {
      currentLimitFaults.add(
          new SubsystemFault(String.format("[%s]: Stator current limit is disabled", label)));
    } else if (MathUtil.isNear(
        defaults.StatorCurrentLimit, config.StatorCurrentLimit, CONFIG_TOLERANCE)) {
      currentLimitFaults.add(
          new SubsystemFault(
              String.format(
                  "[%s]: Stator current limit is Phoenix's default (%.0f A); was it set?",
                  label, defaults.StatorCurrentLimit),
              true));
    }

    return currentLimitFaults;
  }

  @Override
  public void clearStickyFaults() {
    motor.clearStickyFaults();
  }
}
