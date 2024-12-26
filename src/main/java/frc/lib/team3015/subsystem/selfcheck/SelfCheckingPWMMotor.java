package frc.lib.team3015.subsystem.selfcheck;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import frc.lib.team3015.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPWMMotor implements SelfChecking {
  private final String label;
  private final PWMMotorController motor;

  public SelfCheckingPWMMotor(String label, PWMMotorController motor) {
    this.label = label;
    this.motor = motor;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (!motor.isAlive()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device timed out", label)));
    }

    return faults;
  }

  @Override
  public void clearStickyFaults() {
    // PWM motor doesn't have any sticky faults to clear
  }
}
