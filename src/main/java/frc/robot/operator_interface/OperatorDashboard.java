package frc.robot.operator_interface;

import org.wpilib.command2.button.Trigger;
import frc.lib.team6328.util.LoggedTunableBoolean;

/**
 * OperatorDashboard is a class that implements the OperatorInterface. It is not a joystick,
 * controller, or physical button panel. Instead, it is a software-based dashboard with virtual
 * buttons. It is designed to be used by the operator on a handheld touchscreen running Elastic. For
 * a more sophisticated example, including implementing groups of buttons where only one may be
 * selected at a time, refer to the 2025 Huskie Robotics code base.
 */
public class OperatorDashboard implements OperatorInterface {
  // the readAndWrite argument must be true or else the dashboard will not work when tuning mode is
  // disabled.
  public final LoggedTunableBoolean enableVision =
      new LoggedTunableBoolean("operatorDashboard/Enable Vision", true, true);

  public final LoggedTunableBoolean enablePrimaryIRSensors =
      new LoggedTunableBoolean("operatorDashboard/Enable Primary IR Sensors", true, true);

  public final LoggedTunableBoolean autoSnapsEnabled =
      new LoggedTunableBoolean("operatorDashboard/Auto Snaps Enabled", true, true);

  public final LoggedTunableBoolean simulateCollisionButton =
      new LoggedTunableBoolean("operatorDashboard/Simulate Collision", false, true);

  public OperatorDashboard() {}

  @Override
  public Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> enableVision.get());
  }

  @Override
  public Trigger getEnablePrimaryIRSensorsTrigger() {
    return new Trigger(() -> enablePrimaryIRSensors.get());
  }

  @Override
  public Trigger getAutoSnapsEnabledTrigger() {
    return new Trigger(() -> autoSnapsEnabled.get());
  }

  @Override
  public Trigger getSimulateCollisionButton() {
    return new Trigger(() -> simulateCollisionButton.get());
  }
}
