package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.util.LoggedTunableBoolean;

public class OperatorDashboard implements OperatorInterface {
  public final LoggedTunableBoolean enableVision =
      new LoggedTunableBoolean("operatorDashboard/Enable Vision", true, true);

  public final LoggedTunableBoolean enablePrimaryIRSensors =
      new LoggedTunableBoolean("operatorDashboard/Enable Primary IR Sensors", true, true);

  public final LoggedTunableBoolean level1 =
      new LoggedTunableBoolean("operatorDashboard/Level 1", false, true);
  public final LoggedTunableBoolean level2 =
      new LoggedTunableBoolean("operatorDashboard/Level 2", false, true);
  public final LoggedTunableBoolean level3 =
      new LoggedTunableBoolean("operatorDashboard/Level 3 ", false, true);
  public final LoggedTunableBoolean level4 =
      new LoggedTunableBoolean("operatorDashboard/Level 4 ", true, true);

  public final LoggedTunableBoolean highAlgaeRemoval =
      new LoggedTunableBoolean("operatorDashboard/High Algae Removal", false, true);
  public final LoggedTunableBoolean lowAlgaeRemoval =
      new LoggedTunableBoolean("operatorDashboard/Low Algae Removal", false, true);

  public OperatorDashboard() {}

  @Override
  public Trigger getVisionIsEnabledTrigger() {
    return new Trigger(() -> enableVision.get());
  }
}
