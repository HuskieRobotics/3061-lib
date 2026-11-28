package first.lib.util;

import java.util.HashMap;
import java.util.Map;
import org.wpilib.command2.Command;
import org.wpilib.command2.SelectCommand;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.command2.sysid.SysIdRoutine.Direction;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

/**
 * Singleton class to manage SysId routines for dynamic and quasistatic testing. Each subsystem that
 * needs to be characterized registers its SysId routines with this class. It uses a
 * LoggedDashboardChooser to allow users to select the desired subsystem's SysId routine from the
 * dashboard.
 */
public class SysIdRoutineChooser {

  private static final SysIdRoutineChooser instance = new SysIdRoutineChooser();

  private Map<Integer, Command> dynamicForwardRoutines = new HashMap<>();
  private Map<Integer, Command> dynamicReverseRoutines = new HashMap<>();
  private Map<Integer, Command> quasistaticForwardRoutines = new HashMap<>();
  private Map<Integer, Command> quasistaticReverseRoutines = new HashMap<>();
  private Command dynamicForwardSelectorCommand = null;
  private Command dynamicBackupSelectorCommand = null;
  private Command quasistaticForwardSelectorCommand = null;
  private Command quasistaticBackwardSelectorCommand = null;

  private final Selectable<Integer> sysIdChooser = new Selectable<>();

  private SysIdRoutineChooser() {}

  public static SysIdRoutineChooser getInstance() {
    return instance;
  }

  public void addOption(String name, SysIdRoutine sysIdRoutine) {
    sysIdChooser.add(name, name.hashCode());
    dynamicForwardRoutines.put(name.hashCode(), sysIdRoutine.dynamic(Direction.FORWARD));
    dynamicReverseRoutines.put(name.hashCode(), sysIdRoutine.dynamic(Direction.REVERSE));
    quasistaticForwardRoutines.put(name.hashCode(), sysIdRoutine.quasistatic(Direction.FORWARD));
    quasistaticReverseRoutines.put(name.hashCode(), sysIdRoutine.quasistatic(Direction.REVERSE));
    Tunables.publish("SysId Selection", sysIdChooser);
  }

  public Command getDynamicForward() {
    if (dynamicForwardSelectorCommand == null) {
      dynamicForwardSelectorCommand =
          new SelectCommand<>(this.dynamicForwardRoutines, sysIdChooser::getSelected);
    }
    return dynamicForwardSelectorCommand;
  }

  public Command getDynamicReverse() {
    if (dynamicBackupSelectorCommand == null) {
      dynamicBackupSelectorCommand =
          new SelectCommand<>(this.dynamicReverseRoutines, sysIdChooser::getSelected);
    }

    return dynamicBackupSelectorCommand;
  }

  public Command getQuasistaticForward() {
    if (quasistaticForwardSelectorCommand == null) {
      quasistaticForwardSelectorCommand =
          new SelectCommand<>(this.quasistaticForwardRoutines, sysIdChooser::getSelected);
    }

    return quasistaticForwardSelectorCommand;
  }

  public Command getQuasistaticReverse() {
    if (quasistaticBackwardSelectorCommand == null) {
      quasistaticBackwardSelectorCommand =
          new SelectCommand<>(this.quasistaticReverseRoutines, sysIdChooser::getSelected);
    }
    return quasistaticBackwardSelectorCommand;
  }
}
